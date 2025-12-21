#include "onigokko/onigokko_node.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <vector>  // ★ 追加

namespace onigokko{

OnigokkoNode::OnigokkoNode(const rclcpp::NodeOptions& options) : OnigokkoNode("", options) {}

OnigokkoNode::OnigokkoNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("onigokko_node", name_space, options),
  cloud_topic_(this->get_parameter("cloud_topic").as_string()),
  max_vel_(this->get_parameter("linear_max.vel").as_double()),
  ignore_range_(this->get_parameter("ignore_range").as_double()),
  min_z_(this->get_parameter("min_z").as_double()),
  max_z_(this->get_parameter("max_z").as_double()),
  autonomous_flag_(false),
  target_angle_(0.0),
  _qos(rclcpp::QoS(10))
{
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&OnigokkoNode::cloudCallback, this, std::placeholders::_1));

    autonomous_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/autonomous",
        _qos,
        std::bind(&OnigokkoNode::autonomousCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", _qos);

    RCLCPP_INFO(this->get_logger(), "Onigokko Node has been started. cloud_topic: %s", cloud_topic_.c_str());
}

void OnigokkoNode::autonomousCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    autonomous_flag_ = msg->data;
}

void OnigokkoNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    computeVel(*msg);
}

void OnigokkoNode::computeVel(const sensor_msgs::msg::PointCloud2 & msg)
{
    if(!autonomous_flag_){
        return;
    }

    const int sector_count = 36;
    const double blend_open = 0.35;       // 0:逃避のみ, 1:空き方向のみ（0.2~0.5推奨）
    const double angle_smooth = 0.25;     // 角度平滑化（0.15~0.35推奨）
    const double max_clear_cap = 5.0;     // “点が無い=無限”扱いを上限で丸める
    const double eps = 1e-6;              // ★ 同率判定用（浮動小数誤差対策）

    auto wrap = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    const double sector_rad = (2.0 * M_PI) / static_cast<double>(sector_count);
    std::vector<double> sector_min_dist(sector_count, std::numeric_limits<double>::infinity());

    // 最近傍点
    double nearest_d = std::numeric_limits<double>::infinity();
    double nearest_ang = 0.0;

    for (sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x"), it_y(msg, "y"), it_z(msg, "z");
         it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
        const float x = *it_x;
        const float y = *it_y;
        const float z = *it_z;

        const double d = std::sqrt(x * x + y * y);
        if (d < ignore_range_ || z < min_z_ || z > max_z_) {
            continue;
        }

        const double ang = std::atan2(y, x);  // [-pi, pi]

        int idx = static_cast<int>(std::floor((ang + M_PI) / sector_rad));
        idx = std::max(0, std::min(sector_count - 1, idx));

        if (d < sector_min_dist[idx]) {
            sector_min_dist[idx] = d;
        }

        if (d < nearest_d) {
            nearest_d = d;
            nearest_ang = ang;
        }
    }

    if (!std::isfinite(nearest_d)) {
        return;
    }

    // 1) 逃避方向：最近傍点の反対
    const double flee_ang = wrap(nearest_ang + M_PI);

    // 2) 空き方向：連続した最大セクタの中央
    auto capped_clear = [&](int i) -> double {
        double c = sector_min_dist[i];
        if (!std::isfinite(c)) c = max_clear_cap;
        if (c > max_clear_cap) c = max_clear_cap;
        return c;
    };

    // 最大クリアランス値
    double max_clear = -1.0;
    for (int i = 0; i < sector_count; ++i) {
        max_clear = std::max(max_clear, capped_clear(i));
    }
    if (max_clear < 0.0) {
        return;
    }

    // リング（0と最後がつながる）に対応するため2周走査で最長連続区間を探す
    int best_start = -1;
    int best_len = 0;

    int cur_start = -1;
    int cur_len = 0;

    for (int k = 0; k < sector_count * 2; ++k) {
        const int i = k % sector_count;
        const double c = capped_clear(i);

        // 「最大と同率」を許容（浮動小数誤差対策）
        const bool is_max = (c >= max_clear - eps);

        if (is_max) {
            if (cur_start < 0) cur_start = k;
            cur_len++;
        } else {
            if (cur_start >= 0) {
                const int len = std::min(cur_len, sector_count);  // 2周分を1周に制限
                if (len > best_len) {
                    best_len = len;
                    best_start = cur_start;
                }
                cur_start = -1;
                cur_len = 0;
            }
        }
    }

    // 末尾が最大区間のまま終わった場合
    if (cur_start >= 0) {
        const int len = std::min(cur_len, sector_count);
        if (len > best_len) {
            best_len = len;
            best_start = cur_start;
        }
    }

    // 区間が見つからないことは基本ないが念のため
    if (best_start < 0 || best_len <= 0) {
        return;
    }

    // 最長区間の中央（偶数長は中央が2つあるが、ここでは前寄りの中央を採用）
    const int center_k = best_start + (best_len - 1) / 2;
    const int center_i = center_k % sector_count;

    const double open_ang = wrap(-M_PI + (center_i + 0.5) * sector_rad);

    // 3) flee と open を角度として安全にブレンド
    const double diff_open = wrap(open_ang - flee_ang);
    const double desired_ang = wrap(flee_ang + blend_open * diff_open);

    // 角度をスムージング（ジッター低減）
    const double diff = wrap(desired_ang - target_angle_);
    target_angle_ = wrap(target_angle_ + angle_smooth * diff);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = max_vel_ * std::cos(target_angle_);
    cmd_vel.linear.y = max_vel_ * std::sin(target_angle_);
    cmd_vel.angular.z = 0.0;

    cmd_vel_pub_->publish(cmd_vel);
}

}
