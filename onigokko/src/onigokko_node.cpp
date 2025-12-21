#include "onigokko/onigokko_node.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <algorithm>
#include <vector>

namespace onigokko{

OnigokkoNode::OnigokkoNode(const rclcpp::NodeOptions& options) : OnigokkoNode("", options) {}

OnigokkoNode::OnigokkoNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("onigokko_node", name_space, options),
  cloud_topic_(this->get_parameter("cloud_topic").as_string()),
  max_vel_(this->get_parameter("linear_max.vel").as_double()),
  ignore_range_(this->get_parameter("ignore_range").as_double()),
  max_range_(this->get_parameter("max_range").as_double()),
  min_z_(this->get_parameter("min_z").as_double()),
  max_z_(this->get_parameter("max_z").as_double()),
  direction_count_(this->get_parameter("direction_count").as_int()),
  autonomous_flag_(false),
  target_angle_(0.0),
  last_cmd_angle_(0.0),
  has_last_cmd_angle_(false),
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

    const double sector_width = 2.0 * M_PI / static_cast<double>(direction_count_);
    std::vector<double> min_distances(direction_count_, std::numeric_limits<double>::max());

    auto normalizeAngle = [](double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    };

    auto angleToDirectionIndex = [&](double angle) -> std::size_t {
        double wrapped_angle = angle + M_PI;
        while (wrapped_angle < 0.0) {
            wrapped_angle += 2.0 * M_PI;
        }
        while (wrapped_angle >= 2.0 * M_PI) {
            wrapped_angle -= 2.0 * M_PI;
        }
        const std::size_t index = static_cast<std::size_t>(wrapped_angle / sector_width);
        return std::min(index, direction_count_ - 1);
    };

    for (sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x"), it_y(msg, "y"), it_z(msg, "z");
         it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
        const float x = *it_x;
        const float y = *it_y;
        const float z = *it_z;

        const double distance = std::sqrt(x * x + y * y);
        if (distance < ignore_range_ || distance > max_range_ || z < min_z_ || z > max_z_)
        {
            continue;
        }

        const double angle = std::atan2(y, x);
        const std::size_t direction_index = angleToDirectionIndex(angle);
        min_distances[direction_index] = std::min(min_distances[direction_index], distance);
    }

    const auto closest_iter = std::min_element(min_distances.begin(), min_distances.end());
    if (closest_iter == min_distances.end() || *closest_iter == std::numeric_limits<double>::max()) {
        return;
    }

    std::size_t closest_direction = static_cast<std::size_t>(std::distance(min_distances.begin(), closest_iter));

    if (has_last_cmd_angle_) {
        const double candidate_direction_center =
            (static_cast<double>(closest_direction) + 0.5) * sector_width - M_PI;
        const double candidate_target_angle = normalizeAngle(candidate_direction_center + M_PI);
        const double diff = std::fabs(normalizeAngle(candidate_target_angle - last_cmd_angle_));
        if (std::fabs(diff - M_PI) <= (sector_width / 2.0)) {
            double next_min_distance = std::numeric_limits<double>::max();
            std::size_t next_direction = closest_direction;
            for (std::size_t i = 0; i < min_distances.size(); ++i) {
                if (i == closest_direction) {
                    continue;
                }
                if (min_distances[i] < next_min_distance) {
                    next_min_distance = min_distances[i];
                    next_direction = i;
                }
            }
            if (next_min_distance != std::numeric_limits<double>::max()) {
                closest_direction = next_direction;
            }
        }
    }

    const double direction_center = (static_cast<double>(closest_direction) + 0.5) * sector_width - M_PI;
    target_angle_ = std::atan2(std::sin(direction_center + M_PI), std::cos(direction_center + M_PI));

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = max_vel_ * std::cos(target_angle_);
    cmd_vel.linear.y = max_vel_ * std::sin(target_angle_);
    cmd_vel.angular.z = 0.0;

    cmd_vel_pub_->publish(cmd_vel);
    last_cmd_angle_ = target_angle_;
    has_last_cmd_angle_ = true;
}

}
