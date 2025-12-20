#include "onigokko/onigokko_node.hpp"

#include <cmath>
#include <functional>
#include <limits>

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

    double max_distance = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;

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

        if (distance > max_distance)
        {
            max_distance = distance;
            max_x = x;
            max_y = y;
        }

    }

    target_angle_ = std::atan2(max_y, max_x);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = max_vel_ * std::cos(target_angle_);
    cmd_vel.linear.y = max_vel_ * std::sin(target_angle_);
    cmd_vel.angular.z = 0.0;

    cmd_vel_pub_->publish(cmd_vel);
}

}
