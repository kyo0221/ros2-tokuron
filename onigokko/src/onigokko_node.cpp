#include "onigokko/onigokko_node.hpp"

#include <cmath>
#include <functional>
#include <limits>

namespace onigokko{

OnigokkoNode::OnigokkoNode(const rclcpp::NodeOptions& options) : OnigokkoNode("", options) {}

OnigokkoNode::OnigokkoNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("onigokko_node", name_space, options),
  cloud_topic_(this->declare_parameter<std::string>("cloud_topic", "/livox/lidar")),
  max_vel_(this->declare_parameter<double>("max_vel", 0.5)),
  ignore_range_(this->declare_parameter<double>("ignore_range", 0.3)),
  min_z_(this->declare_parameter<double>("min_z", -0.2)),
  max_z_(this->declare_parameter<double>("max_z", 1.0)),
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

    double min_distance = std::numeric_limits<double>::max();
    double min_x = 0.0;
    double min_y = 0.0;

    for (sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x"), it_y(msg, "y"), it_z(msg, "z");
         it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
        const float x = *it_x;
        const float y = *it_y;
        const float z = *it_z;

        const double distance = std::sqrt(x * x + y * y);
        if (distance < ignore_range_ || z < min_z_ || z > max_z_)
        {
            continue;
        }

        if (distance < min_distance)
        {
            min_distance = distance;
            min_x = x;
            min_y = y;
        }
    }

    if (min_distance == std::numeric_limits<double>::max())
    {
        return;
    }

    target_angle_ = std::atan2(min_y, min_x) + M_PI;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = max_vel_ * std::cos(target_angle_);
    cmd_vel.linear.y = max_vel_ * std::sin(target_angle_);
    cmd_vel.angular.z = 0.0;

    cmd_vel_pub_->publish(cmd_vel);
}

}
