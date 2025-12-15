#include "onigokko/onigokko_node.hpp"

namespace onigokko{

OnigokkoNode::OnigokkoNode(const rclcpp::NodeOptions& options) : OnigokkoNode("", options) {}

OnigokkoNode::OnigokkoNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("onigokko_node", name_space, options),
this->get_parameter("cloud_topic").as_string(),
max_vel_(this->get_parameter("max_vel").as_double()),
ignore_range_(this->get_parameter("ignore_range").as_double()),
autonomous_flag_(false),
_qos(rclcpp::QoS(10))
{
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        rclcpp::SensorDataQoS(),
        std::bind(&OnigokkoNode::cloudCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel" _qos);
    autonomous_flag_ = this->create_publisher<std_msgs::msg::Bool>("autonomous" _qos);

    auto autonomous_msg = std_msgs::msg::Bool();
    autonomous_msg.data = autonomous_flag_;
    autonomous_pub_->publish(autonomous_msg);

    RCLCPP_INFO(this->get_logger(), "Onigokko Node has been started. dummy_parameter: %.2f");
}

void OnigokkoNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    geometry_msgs::msg::Twist twist_msg;
    if (autonomous_flag_ == true){
        ComputeVel(*msg, twist_msg)
        cmd_vel_pub_->publish(twist_msg);
    } else {
        return;
    }
}

void OnigokkoNode::ComputeVel(const sensor_msgs::msg::PointCloud2 & msg, geometry_msgs::msg::Twist & out_vel)
{
    
}
