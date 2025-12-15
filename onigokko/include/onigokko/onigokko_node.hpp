#pragma once

#include <rclcpp/rclcpp.hpp>
#include "onigokko/visibility_control.hpp"

namespace onigokko{

class OnigokkoNode : public rclcpp::Node {
public:
    ONIGOKKO_PUBLIC
    explicit OnigokkoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ONIGOKKO_PUBLIC
    explicit OnigokkoNode(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void ComputeVel(const sensor_msgs::msg::PointCloud2 & msg, geometry_msgs)::
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autonomous_pub_;

    const double max_vel_;
    const double ignore_range_;
    bool autonomous_flag_;

    rclcpp::QoS _qos;
};

}
