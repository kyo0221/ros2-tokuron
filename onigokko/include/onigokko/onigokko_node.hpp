#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cstddef>
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
    void autonomousCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void computeVel(const sensor_msgs::msg::PointCloud2 & msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    const std::string cloud_topic_;
    const double max_vel_;
    const double ignore_range_;
    const double max_range_;
    const double min_z_;
    const double max_z_;
    const std::size_t direction_count_;

    bool autonomous_flag_;
    double target_angle_;
    double last_cmd_angle_;
    bool has_last_cmd_angle_;

    rclcpp::QoS _qos;
};

}
