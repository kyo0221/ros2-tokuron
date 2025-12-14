#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include "teleop_omni/visibility_control.hpp"

namespace teleop_omni{

enum class Button {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    L1 = 4,
    R1 = 5,
    Back = 6,
    Start = 7,
    LPress = 9,
};

class TeleopOmniNode : public rclcpp::Node {
public:
    TELEOP_OMNI_PUBLIC
    explicit TeleopOmniNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    TELEOP_OMNI_PUBLIC
    explicit TeleopOmniNode(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autonomous_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr restart_pub_;

    const double linear_max_vel_;
    const double angular_max_vel_;

    bool autonomous_flag_;
    bool prev_auto_button_;

    rclcpp::QoS _qos;
};

}
