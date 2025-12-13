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
    const double dummy_parameter;
    rclcpp::QoS _qos;
};

}
