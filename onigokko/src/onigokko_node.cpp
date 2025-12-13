#include "onigokko/onigokko_node.hpp"

namespace onigokko{

OnigokkoNode::OnigokkoNode(const rclcpp::NodeOptions& options) : OnigokkoNode("", options) {}

OnigokkoNode::OnigokkoNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("onigokko_node", name_space, options),
dummy_parameter(this->declare_parameter("dummy_parameter", 0.0))
{
    RCLCPP_INFO(this->get_logger(), "Onigokko Node has been started. dummy_parameter: %.2f", dummy_parameter);
}

}
