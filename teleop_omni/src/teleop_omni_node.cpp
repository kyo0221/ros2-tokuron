#include "teleop_omni/teleop_omni_node.hpp"

namespace teleop_omni{

TeleopOmniNode::TeleopOmniNode(const rclcpp::NodeOptions& options) : TeleopOmniNode("", options) {}

TeleopOmniNode::TeleopOmniNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("teleop_omni_node", name_space, options),
linear_max_vel_(this->get_parameter("linear_max.vel").as_double()),
angular_max_vel_(this->get_parameter("angular_max.vel").as_double() * M_PI / 180.0),
autonomous_flag_(false),
prev_start_button_(false),
_qos(rclcpp::QoS(10))
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", _qos, std::bind(&TeleopOmniNode::joyCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", _qos);
    autonomous_pub_ = this->create_publisher<std_msgs::msg::Bool>("/autonomous", _qos);

    auto autonomous_msg = std_msgs::msg::Bool();
    autonomous_msg.data = autonomous_flag_;
    autonomous_pub_->publish(autonomous_msg);

    RCLCPP_INFO(this->get_logger(), "Teleop Omni Node has been started.");
}

void TeleopOmniNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    bool start_button = msg->buttons[static_cast<int>(Button::Start)];
    if(start_button && !prev_start_button_){
        autonomous_flag_ = !autonomous_flag_;
        auto autonomous_msg = std_msgs::msg::Bool();
        autonomous_msg.data = autonomous_flag_;
        autonomous_pub_->publish(autonomous_msg);

        if(autonomous_flag_){
            RCLCPP_INFO(this->get_logger(), "自律フラグtrue");
        } else {
            RCLCPP_INFO(this->get_logger(), "自律フラグfalse");
        }
    }
    prev_start_button_ = start_button;

    if(!autonomous_flag_){
        auto twist_msg = geometry_msgs::msg::Twist();

        twist_msg.linear.x = msg->axes[1] * linear_max_vel_;
        twist_msg.linear.y = msg->axes[0] * linear_max_vel_;
        twist_msg.angular.z = msg->axes[3] * angular_max_vel_;

        cmd_vel_pub_->publish(twist_msg);
    }
}

}
