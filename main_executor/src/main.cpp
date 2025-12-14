#include <rclcpp/rclcpp.hpp>
#include "chassis_driver/chassis_driver_node.hpp"
#include "socketcan_interface/socketcan_interface_node.hpp"
#include "onigokko/onigokko_node.hpp"
#include "teleop_omni/teleop_omni_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto onigokko_node = std::make_shared<onigokko::OnigokkoNode>(nodes_option);
    auto teleop_omni_node = std::make_shared<teleop_omni::TeleopOmniNode>(nodes_option);
    auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);

    exec.add_node(socketcan_node);
    exec.add_node(chassis_driver_node);
    exec.add_node(onigokko_node);
    exec.add_node(teleop_omni_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
