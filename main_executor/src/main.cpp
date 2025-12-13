#include <rclcpp/rclcpp.hpp>
#include "onigokko/onigokko_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto onigokko_node = std::make_shared<onigokko::OnigokkoNode>(nodes_option);

    exec.add_node(onigokko_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
