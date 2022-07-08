#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pid_controller/pid_controller.hpp>

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    std::string name = "gpn_pid_controller_node";
    rclcpp::NodeOptions options;

    auto gpn_pid_controller_node = std::make_shared<gpn::pid_controller>(name,options);
    rclcpp::spin(gpn_pid_controller_node);
    
    rclcpp::shutdown();
    return 0;

}
