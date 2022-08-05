#include <string>
#include <rclcpp/rclcpp.hpp>
#include <gpn_pid_controller/gpn_pid_controller.hpp>

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    std::string name = "gpn_pid_controller_node";
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;
    auto gpn_pid_controller_node = std::make_shared<gpn::pid_controller>(name,options);

    exec.add_node(gpn_pid_controller_node);
    exec.spin();
    
    rclcpp::shutdown();
    return 0;

}
