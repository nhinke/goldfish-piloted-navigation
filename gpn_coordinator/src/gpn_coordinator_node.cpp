#include <string>
#include <rclcpp/rclcpp.hpp>
#include <gpn_coordinator/gpn_coordinator.hpp>

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    std::string name = "gpn_coordinator_node";
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;
    auto gpn_coordinator_node = std::make_shared<gpn::coordinator>(name,options);

    exec.add_node(gpn_coordinator_node);
    exec.spin();
    
    rclcpp::shutdown();
    return 0;

}
