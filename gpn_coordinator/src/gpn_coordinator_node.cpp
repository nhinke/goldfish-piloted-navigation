#include <string>
#include <rclcpp/rclcpp.hpp>
#include <gpn_coordinator/gpn_coordinator.hpp>

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    std::string name = "gpn_coordinator_node";
    rclcpp::NodeOptions options;

    auto gpn_coordinator_node = std::make_shared<gpn::coordinator>(name,options);
    rclcpp::spin(gpn_coordinator_node);
    
    rclcpp::shutdown();
    return 0;

}
