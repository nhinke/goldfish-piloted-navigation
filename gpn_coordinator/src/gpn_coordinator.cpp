#include <memory>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "gpn_msgs/msg/fish_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class gpn_coordinator : public rclcpp::Node {

    public:

        gpn_coordinator() : Node("gpn_coordinator") {

            std::string odometry_topic_name;
            std::string odometry_topic_param = "odometry_topic";
            this->get_parameter_or(odometry_topic_param, odometry_topic_name, std::string("/odom"));
     
            std::string fish_command_topic_name;
            std::string fish_command_topic_param = "fish_command_topic";
            this->get_parameter_or(fish_command_topic_param, fish_command_topic_name, std::string("/fish_cmd"));
            
            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name, 10, std::bind(&gpn_coordinator::odometry_callback, this, _1));
            sub_fish_cmd = this->create_subscription<gpn_msgs::msg::FishCmd>(fish_command_topic_name, 10, std::bind(&gpn_coordinator::fish_command_callback, this, _1));

        }

        ~gpn_coordinator() {}

    private:

        void odometry_callback(const nav_msgs::msg::Odometry& odom_msg) {

            curr_pose = odom_msg.pose.pose;
            curr_twist = odom_msg.twist.twist;
            std::cout << "curr lin vel:  [ " << curr_twist.linear.x << " " << curr_twist.linear.y << " " << curr_twist.linear.z << " ]" << std::endl;
            
        }
        
        void fish_command_callback(const gpn_msgs::msg::FishCmd& cmd_msg) {
            
            double cmd_heading = cmd_msg.heading;
            double cmd_magnitude = cmd_msg.magnitude;    
            std::cout << "fish command:  " << cmd_heading << "  " << cmd_magnitude << std::endl;
        
        }

        // double curr_vel;
        // double curr_heading;

        geometry_msgs::msg::Pose curr_pose;
        geometry_msgs::msg::Twist curr_twist;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Subscription<gpn_msgs::msg::FishCmd>::SharedPtr sub_fish_cmd;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gpn_coordinator>());
    rclcpp::shutdown();
    return 0;
}
