#include <memory>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <gpn_msgs/msg/fish_cmd.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#ifndef GPN_COORDINATOR_COMPONENT
#define GPN_COORDINATOR_COMPONENT

namespace gpn {

    class coordinator : public rclcpp::Node {

        private:

            int num_param_test;
            std::string odometry_topic_name;
            std::string fish_cmd_topic_name;

            std::string odometry_topic_param;
            std::string fish_cmd_topic_param;

            geometry_msgs::msg::Pose curr_pose;
            geometry_msgs::msg::Twist curr_twist;

            bool waiting_for_init_pose;
            geometry_msgs::msg::Pose init_pose;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
            rclcpp::Subscription<gpn_msgs::msg::FishCmd>::SharedPtr sub_fish_cmd;

        protected:

            void odometry_callback(const nav_msgs::msg::Odometry& odom_msg);
            void fish_command_callback(const gpn_msgs::msg::FishCmd& cmd_msg);

        public:

            coordinator(const std::string& name, const rclcpp::NodeOptions& options);
            ~coordinator();
            void intialize_params();
            void configure();

    };

}

#endif
