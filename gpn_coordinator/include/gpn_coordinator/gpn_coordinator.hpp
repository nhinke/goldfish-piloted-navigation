#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>

#include <gpn_msgs/msg/fish_cmd.hpp>
#include <gpn_msgs/srv/compute_controls.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define FIXED_FLOAT(x) std::fixed << std::setprecision(2) << (x)

#ifndef GPN_COORDINATOR_COMPONENT
#define GPN_COORDINATOR_COMPONENT

namespace gpn {

    class coordinator : public rclcpp::Node {

        private:

            double max_lin_vel_, max_ang_vel_;
            std::string max_lin_vel_param_, max_ang_vel_param_;

            std::string odometry_topic_name_, fish_cmd_topic_name_;
            std::string odometry_topic_param_, fish_cmd_topic_param_;

            std::string controller_server_name_;
            std::string controller_server_name_param_;

            double curr_heading_;
            geometry_msgs::msg::Pose curr_pose_;
            geometry_msgs::msg::Twist curr_twist_;

            bool waiting_for_init_pose_;
            geometry_msgs::msg::Pose init_pose_;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
            rclcpp::Subscription<gpn_msgs::msg::FishCmd>::SharedPtr sub_fish_cmd_;

            rclcpp::Client<gpn_msgs::srv::ComputeControls>::SharedPtr client_controller_;

        protected:

            void odometry_callback(const nav_msgs::msg::Odometry& odom_msg);
            void fish_command_callback(const gpn_msgs::msg::FishCmd& cmd_msg);

        public:

            coordinator(const std::string& name, const rclcpp::NodeOptions& options);
            ~coordinator();
            void initialize_params();
            void configure();

    };

}

#endif
