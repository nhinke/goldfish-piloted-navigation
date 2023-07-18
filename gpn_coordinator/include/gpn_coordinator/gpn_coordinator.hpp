#include <mutex>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>

#include <gpn_msgs/msg/fish_cmd.hpp>
#include <gpn_msgs/msg/gpn_state.hpp>
#include <gpn_msgs/srv/compute_controls.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define FIXED_FLOAT(x) std::fixed << std::setprecision(2) << (x)

#ifndef GPN_COORDINATOR_COMPONENT
#define GPN_COORDINATOR_COMPONENT

namespace gpn {

    using namespace std::chrono_literals;

    typedef std::shared_ptr<gpn_msgs::srv::ComputeControls::Request> ctrls_req;
    typedef std::shared_ptr<gpn_msgs::srv::ComputeControls::Response> ctrls_res;

    class coordinator : public rclcpp::Node {

        private:

            double dt_;
            long int dt_ms_;

            bool debug_;
            double freq_hz_;
            double max_lin_vel_;
            double max_ang_vel_;
            double controller_timeout_sec_;
            std::string controller_server_name_;
            std::string odometry_topic_name_;
            std::string fish_cmd_topic_name_;
            
            const std::string debug_param_ = "debug_stream";
            const std::string frequency_param_ = "freq_hz";
            const std::string max_lin_vel_param_ = "max_lin_vel";
            const std::string max_ang_vel_param_ = "max_ang_vel";
            const std::string controller_timeout_param_ = "controller_timeout_sec";
            const std::string controller_server_name_param_ = "controller_server";
            const std::string odometry_topic_param_ = "odometry_topic";
            const std::string fish_cmd_topic_param_ = "fish_cmd_topic";

            std::mutex goal_mutex_;
            std::mutex curr_mutex_;

            bool has_new_goal_;
            bool waiting_for_init_pose_;
            geometry_msgs::msg::Pose init_pose_;

            gpn_msgs::msg::GpnOdom goal_odom_;
            gpn_msgs::msg::GpnState curr_state_;

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
            rclcpp::Subscription<gpn_msgs::msg::FishCmd>::SharedPtr sub_fish_cmd_;
            rclcpp::Client<gpn_msgs::srv::ComputeControls>::SharedPtr client_controller_;

        protected:

            void iterate();
            void odometry_callback(const nav_msgs::msg::Odometry& odom_msg);
            void fish_command_callback(const gpn_msgs::msg::FishCmd& cmd_msg);
            void compute_controls(const gpn_msgs::msg::GpnState& curr, const gpn_msgs::msg::GpnOdom& goal);
            void controls_response_callback(rclcpp::Client<gpn_msgs::srv::ComputeControls>::SharedFuture future);

        public:

            coordinator(const std::string& name, const rclcpp::NodeOptions& options);
            ~coordinator();
            void initialize_params();
            void configure();

    };

}

#endif
