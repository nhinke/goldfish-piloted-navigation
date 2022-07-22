#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <gpn_msgs/srv/compute_controls.hpp>

#define FIXED_FLOAT(x) std::fixed << std::setprecision(2) << (x)

#ifndef PID_CONTROLLER_COMPONENT
#define PID_CONTROLLER_COMPONENT

namespace gpn {

    class pid_controller : public rclcpp::Node {

        private:

            double max_lin_vel_, max_ang_vel_;
            std::string max_lin_vel_param_, max_ang_vel_param_;

            std::string server_pid_name_;
            std::string server_pid_name_param_;

            double max_time_thresh_D_;
            std::string max_time_thresh_D_param_;

            double gain_lin_P_, gain_lin_I_, gain_lin_D_;
            std::string gain_lin_P_param_, gain_lin_I_param_, gain_lin_D_param_;

            double gain_ang_P_, gain_ang_I_, gain_ang_D_;
            std::string gain_ang_P_param_, gain_ang_I_param_, gain_ang_D_param_;

            rclcpp::Time curr_time_, prev_time_;
            double curr_err_lin_, curr_err_ang_; // current
            double prev_err_lin_, prev_err_ang_; // previous
            double accu_err_lin_, accu_err_ang_; // accumulated

            rclcpp::Service<gpn_msgs::srv::ComputeControls>::SharedPtr server_ctrls_;

        protected:

            void compute_controls_callback(const std::shared_ptr<gpn_msgs::srv::ComputeControls::Request> request, std::shared_ptr<gpn_msgs::srv::ComputeControls::Response> response);

        public:

            pid_controller(const std::string& name, const rclcpp::NodeOptions& options);
            ~pid_controller();
            void initialize_params();
            void configure();

    };

}

#endif
