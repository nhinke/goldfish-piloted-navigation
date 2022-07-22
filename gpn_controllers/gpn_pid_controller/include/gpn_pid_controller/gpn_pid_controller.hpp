#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <gpn_msgs/srv/compute_controls.hpp>

#ifndef PID_CONTROLLER_COMPONENT
#define PID_CONTROLLER_COMPONENT

namespace gpn {

    class pid_controller : public rclcpp::Node {

        private:

            double max_lin_vel_, max_ang_vel_;
            std::string max_lin_vel_param_, max_ang_vel_param_;

            std::string gain_P_param_;
            std::string gain_I_param_;
            std::string gain_D_param_;
            std::string server_pid_name_param_;

            double gain_P_;
            double gain_I_;
            double gain_D_;
            std::string server_pid_name_;

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
