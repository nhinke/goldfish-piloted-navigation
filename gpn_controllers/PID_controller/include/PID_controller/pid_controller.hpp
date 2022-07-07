#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#ifndef PID_CONTROLLER_COMPONENT
#define PID_CONTROLLER_COMPONENT

namespace gpn {

    class pid_controller : public rclcpp::Node {
    // class pid_controller {

        private:

            double gain_P_;
            double gain_I_;
            double gain_D_;

        public:

            pid_controller(const std::string& name, const rclcpp::NodeOptions& options);
            // pid_controller(const double& Pgain, const double& Igain, const double& Dgain);
            ~pid_controller();
            void intialize_params();
            void configure();

    };

}

#endif
