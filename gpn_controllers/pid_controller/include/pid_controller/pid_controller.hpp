#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#ifndef PID_CONTROLLER_COMPONENT
#define PID_CONTROLLER_COMPONENT

namespace gpn {

    class pid_controller : public rclcpp::Node {

        private:

            std::string gain_P_param_;
            std::string gain_I_param_;
            std::string gain_D_param_;

            double gain_P_;
            double gain_I_;
            double gain_D_;

        public:

            pid_controller(const std::string& name, const rclcpp::NodeOptions& options);
            ~pid_controller();
            void initialize_params();
            void configure();

    };

}

#endif
