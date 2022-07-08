#include <gpn_pid_controller/gpn_pid_controller.hpp>

gpn::pid_controller::pid_controller(const std::string& name, const rclcpp::NodeOptions& options) : 
    Node(name, options) {

    this->initialize_params();
    this->configure();

    std::cout << "gpn_pid_controller_node constructed successfully" << std::endl;

}

gpn::pid_controller::~pid_controller() {}

void gpn::pid_controller::initialize_params() {

    gain_P_param_ = "pid_controller_gain_P";
    gain_I_param_ = "pid_controller_gain_I";
    gain_D_param_ = "pid_controller_gain_D";

    rcl_interfaces::msg::ParameterDescriptor gain_P_descriptor;
    gain_P_descriptor.name = gain_P_param_;
    gain_P_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    gain_P_descriptor.description = "Value of proportional constant used in PID controller";
    gain_P_descriptor.additional_constraints = "Should be of form '1.0', for example";
    this->declare_parameter(gain_P_param_, 1.0, gain_P_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_I_descriptor;
    gain_I_descriptor.name = gain_I_param_;
    gain_I_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    gain_I_descriptor.description = "Value of integral constant used in PID controller";
    gain_I_descriptor.additional_constraints = "Should be of form '0.0', for example";
    this->declare_parameter(gain_I_param_, 0.0, gain_I_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_D_descriptor;
    gain_D_descriptor.name = gain_D_param_;
    gain_D_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    gain_D_descriptor.description = "Value of derivative constant used in PID controller";
    gain_D_descriptor.additional_constraints = "Should be of form '0.1', for example";
    this->declare_parameter(gain_D_param_, 0.1, gain_D_descriptor);

}

void gpn::pid_controller::configure() {

    this->get_parameter<double>(gain_P_param_, gain_P_);
    std::cout << "PID controller proportional gain: " << gain_P_ << std::endl;

    this->get_parameter<double>(gain_I_param_, gain_I_);
    std::cout << "PID controller integral gain:     " << gain_I_ << std::endl;

    this->get_parameter<double>(gain_D_param_, gain_D_);
    std::cout << "PID controller derivative gain:   " << gain_D_ << std::endl;

}

