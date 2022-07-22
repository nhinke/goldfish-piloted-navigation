#include <gpn_pid_controller/gpn_pid_controller.hpp>

gpn::pid_controller::pid_controller(const std::string& name, const rclcpp::NodeOptions& options) : 
    Node(name, options) {

    this->initialize_params();
    this->configure();

    server_ctrls_ = this->create_service<gpn_msgs::srv::ComputeControls>(server_pid_name_, std::bind(&gpn::pid_controller::compute_controls_callback, this, std::placeholders::_1, std::placeholders::_2));

    std::cout << "gpn_pid_controller_node constructed successfully" << std::endl;

}

gpn::pid_controller::~pid_controller() {}

void gpn::pid_controller::initialize_params() {

    max_lin_vel_param_ = "max_lin_vel";
    max_ang_vel_param_ = "max_ang_vel";
    gain_P_param_ = "pid_controller_gain_P";
    gain_I_param_ = "pid_controller_gain_I";
    gain_D_param_ = "pid_controller_gain_D";
    server_pid_name_param_ = "pid_controller_server";

    rcl_interfaces::msg::ParameterDescriptor max_lin_vel_descriptor;
    max_lin_vel_descriptor.name = max_lin_vel_param_;
    max_lin_vel_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_lin_vel_descriptor.description = "Maximum commanded forward velocity of create3 (m/s)";
    max_lin_vel_descriptor.additional_constraints = "Should be of form 1.0, for example";
    this->declare_parameter(max_lin_vel_param_, 1.0, max_lin_vel_descriptor);

    rcl_interfaces::msg::ParameterDescriptor max_ang_vel_descriptor;
    max_ang_vel_descriptor.name = max_ang_vel_param_;
    max_ang_vel_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_ang_vel_descriptor.description = "Maximum commanded angular velocity of create3 (m/s)";
    max_ang_vel_descriptor.additional_constraints = "Should be of form 1.0, for example";
    this->declare_parameter(max_ang_vel_param_, 1.0, max_ang_vel_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_P_descriptor;
    gain_P_descriptor.name = gain_P_param_;
    gain_P_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_P_descriptor.description = "Value of proportional constant used in PID controller";
    gain_P_descriptor.additional_constraints = "Should be of form 1.0, for example";
    this->declare_parameter(gain_P_param_, 1.0, gain_P_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_I_descriptor;
    gain_I_descriptor.name = gain_I_param_;
    gain_I_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_I_descriptor.description = "Value of integral constant used in PID controller";
    gain_I_descriptor.additional_constraints = "Should be of form 0.0, for example";
    this->declare_parameter(gain_I_param_, 0.0, gain_I_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_D_descriptor;
    gain_D_descriptor.name = gain_D_param_;
    gain_D_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_D_descriptor.description = "Value of derivative constant used in PID controller";
    gain_D_descriptor.additional_constraints = "Should be of form 0.1, for example";
    this->declare_parameter(gain_D_param_, 0.1, gain_D_descriptor);

    rcl_interfaces::msg::ParameterDescriptor server_pid_name_descriptor;
    server_pid_name_descriptor.name = server_pid_name_param_;
    server_pid_name_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    server_pid_name_descriptor.description = "Name of server used to compute PID controls";
    server_pid_name_descriptor.additional_constraints = "Should be of form 'compute_pid_controls', for example";
    this->declare_parameter(server_pid_name_param_, "compute_pid_controls", server_pid_name_descriptor);

}

void gpn::pid_controller::configure() {

    this->get_parameter<double>(max_lin_vel_param_, max_lin_vel_);
    std::cout << "max linear velocity (m/s):        " << max_lin_vel_ << std::endl;

    this->get_parameter<double>(max_ang_vel_param_, max_ang_vel_);
    std::cout << "max angular velocity (m/s):       " << max_ang_vel_ << std::endl;

    this->get_parameter<double>(gain_P_param_, gain_P_);
    std::cout << "PID controller proportional gain: " << gain_P_ << std::endl;

    this->get_parameter<double>(gain_I_param_, gain_I_);
    std::cout << "PID controller integral gain:     " << gain_I_ << std::endl;

    this->get_parameter<double>(gain_D_param_, gain_D_);
    std::cout << "PID controller derivative gain:   " << gain_D_ << std::endl;

    this->get_parameter<std::string>(server_pid_name_param_, server_pid_name_);
    std::cout << "PID controller server name:       " << server_pid_name_ << std::endl;

}

void gpn::pid_controller::compute_controls_callback(const std::shared_ptr<gpn_msgs::srv::ComputeControls::Request> request,
    std::shared_ptr<gpn_msgs::srv::ComputeControls::Response> response) {

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = -1;



    response->cmd_vel = cmd_vel;

}
