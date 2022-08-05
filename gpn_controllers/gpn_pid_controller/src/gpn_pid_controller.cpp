#include <gpn_pid_controller/gpn_pid_controller.hpp>

gpn::pid_controller::pid_controller(const std::string& name, const rclcpp::NodeOptions& options) : 
    Node(name, options), prev_time_(this->now()) {

    this->initialize_params();
    this->configure();

    server_controller_ = this->create_service<gpn_msgs::srv::ComputeControls>(controller_server_name_, std::bind(&gpn::pid_controller::compute_controls_callback, this, std::placeholders::_1, std::placeholders::_2));

    std::cout << "gpn_pid_controller_node constructed successfully" << std::endl;

}

gpn::pid_controller::~pid_controller() {}

void gpn::pid_controller::initialize_params() {

    debug_param_ = "debug_stream";
    max_lin_vel_param_ = "max_lin_vel";
    max_ang_vel_param_ = "max_ang_vel";
    gain_lin_P_param_ = "pid_gain_lin_P";
    gain_lin_I_param_ = "pid_gain_lin_I";
    gain_lin_D_param_ = "pid_gain_lin_D";
    gain_ang_P_param_ = "pid_gain_ang_P";
    gain_ang_I_param_ = "pid_gain_ang_I";
    gain_ang_D_param_ = "pid_gain_ang_D";
    controller_server_name_param_ = "controller_server";
    max_time_thresh_D_param_ = "pid_D_term_max_time_threshold";

    rcl_interfaces::msg::ParameterDescriptor debug_descriptor;
    debug_descriptor.name = debug_param_;
    debug_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    debug_descriptor.description = "Control whether or not debugging stream printed to active terminal";
    debug_descriptor.additional_constraints = "Should be of form false, for example";
    this->declare_parameter(debug_param_, false, debug_descriptor);

    rcl_interfaces::msg::ParameterDescriptor max_lin_vel_descriptor;
    max_lin_vel_descriptor.name = max_lin_vel_param_;
    max_lin_vel_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_lin_vel_descriptor.description = "Maximum commanded forward velocity of create3 (m/s)";
    max_lin_vel_descriptor.additional_constraints = "Should be of form 1.0, for example";
    this->declare_parameter(max_lin_vel_param_, 1.0, max_lin_vel_descriptor);

    rcl_interfaces::msg::ParameterDescriptor max_ang_vel_descriptor;
    max_ang_vel_descriptor.name = max_ang_vel_param_;
    max_ang_vel_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_ang_vel_descriptor.description = "Maximum commanded angular velocity of create3 (rad/s)";
    max_ang_vel_descriptor.additional_constraints = "Should be of form 0.5, for example";
    this->declare_parameter(max_ang_vel_param_, 0.5, max_ang_vel_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_lin_P_descriptor;
    gain_lin_P_descriptor.name = gain_lin_P_param_;
    gain_lin_P_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_lin_P_descriptor.description = "Value of proportional constant used in PID controller for linear velocity";
    gain_lin_P_descriptor.additional_constraints = "Should be of form 1.0, for example";
    this->declare_parameter(gain_lin_P_param_, 1.0, gain_lin_P_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_lin_I_descriptor;
    gain_lin_I_descriptor.name = gain_lin_I_param_;
    gain_lin_I_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_lin_I_descriptor.description = "Value of integral constant used in PID controller for linear velocity";
    gain_lin_I_descriptor.additional_constraints = "Should be of form 0.0, for example";
    this->declare_parameter(gain_lin_I_param_, 0.0, gain_lin_I_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_lin_D_descriptor;
    gain_lin_D_descriptor.name = gain_lin_D_param_;
    gain_lin_D_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_lin_D_descriptor.description = "Value of derivative constant used in PID controller for linear velocity";
    gain_lin_D_descriptor.additional_constraints = "Should be of form 0.1, for example";
    this->declare_parameter(gain_lin_D_param_, 0.1, gain_lin_D_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_ang_P_descriptor;
    gain_ang_P_descriptor.name = gain_ang_P_param_;
    gain_ang_P_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_ang_P_descriptor.description = "Value of proportional constant used in PID controller for angular velocity";
    gain_ang_P_descriptor.additional_constraints = "Should be of form 1.0, for example";
    this->declare_parameter(gain_ang_P_param_, 1.0, gain_ang_P_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_ang_I_descriptor;
    gain_ang_I_descriptor.name = gain_ang_I_param_;
    gain_ang_I_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_ang_I_descriptor.description = "Value of integral constant used in PID controller for angular velocity";
    gain_ang_I_descriptor.additional_constraints = "Should be of form 0.0, for example";
    this->declare_parameter(gain_ang_I_param_, 0.0, gain_ang_I_descriptor);

    rcl_interfaces::msg::ParameterDescriptor gain_ang_D_descriptor;
    gain_ang_D_descriptor.name = gain_ang_D_param_;
    gain_ang_D_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gain_ang_D_descriptor.description = "Value of derivative constant used in PID controller for angular velocity";
    gain_ang_D_descriptor.additional_constraints = "Should be of form 0.1, for example";
    this->declare_parameter(gain_ang_D_param_, 0.1, gain_ang_D_descriptor);

    rcl_interfaces::msg::ParameterDescriptor controller_server_name_descriptor;
    controller_server_name_descriptor.name = controller_server_name_param_;
    controller_server_name_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    controller_server_name_descriptor.description = "Name of server used to compute PID controls";
    controller_server_name_descriptor.additional_constraints = "Should be of form 'compute_control_inputs', for example";
    this->declare_parameter(controller_server_name_param_, "compute_control_inputs", controller_server_name_descriptor);

    rcl_interfaces::msg::ParameterDescriptor max_time_thresh_D_vel_descriptor;
    max_time_thresh_D_vel_descriptor.name = max_time_thresh_D_param_;
    max_time_thresh_D_vel_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_time_thresh_D_vel_descriptor.description = "Time threshold between setpoints under which the derivative term will be applied within the computed PID controls (sec)";
    max_time_thresh_D_vel_descriptor.additional_constraints = "Should be of form 2.0, for example";
    this->declare_parameter(max_time_thresh_D_param_, 2.0, max_time_thresh_D_vel_descriptor);

}

void gpn::pid_controller::configure() {

    this->get_parameter<bool>(debug_param_, debug_);
    std::cout << "Debugging stream status:                " << debug_ << std::endl;

    this->get_parameter<double>(max_lin_vel_param_, max_lin_vel_);
    std::cout << "Max linear velocity (m/s):              " << FIXED_FLOAT(max_lin_vel_) << std::endl;

    this->get_parameter<double>(max_ang_vel_param_, max_ang_vel_);
    std::cout << "Max angular velocity (rad/s):           " << FIXED_FLOAT(max_ang_vel_) << std::endl;

    this->get_parameter<double>(gain_lin_P_param_, gain_lin_P_);
    std::cout << "PID controller lin. proportional gain:  " << FIXED_FLOAT(gain_lin_P_) << std::endl;

    this->get_parameter<double>(gain_lin_I_param_, gain_lin_I_);
    std::cout << "PID controller lin. integral gain:      " << FIXED_FLOAT(gain_lin_I_) << std::endl;

    this->get_parameter<double>(gain_lin_D_param_, gain_lin_D_);
    std::cout << "PID controller lin. derivative gain:    " << FIXED_FLOAT(gain_lin_D_) << std::endl;

    this->get_parameter<double>(gain_ang_P_param_, gain_ang_P_);
    std::cout << "PID controller ang. proportional gain:  " << FIXED_FLOAT(gain_ang_P_) << std::endl;

    this->get_parameter<double>(gain_ang_I_param_, gain_ang_I_);
    std::cout << "PID controller ang. integral gain:      " << FIXED_FLOAT(gain_ang_I_) << std::endl;

    this->get_parameter<double>(gain_ang_D_param_, gain_ang_D_);
    std::cout << "PID controller ang. derivative gain:    " << FIXED_FLOAT(gain_ang_D_) << std::endl;

    this->get_parameter<double>(max_time_thresh_D_param_, max_time_thresh_D_);
    std::cout << "Max time threshold for derivative term: " << FIXED_FLOAT(max_time_thresh_D_) << std::endl;

    this->get_parameter<std::string>(controller_server_name_param_, controller_server_name_);
    std::cout << "Controller server name:                 " << controller_server_name_ << std::endl;

}

void gpn::pid_controller::compute_controls_callback(const std::shared_ptr<gpn_msgs::srv::ComputeControls::Request> request,
    std::shared_ptr<gpn_msgs::srv::ComputeControls::Response> response) {

    double cmd_lin = 0.0;
    double cmd_ang = 0.0;
    geometry_msgs::msg::Twist cmd_vel;

    // get current time and errors
    curr_time_ = request->curr_odom.header.stamp; 
    curr_err_lin_ = request->goal_odom.fwd_vel - request->curr_odom.fwd_vel;
    curr_err_ang_ = request->goal_odom.heading - request->curr_odom.heading;
    
    // add proportional terms
    cmd_lin += gain_lin_P_*curr_err_lin_;
    cmd_ang += gain_ang_P_*curr_err_ang_;

    // add derivative terms
    double dt = (curr_time_-prev_time_).seconds();
    std::cout << "dt: " << dt << std::endl; // TODO DELETE
    if (dt < max_time_thresh_D_) {
        cmd_lin += gain_lin_D_*(curr_err_lin_ - prev_err_lin_)/dt;
        cmd_ang += gain_ang_D_*(curr_err_ang_ - prev_err_ang_)/dt;
    }

    // add integral terms (TODO: anti-windup)
    cmd_lin += gain_lin_I_*accu_err_lin_;
    cmd_ang += gain_ang_I_*accu_err_ang_;

    // enforce control constraints
    if (cmd_lin > 0.0) {
        cmd_vel.linear.x = (cmd_lin < max_lin_vel_) ? cmd_lin : max_lin_vel_; 
    } else {
        cmd_vel.linear.x = 0.0;
    }
    if (cmd_ang > 0.0) {
        cmd_vel.angular.z = (cmd_ang < max_ang_vel_) ? cmd_ang : max_ang_vel_;
    } else {
        cmd_vel.angular.z = (cmd_ang > -1.0*max_ang_vel_) ? cmd_ang : -1.0*max_ang_vel_;
    }

    // TODO DELETE
    std::cout << "cmd_lin: " << cmd_lin << std::endl;
    std::cout << "cmd_ang: " << cmd_ang << std::endl;

    // populate response
    response->cmd_vel = cmd_vel;
    
    // set previous and accumulated values
    prev_time_ = curr_time_;
    prev_err_lin_ = curr_err_lin_;
    prev_err_ang_ = curr_err_ang_;
    accu_err_lin_ += curr_err_lin_;
    accu_err_ang_ += curr_err_ang_;

}
