#include <gpn_coordinator/gpn_coordinator.hpp>

gpn::coordinator::coordinator(const std::string& name, const rclcpp::NodeOptions& options) : 
    Node(name, options), waiting_for_init_pose_(true) {

    this->initialize_params();
    this->configure();
    
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 10, std::bind(&gpn::coordinator::odometry_callback, this, std::placeholders::_1));
    sub_fish_cmd_ = this->create_subscription<gpn_msgs::msg::FishCmd>(fish_cmd_topic_name_, 10, std::bind(&gpn::coordinator::fish_command_callback, this, std::placeholders::_1));

    std::cout << "gpn_coordinator_node constructed successfully" << std::endl;

}

gpn::coordinator::~coordinator() {}

void gpn::coordinator::initialize_params() {

    max_lin_vel_param_ = "max_lin_vel";
    max_ang_vel_param_ = "max_ang_vel";
    odometry_topic_param_ = "odometry_topic";
    fish_cmd_topic_param_ = "fish_cmd_topic";
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
    max_ang_vel_descriptor.description = "Maximum commanded angular velocity of create3 (rad/s)";
    max_ang_vel_descriptor.additional_constraints = "Should be of form 0.5, for example";
    this->declare_parameter(max_ang_vel_param_, 0.5, max_ang_vel_descriptor);

    rcl_interfaces::msg::ParameterDescriptor odometry_topic_descriptor;
    odometry_topic_descriptor.name = odometry_topic_param_;
    odometry_topic_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    odometry_topic_descriptor.description = "Name of topic to which create3 odometry information is published";
    odometry_topic_descriptor.additional_constraints = "Should be of form 'odom', for example";
    this->declare_parameter(odometry_topic_param_, "odom", odometry_topic_descriptor);

    rcl_interfaces::msg::ParameterDescriptor fish_cmd_topic_descriptor;
    fish_cmd_topic_descriptor.name = fish_cmd_topic_param_;
    fish_cmd_topic_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    fish_cmd_topic_descriptor.description = "Name of topic to which fish command information is published";
    fish_cmd_topic_descriptor.additional_constraints = "Should be of form 'fish_cmd', for example";
    this->declare_parameter(fish_cmd_topic_param_, "fish_cmd", fish_cmd_topic_descriptor);

    rcl_interfaces::msg::ParameterDescriptor server_pid_name_descriptor;
    server_pid_name_descriptor.name = server_pid_name_param_;
    server_pid_name_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    server_pid_name_descriptor.description = "Name of server used to compute PID controls";
    server_pid_name_descriptor.additional_constraints = "Should be of form 'compute_pid_controls', for example";
    this->declare_parameter(server_pid_name_param_, "compute_pid_controls", server_pid_name_descriptor);

}

void gpn::coordinator::configure() {

    this->get_parameter<double>(max_lin_vel_param_, max_lin_vel_);
    std::cout << "Max lin. velocity (m/s):    " << FIXED_FLOAT(max_lin_vel_) << std::endl;

    this->get_parameter<double>(max_ang_vel_param_, max_ang_vel_);
    std::cout << "Max ang. velocity (rad/s):  " << FIXED_FLOAT(max_ang_vel_) << std::endl;

    this->get_parameter<std::string>(odometry_topic_param_, odometry_topic_name_);
    std::cout << "create3 odometry topic:     " << odometry_topic_name_ << std::endl;

    this->get_parameter<std::string>(fish_cmd_topic_param_, fish_cmd_topic_name_);
    std::cout << "Fish command topic:         " << fish_cmd_topic_name_ << std::endl;

    this->get_parameter<std::string>(server_pid_name_param_, server_pid_name_);
    std::cout << "PID controller server name: " << server_pid_name_ << std::endl;

}

void gpn::coordinator::odometry_callback(const nav_msgs::msg::Odometry& odom_msg) {

    curr_pose_ = odom_msg.pose.pose;
    curr_twist_ = odom_msg.twist.twist;
    curr_heading_ = tf2::getYaw(curr_pose_.orientation);
    std::cout << "Current vehicle heading: " << curr_heading_ << std::endl;
    
    if (waiting_for_init_pose_) {
        init_pose_ = curr_pose_;
        waiting_for_init_pose_ = false;
    }

}

void gpn::coordinator::fish_command_callback(const gpn_msgs::msg::FishCmd& cmd_msg) {
    
    double cmd_heading = cmd_msg.heading;
    double cmd_magnitude = cmd_msg.magnitude;    
    std::cout << "fish command:  " << cmd_heading << "  " << cmd_magnitude << std::endl;

}
