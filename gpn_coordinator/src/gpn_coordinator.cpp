#include <gpn_coordinator/gpn_coordinator.hpp>

gpn::coordinator::coordinator(const std::string& name, const rclcpp::NodeOptions& options) : 
    Node(name, options), waiting_for_init_pose_(true) {

    this->initialize_params();
    this->configure();
    
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 10, std::bind(&gpn::coordinator::odometry_callback, this, std::placeholders::_1));
    sub_fish_cmd_ = this->create_subscription<gpn_msgs::msg::FishCmd>(fish_cmd_topic_name_, 10, std::bind(&gpn::coordinator::fish_command_callback, this, std::placeholders::_1));

    rclcpp::Time now = this->now();
    client_controller_ = this->create_client<gpn_msgs::srv::ComputeControls>(controller_server_name_);
    if (!client_controller_->wait_for_service(std::chrono::seconds(controller_timeout_sec_))) {
        std::cout << "ERROR: could not connect to controller server after waiting for " << controller_timeout_sec_ << " seconds" << std::endl;
        // TODO: decide what should happen if connection unsuccessful --> e.g. retry connect/kill node/carry on
    } else if (debug_) {
        std::cout << "Successfully connected to controller server at '" << controller_server_name_ << "' after " << (this->now() - now).seconds() << " seconds" << std::endl;
    }

    std::cout << "gpn_coordinator_node constructed successfully" << std::endl;

}

gpn::coordinator::~coordinator() {}

void gpn::coordinator::initialize_params() {

    debug_param_ = "debug_stream";
    max_lin_vel_param_ = "max_lin_vel";
    max_ang_vel_param_ = "max_ang_vel";
    odometry_topic_param_ = "odometry_topic";
    fish_cmd_topic_param_ = "fish_cmd_topic";
    controller_timeout_param_ = "controller_timeout";
    controller_server_name_param_ = "controller_server";

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

    rcl_interfaces::msg::ParameterDescriptor controller_timeout_descriptor;
    controller_timeout_descriptor.name = controller_timeout_param_;
    controller_timeout_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    controller_timeout_descriptor.description = "Time to wait for controller server to become available (sec)";
    controller_timeout_descriptor.additional_constraints = "Should be of form 5, for example";
    this->declare_parameter(controller_timeout_param_, 5, controller_timeout_descriptor);

    rcl_interfaces::msg::ParameterDescriptor controller_server_name_descriptor;
    controller_server_name_descriptor.name = controller_server_name_param_;
    controller_server_name_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    controller_server_name_descriptor.description = "Name of server used to compute PID controls";
    controller_server_name_descriptor.additional_constraints = "Should be of form 'compute_control_inputs', for example";
    this->declare_parameter(controller_server_name_param_, "compute_control_inputs", controller_server_name_descriptor);

}

void gpn::coordinator::configure() {

    this->get_parameter<bool>(debug_param_, debug_);
    std::cout << "Debugging stream status:   " << std::boolalpha << debug_ << std::endl;

    this->get_parameter<double>(max_lin_vel_param_, max_lin_vel_);
    std::cout << "Max lin. velocity (m/s):   " << FIXED_FLOAT(max_lin_vel_) << std::endl;

    this->get_parameter<double>(max_ang_vel_param_, max_ang_vel_);
    std::cout << "Max ang. velocity (rad/s): " << FIXED_FLOAT(max_ang_vel_) << std::endl;

    this->get_parameter<std::string>(odometry_topic_param_, odometry_topic_name_);
    std::cout << "create3 odometry topic:    " << odometry_topic_name_ << std::endl;

    this->get_parameter<std::string>(fish_cmd_topic_param_, fish_cmd_topic_name_);
    std::cout << "Fish command topic:        " << fish_cmd_topic_name_ << std::endl;
    
    this->get_parameter<int>(controller_timeout_param_, controller_timeout_sec_);
    std::cout << "Controller timeout (sec):  " << controller_timeout_sec_ << std::endl;

    this->get_parameter<std::string>(controller_server_name_param_, controller_server_name_);
    std::cout << "Controller server name:    " << controller_server_name_ << std::endl;

}

void gpn::coordinator::odometry_callback(const nav_msgs::msg::Odometry& odom_msg) {

    curr_odom_time_ = odom_msg.header.stamp;
    curr_pose_ = odom_msg.pose.pose;
    curr_twist_ = odom_msg.twist.twist;
    curr_heading_ = tf2::getYaw(curr_pose_.orientation);

    if (debug_) {
        std::cout << "Current vehicle heading: " << curr_heading_ << std::endl;
    }

    if (waiting_for_init_pose_) {
        init_pose_ = curr_pose_;
        waiting_for_init_pose_ = false;
    }

}

void gpn::coordinator::fish_command_callback(const gpn_msgs::msg::FishCmd& cmd_msg) {

    if (waiting_for_init_pose_) {
        std::cout << "ERROR: cannot accept any fish commands until odometry information received" << std::endl;
        return;
    }

    double cmd_heading = cmd_msg.heading;
    double cmd_magnitude = cmd_msg.magnitude;    

    if (debug_) {
        std::cout << "Received fish command:  " << cmd_heading << "  " << cmd_magnitude << std::endl;
    }

    std::shared_ptr<gpn_msgs::srv::ComputeControls::Request> ctrls_srv_req;
    ctrls_srv_req->curr_odom.heading = curr_heading_;
    ctrls_srv_req->curr_odom.fwd_vel = curr_twist_.linear.x;
    ctrls_srv_req->curr_odom.header.stamp = curr_odom_time_;
    
    // TODO: can create3 receive fwd and ang commands at same time?
    ctrls_srv_req->goal_odom.heading = cmd_heading;
    ctrls_srv_req->goal_odom.fwd_vel = cmd_magnitude*max_lin_vel_;
    ctrls_srv_req->goal_odom.header.stamp = cmd_msg.header.stamp;

    // auto ctrls_srv_res = client_controller_->async_send_request(ctrls_srv_req);

    // TODO: implement timer-based publisher of velocity commands to robot with configurable frequency,
    //       be sure to consider what happens if you haven't received a fish command for a while --> whether to keep sending same command or start sending zeros)

    // TODO: decide how to handle calling service within callback:
    //           - option 1: use multi-threaded executor with reentrant callback group
    //           - option 2: use multi-threaded executor with two mutually-exclusive callback groups
    //           - option 3: provide callback with asynchronous server request and publish most recent command in timer callback (must not be reentrant callback groups (i.e. singlethreaded exec or multithreaded exec w/ ME callback groups) to prevent data race for latest command, could get weird)
    //       I'm thinking option 2 or 3 --> https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
    //       I'm thinking option 3 with a single-threaded executor to prevent any chance of data races (just keep everything asynchronous)

}
