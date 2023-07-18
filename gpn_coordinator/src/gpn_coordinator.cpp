#include <gpn_coordinator/gpn_coordinator.hpp>

gpn::coordinator::coordinator(const std::string& name, const rclcpp::NodeOptions& options) : 
    Node(name, options), has_new_goal_(false), waiting_for_init_pose_(true) {

    this->initialize_params();
    this->configure();
    dt_ = freq_hz_ > 0.0 ? 1 / freq_hz_ : 0.1;
    dt_ms_ = (int)std::round(controller_timeout_sec_*1000.0);
    
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 10, std::bind(&gpn::coordinator::odometry_callback, this, std::placeholders::_1));
    sub_fish_cmd_ = this->create_subscription<gpn_msgs::msg::FishCmd>(fish_cmd_topic_name_, 10, std::bind(&gpn::coordinator::fish_command_callback, this, std::placeholders::_1));

    rclcpp::Time now = this->now();
    client_controller_ = this->create_client<gpn_msgs::srv::ComputeControls>(controller_server_name_);
    if (!client_controller_->wait_for_service(std::chrono::milliseconds(dt_ms_))) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: could not connect to controller server after waiting for %.2f seconds.", controller_timeout_sec_);
        // TODO: decide what should happen if connection unsuccessful --> e.g. retry connect/kill node/carry on
    } else if (debug_) {
        RCLCPP_DEBUG(this->get_logger(), "Successfully connected to controller server at '%s' after %.2f seconds.", controller_server_name_.c_str(), (this->now() - now).seconds());
    }

    timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&gpn::coordinator::iterate, this));
    RCLCPP_DEBUG(this->get_logger(), "gpn_coordinator_node constructed successfully.");

}

gpn::coordinator::~coordinator() {}

void gpn::coordinator::initialize_params() {

    rcl_interfaces::msg::ParameterDescriptor debug_descriptor;
    debug_descriptor.name = debug_param_;
    debug_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    debug_descriptor.description = "Control whether or not debugging stream printed to active terminal";
    debug_descriptor.additional_constraints = "Should be of form false, for example";
    this->declare_parameter(debug_param_, false, debug_descriptor);

    rcl_interfaces::msg::ParameterDescriptor frequency_descriptor;
    frequency_descriptor.name = frequency_param_;
    frequency_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    frequency_descriptor.description = "Operating frequency of GPN coordinator node (Hz)";
    frequency_descriptor.additional_constraints = "Should be of form 10.0, for example";
    this->declare_parameter(frequency_param_, 10.0, frequency_descriptor);

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

    rcl_interfaces::msg::ParameterDescriptor controller_timeout_descriptor;
    controller_timeout_descriptor.name = controller_timeout_param_;
    controller_timeout_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    controller_timeout_descriptor.description = "Time to wait for controller server to become available (sec)";
    controller_timeout_descriptor.additional_constraints = "Should be of form 5.0, for example";
    this->declare_parameter(controller_timeout_param_, 5.0, controller_timeout_descriptor);

    rcl_interfaces::msg::ParameterDescriptor controller_server_name_descriptor;
    controller_server_name_descriptor.name = controller_server_name_param_;
    controller_server_name_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    controller_server_name_descriptor.description = "Name of server used to compute PID controls";
    controller_server_name_descriptor.additional_constraints = "Should be of form 'compute_control_inputs', for example";
    this->declare_parameter(controller_server_name_param_, "compute_control_inputs", controller_server_name_descriptor);

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

}

void gpn::coordinator::configure() {

    // TODO use proper logger

    this->get_parameter<bool>(debug_param_, debug_);
    std::cout << "Debugging stream status:   " << std::boolalpha << debug_ << std::endl;

    this->get_parameter<double>(frequency_param_, freq_hz_);
    std::cout << "Operating frequency (hz):  " << FIXED_FLOAT(freq_hz_) << std::endl;

    this->get_parameter<double>(max_lin_vel_param_, max_lin_vel_);
    std::cout << "Max lin. velocity (m/s):   " << FIXED_FLOAT(max_lin_vel_) << std::endl;

    this->get_parameter<double>(max_ang_vel_param_, max_ang_vel_);
    std::cout << "Max ang. velocity (rad/s): " << FIXED_FLOAT(max_ang_vel_) << std::endl;

    this->get_parameter<double>(controller_timeout_param_, controller_timeout_sec_);
    std::cout << "Controller timeout (sec):  " << controller_timeout_sec_ << std::endl;

    this->get_parameter<std::string>(controller_server_name_param_, controller_server_name_);
    std::cout << "Controller server name:    " << controller_server_name_ << std::endl;

    this->get_parameter<std::string>(odometry_topic_param_, odometry_topic_name_);
    std::cout << "create3 odometry topic:    " << odometry_topic_name_ << std::endl;

    this->get_parameter<std::string>(fish_cmd_topic_param_, fish_cmd_topic_name_);
    std::cout << "Fish command topic:        " << fish_cmd_topic_name_ << std::endl;
    
}

void gpn::coordinator::odometry_callback(const nav_msgs::msg::Odometry& odom_msg) {

    const std::lock_guard<std::mutex> curr_lock(curr_mutex_);

    curr_state_.header = odom_msg.header;
    curr_state_.pose = odom_msg.pose.pose;
    curr_state_.twist = odom_msg.twist.twist;
    curr_state_.heading = tf2::getYaw(odom_msg.pose.pose.orientation);

    if (debug_) {
        RCLCPP_DEBUG(this->get_logger(), "Current vehicle heading:  %.2f", curr_state_.heading);
    }

    if (waiting_for_init_pose_) {
        init_pose_ = curr_state_.pose;
        waiting_for_init_pose_ = false;
    }

}

void gpn::coordinator::fish_command_callback(const gpn_msgs::msg::FishCmd& cmd_msg) {

    const std::lock_guard<std::mutex> goal_lock(goal_mutex_);

    has_new_goal_ = true;

    if (debug_) {
        RCLCPP_DEBUG(this->get_logger(), "Received fish command:  heading = %.2f, magnitude = %.2f", cmd_msg.heading, cmd_msg.magnitude);
    }

    goal_odom_.header = cmd_msg.header;
    goal_odom_.heading = cmd_msg.heading;
    goal_odom_.fwd_vel = cmd_msg.magnitude * max_lin_vel_;

}

void gpn::coordinator::compute_controls(const gpn_msgs::msg::GpnState& curr, const gpn_msgs::msg::GpnOdom& goal) {

    const std::lock_guard<std::mutex> curr_lock(curr_mutex_);
    const std::lock_guard<std::mutex> goal_lock(goal_mutex_);

    // TODO: can create3 receive fwd and ang commands at same time?
    std::shared_ptr<gpn_msgs::srv::ComputeControls::Request> ctrls_srv_req;
    RCLCPP_INFO(this->get_logger(), "44Sending compute controls request to service server.");
    ctrls_srv_req->goal_odom = goal;
    RCLCPP_INFO(this->get_logger(), "55Sending compute controls request to service server.");

    ctrls_srv_req->curr_odom.header = curr.header;
    ctrls_srv_req->curr_odom.heading = curr.heading;
    ctrls_srv_req->curr_odom.fwd_vel = curr.twist.linear.x;
    
    RCLCPP_INFO(this->get_logger(), "66Sending compute controls request to service server.");
    RCLCPP_INFO(this->get_logger(), "Sending compute controls request to service server.");
    
    rclcpp::Client<gpn_msgs::srv::ComputeControls>::SharedFuture ctrls_srv_future = 
        client_controller_->async_send_request(ctrls_srv_req, std::bind(&gpn::coordinator::controls_response_callback, this, std::placeholders::_1));

}

void gpn::coordinator::controls_response_callback(rclcpp::Client<gpn_msgs::srv::ComputeControls>::SharedFuture future) {
    std::future_status status = future.wait_for(std::chrono::milliseconds(dt_ms_));
    RCLCPP_INFO(this->get_logger(), "ready: %d, deferred: %d, timeout: %d", status == std::future_status::ready, status == std::future_status::deferred, status == std::future_status::timeout);
    while (status != std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "ready: %d, deferred: %d, timeout: %d", status == std::future_status::ready, status == std::future_status::deferred, status == std::future_status::timeout);
    }
}

void gpn::coordinator::iterate() {

    // if no goal state 
    if (!has_new_goal_) {
        // be sure to consider what happens if you haven't received a fish command for a while --> whether to keep sending same command or start sending zeros)
        // publish current state as command?
        return;
    }

    // if waiting for initial pose
    if (waiting_for_init_pose_) {
        RCLCPP_DEBUG(this->get_logger(), "Waiting for initial odometry measurements.");
        return;
    }

    compute_controls(curr_state_, goal_odom_);

}
