#include <gpn_coordinator/gpn_coordinator.hpp>

using std::placeholders::_1;

gpn::coordinator::coordinator(const std::string& name, const rclcpp::NodeOptions& options) : 
    Node(name, options), waiting_for_init_pose(true) {

    this->intialize_params();
    this->configure();
    
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name, 10, std::bind(&gpn::coordinator::odometry_callback, this, _1));
    sub_fish_cmd = this->create_subscription<gpn_msgs::msg::FishCmd>(fish_cmd_topic_name, 10, std::bind(&gpn::coordinator::fish_command_callback, this, _1));

    std::cout << "gpn_coordinator_node constructed successfully" << std::endl;

}

gpn::coordinator::~coordinator() {}

void gpn::coordinator::intialize_params() {

    odometry_topic_param = "odometry_topic";
    fish_cmd_topic_param = "fish_cmd_topic";

    rcl_interfaces::msg::ParameterDescriptor odometry_topic_descriptor;
    odometry_topic_descriptor.name = odometry_topic_param;
    odometry_topic_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    odometry_topic_descriptor.description = "Name of topic to which create3 odometry information is published";
    odometry_topic_descriptor.additional_constraints = "Should be of form 'odom'";
    this->declare_parameter(odometry_topic_param, "odom", odometry_topic_descriptor);

    rcl_interfaces::msg::ParameterDescriptor fish_cmd_topic_descriptor;
    fish_cmd_topic_descriptor.name = fish_cmd_topic_param;
    fish_cmd_topic_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    fish_cmd_topic_descriptor.description = "Name of topic to which fish command information is published";
    fish_cmd_topic_descriptor.additional_constraints = "Should be of form 'fish_cmd'";
    this->declare_parameter(fish_cmd_topic_param, "fish_cmd", fish_cmd_topic_descriptor);

}

void gpn::coordinator::configure() {

    this->get_parameter<std::string>(odometry_topic_param, odometry_topic_name);
    std::cout << "odometry topic: " << odometry_topic_name << std::endl;

    this->get_parameter<std::string>(fish_cmd_topic_param, fish_cmd_topic_name);
    std::cout << "fish command topic: " << fish_cmd_topic_name << std::endl;

}

void gpn::coordinator::odometry_callback(const nav_msgs::msg::Odometry& odom_msg) {

    curr_pose = odom_msg.pose.pose;
    curr_twist = odom_msg.twist.twist;
    std::cout << "curr lin vel:  [ " << curr_twist.linear.x << " " << curr_twist.linear.y << " " << curr_twist.linear.z << " ]" << std::endl;
    
    if (waiting_for_init_pose) {
        init_pose = curr_pose;
        waiting_for_init_pose = false;
    }

}

void gpn::coordinator::fish_command_callback(const gpn_msgs::msg::FishCmd& cmd_msg) {
    
    double cmd_heading = cmd_msg.heading;
    double cmd_magnitude = cmd_msg.magnitude;    
    std::cout << "fish command:  " << cmd_heading << "  " << cmd_magnitude << std::endl;

}
