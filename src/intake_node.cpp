#include "hardware_team_robot/intake_node.h"
#include <chrono>

using namespace std::chrono_literals;

IntakeNode::IntakeNode() : Node("intake_node"), last_intake_cmd_time_(this->now()), current_state_(0) {
    RCLCPP_INFO(this->get_logger(), "Initializing Intake Logic Node...");

    // QoS for reliability
    rclcpp::QoS intake_qos(10);
    intake_qos.transient_local();
    intake_qos.reliable();

    // Subscriber: Listen for raw commands from UI/Teleop
    intake_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/intake_cmd", 10,
        std::bind(&IntakeNode::intake_callback, this, std::placeholders::_1)
    );

    // Publisher: Send validated commands to the Hardware/Chassis node
    intake_pub_ = this->create_publisher<std_msgs::msg::Int8>("/intake_cmd_validated", intake_qos);

    // Watchdog Timer: Checks every 500ms
    watchdog_timer_ = this->create_wall_timer(
        500ms,
        std::bind(&IntakeNode::watchdog_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Intake Logic Node ready. Filtering /intake_cmd -> /intake_cmd_validated");
}

IntakeNode::~IntakeNode() {
    // Verify ROS context and publisher are still valid before attempting to publish
    if (rclcpp::ok() && intake_pub_) {
        RCLCPP_WARN(this->get_logger(), "Shutting down Intake Node. Sending safety stop.");
        publish_validated_state(0);
    } else {
        // Fallback warning using standard error since ROS logging might be offline
        std::cerr << "[IntakeNode WARNING] ROS context invalid or publisher uninitialized. Safety stop could not be guaranteed!" << std::endl;
    }
}

void IntakeNode::intake_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    // Reset the watchdog timer timestamp
    last_intake_cmd_time_ = this->now();
    
    // Logic: Only publish if the state has actually changed to save bandwidth
    if (msg->data != current_state_) {
        publish_validated_state(msg->data);
    }
}

void IntakeNode::watchdog_callback() {
    // Safety: If no command received for > 1.0 second, force state to 0 (Off)
    auto since_last_cmd = (this->now() - last_intake_cmd_time_).seconds();
    
    if (since_last_cmd > 1.0 && current_state_ != 0) {
        RCLCPP_WARN(this->get_logger(), "Watchdog timeout! Stopping intake.");
        publish_validated_state(0);
    }
}

void IntakeNode::publish_validated_state(int state) {
    current_state_ = state;
    auto message = std_msgs::msg::Int8();
    message.data = static_cast<int8_t>(state);
    
   // Extra safety guard just in case
    if (intake_pub_) {
        intake_pub_->publish(message);
    }
    
    // Log the transition
    if (state == 1) RCLCPP_INFO(this->get_logger(), "State: FORWARD");
    else if (state == -1) RCLCPP_INFO(this->get_logger(), "State: REVERSE");
    else RCLCPP_INFO(this->get_logger(), "State: STOPPED");
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntakeNode>());
    rclcpp::shutdown();
    return 0;
}