#include "hardware_team_robot/intake_node.h"
#include <stdexcept>
#include <chrono> // Added for time and duration

IntakeNode::IntakeNode() : Node("intake_node"), last_intake_cmd_time_(this->now()) {
    RCLCPP_INFO(this->get_logger(), "Initializing Intake Node...");

    pca_driver_ = std::make_unique<Hardware::PCA9685Driver>(1, 0x40);
    if (!pca_driver_->initialize()) {
        RCLCPP_FATAL(this->get_logger(), "PCA9685 initialization failed for IntakeNode");
        throw std::runtime_error("PCA9685 initialization failed");
    }

    // Safety: Ensure the motor is strictly off at startup
    stop_intake();

    // Reinstated the transient_local QoS for startup reliability
    rclcpp::QoS intake_qos(10);
    intake_qos.transient_local();
    intake_qos.reliable();

    intake_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/intake_cmd", intake_qos,
        std::bind(&IntakeNode::intake_callback, this, std::placeholders::_1)
    );

    // Watchdog Timer: Checks every 500ms
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&IntakeNode::watchdog_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Intake Node ready and listening on /intake_cmd");
}

IntakeNode::~IntakeNode() {
    RCLCPP_WARN(this->get_logger(), "Shutting down Intake Node. Stopping motor.");
    
    // Safety: Guarantee motor stops if node crashes or is killed
    stop_intake();
    
    // Release PCA9685 resources via RAII
    pca_driver_.reset();
}

void IntakeNode::intake_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    // Reset the watchdog timer on every received command
    last_intake_cmd_time_ = this->now();
    
    // Pass the requested state directly to the hardware logic
    set_intake_hardware(msg->data);
}

void IntakeNode::watchdog_callback() {
    // If more than 1.0 second has passed since the last command, auto-stop
    if ((this->now() - last_intake_cmd_time_).seconds() > 1.0) {
        // Call set_intake_hardware(0) so the static current_state updates correctly.
        // If it is already 0, the function will safely ignore it.
        set_intake_hardware(0);
    }
}

void IntakeNode::set_intake_hardware(int state) {
    // Track the current state to detect hard reversals
    static int current_state = 0;

    // Ignore redundant commands to prevent unnecessary hardware writes
    if (state == current_state) {
        return;
    }

    switch (state) {
        case 1:  // Forward / Intake On
            RCLCPP_INFO(this->get_logger(), "Intake: FORWARD");
            
            // Safety: Cut power first if transitioning directly from reverse
            if (current_state == -1) {
                pca_driver_->setPWM(INTAKE_PWM_CHANNEL_REV, 0, 4096);
            }

            pca_driver_->setPWM(INTAKE_PWM_CHANNEL_FWD, 0, 3072);
            pca_driver_->setPWM(INTAKE_PWM_CHANNEL_REV, 0, 4096);
            break;
            
        case -1: // Reverse / Outtake
            RCLCPP_INFO(this->get_logger(), "Intake: REVERSE");
            
            // Safety: Cut power first if transitioning directly from forward
            if (current_state == 1) {
                pca_driver_->setPWM(INTAKE_PWM_CHANNEL_FWD, 0, 4096);
            }

            pca_driver_->setPWM(INTAKE_PWM_CHANNEL_FWD, 0, 4096);
            pca_driver_->setPWM(INTAKE_PWM_CHANNEL_REV, 0, 3072);
            break;
            
        case 0:  // Explicit Off command
        default: // Failsafe for unexpected data
            stop_intake();
            break;
    }
    
    // Update the tracker
    current_state = state;
}

void IntakeNode::stop_intake() {
    RCLCPP_INFO(this->get_logger(), "Intake: STOPPED");
    // Immediately cut power to the motor
    if (pca_driver_) {
        pca_driver_->setPWM(INTAKE_PWM_CHANNEL_FWD, 0, 4096);
        pca_driver_->setPWM(INTAKE_PWM_CHANNEL_REV, 0, 4096);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    // Create and spin the node
    auto node = std::make_shared<IntakeNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}