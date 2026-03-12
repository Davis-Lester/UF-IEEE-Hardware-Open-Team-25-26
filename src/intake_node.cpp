#include "hardware_team_robot/intake_node.h"
#include <pigpio.h>
#include <stdexcept>

IntakeNode::IntakeNode() : Node("intake_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing Intake Node...");

    // ==========================================
    // 1. HARDWARE INITIALIZATION (pigpio)
    // ==========================================
    // Initialize the pigpio library. This requires root privileges!
    if (gpioInitialise() < 0) {
        RCLCPP_FATAL(this->get_logger(), "pigpio initialization failed! Are you running the node with sudo?");
        throw std::runtime_error("pigpio initialization failed");
    }

    // Configure the motor driver pins as outputs
    gpioSetMode(INTAKE_DIR_PIN, PI_OUTPUT);
    gpioSetMode(INTAKE_PWM_PIN, PI_OUTPUT);

    // Safety: Ensure the motor is strictly off at startup
    stop_intake();

    // ==========================================
    // 2. ROS 2 INTERFACES
    // ==========================================
    intake_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/intake_cmd", 10,
        std::bind(&IntakeNode::intake_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Intake Node ready and listening on /intake_cmd");
}

IntakeNode::~IntakeNode() {
    RCLCPP_WARN(this->get_logger(), "Shutting down Intake Node. Stopping motor.");
    
    // Safety: Guarantee motor stops if node crashes or is killed
    stop_intake();
    
    // Release the GPIO resources
    gpioTerminate();
}

void IntakeNode::intake_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    // Pass the requested state directly to the hardware logic
    set_intake_hardware(msg->data);
}

void IntakeNode::set_intake_hardware(int state) {
    switch (state) {
        case 1:  // Forward / Intake On
            RCLCPP_INFO(this->get_logger(), "Intake: FORWARD");
            gpioWrite(INTAKE_DIR_PIN, 1);
            gpioPWM(INTAKE_PWM_PIN, 255); // 100% duty cycle (0-255 range)
            break;
            
        case -1: // Reverse / Outtake
            RCLCPP_INFO(this->get_logger(), "Intake: REVERSE");
            gpioWrite(INTAKE_DIR_PIN, 0);
            gpioPWM(INTAKE_PWM_PIN, 255); // 100% duty cycle (0-255 range)
            break;
            
        case 0:  // Explicit Off command
        default: // Failsafe for unexpected data
            stop_intake();
            break;
    }
}

void IntakeNode::stop_intake() {
    RCLCPP_INFO(this->get_logger(), "Intake: STOPPED");
    // Immediately cut power to the motor
    gpioPWM(INTAKE_PWM_PIN, 0);
    gpioWrite(INTAKE_DIR_PIN, 0);
}

// ==========================================
// MAIN EXECUTABLE ENTRY POINT
// ==========================================
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    // Create and spin the node
    auto node = std::make_shared<IntakeNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}