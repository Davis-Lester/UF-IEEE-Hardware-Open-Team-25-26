#ifndef INTAKE_NODE_H
#define INTAKE_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

class IntakeNode : public rclcpp::Node {
public:
    IntakeNode();
    
    // The destructor is critical for safety: it ensures stop_intake() 
    // is called if the node crashes or is gracefully shut down.
    ~IntakeNode();

private:
    rclcpp::Time last_intake_cmd_time_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    void watchdog_callback();

    // Subscriber for non-blocking intake commands (-1: Reverse, 0: Off, 1: On)
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr intake_sub_;
    
    // Callback to process incoming intake states
    void intake_callback(const std_msgs::msg::Int8::SharedPtr msg);

    // Translates the integer state into hardware GPIO/PWM writes
    void set_intake_hardware(int state);
    
    // Safety helper to immediately cut power to the motor
    void stop_intake();


    // HARDWARE PIN DEFINITIONS (these need to be updated)
    // WARNING: Do NOT use the following reserved pins:
    // - Ultrasonic: 7, 10, 14, 15
    // - Encoders: 17, 27, 22, 4, 26, 21, 9, 11

    
    // Selected safe pins for the intake motor driver (WILL NEED TO BE MODIFIED)
    static constexpr int INTAKE_PWM_PIN = 12; 
    static constexpr int INTAKE_DIR_PIN = 5;  
};

#endif // INTAKE_NODE_H