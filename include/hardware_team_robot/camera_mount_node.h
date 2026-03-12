#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pigpio.h>

enum MountState
{
    STOWED,
    DRIVE_HEIGHT,
    MAX_ELEVATION
};

class CameraMountNode : public rclcpp::Node
{
public:
    CameraMountNode();
    ~CameraMountNode();

private:

    // GPIO Pins (Dont know which ones to use)
    const int PIN_MOTOR_A = ;
    const int PIN_MOTOR_B = ;
    const int PIN_PWM = ;

    // Can change timings
    const float MAX_TRAVEL_TIME = 2.0;

    const float TIME_STOWED = 0.0;
    const float TIME_DRIVE = 1.2;
    const float TIME_MAX = 2.0;
    const int MAX_PWM = 200;

    float current_time_pos_;
    MountState current_state_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    void cmd_callback(const std_msgs::msg::String::SharedPtr msg);

    void move_to_time(float target_time);

    void set_direction(bool up);

    void ramp_motion(float duration);

    void stop_motor();

    void emergency_stop();
};
