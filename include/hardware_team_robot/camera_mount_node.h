#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pigpio.h>
#include <atomic>

enum MountState
{
    UNKNOWN,
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

    // GPIO Pins (dont know whci ones we will use
    const int PIN_MOTOR_A = 17;
    const int PIN_MOTOR_B = 27;
    const int PIN_PWM = 18;

    // Motion timings to change
    const float MAX_TRAVEL_TIME = 2.0;
    const float TIME_STOWED = 0.0;
    const float TIME_DRIVE = 1.2;
    const float TIME_MAX = 2.0;

    const int MAX_PWM = 200;


    float current_time_pos_;
    MountState current_state_;

    bool pigpio_ready_;
    std::atomic<bool> cancel_motion_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

    void cmd_callback(const std_msgs::msg::String::SharedPtr msg);

    void move_to_time(float target_time);
    void set_direction(bool up);
    void ramp_motion(float duration);

    void stop_motor();
    void emergency_stop();
};
