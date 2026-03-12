#include "hardware_team_robot/camera_mount_node.h"
#include <chrono>
#include <cmath>
#include <algorithm>
using namespace std::chrono_literals;

CameraMountNode::CameraMountNode()
: Node("camera_mount_node"),
  pigpio_ready_(false),
  cancel_motion_(false),
  current_time_pos_(0.0),
  current_state_(STOWED)
{
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/camera/mount_cmd",
        10,
        std::bind(&CameraMountNode::cmd_callback, this, std::placeholders::_1)
    );

    if(gpioInitialise() < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Pigpio failed to start");
        return;
    }

    pigpio_ready_ = true;

    gpioSetMode(PIN_MOTOR_A, PI_OUTPUT);
    gpioSetMode(PIN_MOTOR_B, PI_OUTPUT);
    gpioSetMode(PIN_PWM, PI_OUTPUT);

    stop_motor();

    RCLCPP_INFO(this->get_logger(), "Camera Mount Node Started");
}

CameraMountNode::~CameraMountNode()
{
    if(pigpio_ready_)
    {
        stop_motor();
        gpioTerminate();
    }
}

void CameraMountNode::cmd_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if(!pigpio_ready_)
    {
        RCLCPP_WARN(this->get_logger(),"Pigpio not initialized");
        return;
    }

    std::string command = msg->data;

    if(command == "STOWED")
    {
        move_to_time(TIME_STOWED);
        current_state_ = STOWED;
    }
    else if(command == "DRIVE_HEIGHT")
    {
        move_to_time(TIME_DRIVE);
        current_state_ = DRIVE_HEIGHT;
    }
    else if(command == "MAX_ELEVATION")
    {
        move_to_time(TIME_MAX);
        current_state_ = MAX_ELEVATION;
    }
    else if(command == "STOP")
    {
        cancel_motion_ = true;
        emergency_stop();
    }
}

void CameraMountNode::move_to_time(float target_time)
{
    if(!pigpio_ready_) return;

    cancel_motion_ = false;

    float delta = target_time - current_time_pos_;

    if(std::abs(delta) < 0.05)
        return;

    bool moving_up = delta > 0;

    set_direction(moving_up);

    float move_time = std::min(std::abs(delta), MAX_TRAVEL_TIME);

    ramp_motion(move_time);

    stop_motor();

    current_time_pos_ = std::clamp(target_time, 0.0f, MAX_TRAVEL_TIME);
}

void CameraMountNode::set_direction(bool up)
{
    if(!pigpio_ready_) return;

    if(up)
    {
        gpioWrite(PIN_MOTOR_A, 1);
        gpioWrite(PIN_MOTOR_B, 0);
    }
    else
    {
        gpioWrite(PIN_MOTOR_A, 0);
        gpioWrite(PIN_MOTOR_B, 1);
    }
}

void CameraMountNode::ramp_motion(float duration)
{
    if(!pigpio_ready_) return;

    const int ramp_steps = 20;
    const int ramp_delay_ms = 10;

    const float ramp_time_s = (2.0f * ramp_steps * ramp_delay_ms) / 1000.0f;
    const float cruise_time = std::max(0.0f, duration - ramp_time_s);

    for(int i = 1; i <= ramp_steps; ++i)
    {
        if(cancel_motion_) return;

        int pwm = (MAX_PWM * i) / ramp_steps;

        gpioPWM(PIN_PWM, pwm);

        rclcpp::sleep_for(std::chrono::milliseconds(ramp_delay_ms));
    }

    auto start = std::chrono::steady_clock::now();

    while(true)
    {
        if(cancel_motion_) return;

        auto now = std::chrono::steady_clock::now();

        float elapsed =
            std::chrono::duration<float>(now - start).count();

        if(elapsed >= cruise_time)
            break;

        rclcpp::sleep_for(10ms);
    }

    for(int i = ramp_steps - 1; i >= 0; --i)
    {
        if(cancel_motion_) return;

        int pwm = (MAX_PWM * i) / ramp_steps;

        gpioPWM(PIN_PWM, pwm);

        rclcpp::sleep_for(std::chrono::milliseconds(ramp_delay_ms));
    }
}

void CameraMountNode::stop_motor()
{
    if(!pigpio_ready_) return;

    gpioPWM(PIN_PWM, 0);
    gpioWrite(PIN_MOTOR_A, 0);
    gpioWrite(PIN_MOTOR_B, 0);
}

void CameraMountNode::emergency_stop()
{
    RCLCPP_WARN(this->get_logger(), "Emergency Stop Activated");

    stop_motor();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CameraMountNode>());

    rclcpp::shutdown();

    return 0;
}
