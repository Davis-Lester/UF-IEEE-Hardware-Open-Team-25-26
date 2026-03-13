#ifndef AUTON_ROUTINE_H
#define AUTON_ROUTINE_H

// Name: Davis Lester
// Date: 2/5/2026
// Description: Creating a Chassis PID Control Library for ROS2 Robot

//╭━━━╮╱╱╭╮╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭━━━╮╱╱╱╱╱╭╮╱╱╱╱╱╭╮╱╭╮╱╱╱╭╮
//┃╭━╮┃╱╭╯╰╮╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱┃╭━╮┃╱╱╱╱╭╯╰╮╱╱╱╱┃┃╱┃┃╱╱╱┃┃
//┃┃╱┃┣╮┣╮╭╋━━┳━╮╭━━┳╮╭┳━━┳╮╭┳━━╮┃┃╱╰╋━━┳━╋╮╭╋━┳━━┫┃╱┃┃╱╱╭┫╰━┳━┳━━┳━┳╮╱╭╮
//┃╰━╯┃┃┃┃┃┃╭╮┃╭╮┫╭╮┃╰╯┃╭╮┃┃┃┃━━┫┃┃╱╭┫╭╮┃╭╮┫┃┃╭┫╭╮┃┃╱┃┃╱╭╋┫╭╮┃╭┫╭╮┃╭┫┃╱┃┃
//┃╭━╮┃╰╯┃╰┫╰╯┃┃┃┃╰╯┃┃┃┃╰╯┃╰╯┣━━┃┃╰━╯┃╰╯┃┃┃┃╰┫┃┃╰╯┃╰╮┃╰━╯┃┃╰╯┃┃┃╭╮┃┃┃╰━╯┃
//╰╯╱╰┻━━┻━┻━━┻╯╰┻━━┻┻┻┻━━┻━━┻━━╯╰━━━┻━━┻╯╰┻━┻╯╰━━┻━╯╰━━━┻┻━━┻╯╰╯╰┻╯╰━╮╭╯
//╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭━╯┃
//╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╰━━╯

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hardware_team_robot/action/drive.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int8.hpp"
#include <string>
#include <atomic>
#include <thread>

#ifndef USE_START_LIGHT
#define USE_START_LIGHT 1
#endif

class AutonRoutine : public rclcpp::Node {
public:
    using Drive = hardware_team_robot::action::Drive;
    using GoalHandleDrive = rclcpp_action::ClientGoalHandle<Drive>;

    AutonRoutine();
    ~AutonRoutine();

private:
    // Clients & Publishers
    rclcpp_action::Client<Drive>::SharedPtr client_ptr_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ir_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr intake_pub_; // pubish to this to com w intake

    std::atomic<int> current_intake_state_{0};
    rclcpp::TimerBase::SharedPtr intake_publish_timer_;
    void intake_publish_callback();
    
#if USE_START_LIGHT
    // Start Light Detection Subscriber
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_light_sub_;
    std::atomic<bool> start_detected_{false};
#endif

    std::thread routine_thread_;
    
    void set_intake(int state);
    // Routine Logic
    void run_routine();
    
    // Helper to mimic EZ-Template blocking calls
    bool wait_for_drive(std::string mode, double target_value, double max_speed = 100.0);
    
    void start_light_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void check_and_run(); 
};

#endif // AUTON_ROUTINE_H