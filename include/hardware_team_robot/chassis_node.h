#ifndef HARDWARE_TEAM_ROBOT_CHASSIS_NODE_H
#define HARDWARE_TEAM_ROBOT_CHASSIS_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hardware_team_robot/action/drive.hpp"
#include "hardware_team_robot/sensors/MPU6050.h"
#include "hardware_team_robot/tank_odometry.h"
#include "hardware_team_robot/encoder_driver.h"
#include "hardware_team_robot/sensors/pca9685_driver.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"
#include <pigpio.h>
#include <atomic>
#include <thread>
#include <mutex> 

class ChassisNode : public rclcpp::Node {

    // RGB LED GPIO pins used for visual feedback (shared with camera)
    static constexpr int RGB_PIN_RED = 13;
    static constexpr int RGB_PIN_GREEN = 19;
    static constexpr int RGB_PIN_BLUE = 26;
public:
    using Drive = hardware_team_robot::action::Drive;
    using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

    // Odometry System Interface 
    // Get current position/heading from odometry system
    Hardware::TankOdometry::Pose getOdometryPose() const;
    // Reset odometry to origin
    void resetOdometry();

    ChassisNode();
    ~ChassisNode();
    
    //void handle_encoder_tick(int gpio, int level);
    
private:
    // ROS Action Server & IMU
    rclcpp_action::Server<Drive>::SharedPtr action_server_;
    MPU6050 imu_;

    //Motor Driver
    std::shared_ptr<Hardware::PCA9685Driver> motor_driver_;
    std::atomic<bool> motor_ready_{false};
    std::mutex motor_mutex_;  // Protects motor control operations
    std::atomic<bool> action_active_{false};  // Indicates if an action is currently controlling motors

    // Threads for background processes
    std::thread odometry_thread_;  
    std::thread execute_thread_;

    // Hardware abstraction for encoder and odometry 
    std::shared_ptr<Hardware::EncoderDriver> encoder_driver_;
    std::shared_ptr<Hardware::TankOdometry> odometry_;

    // subscriptions
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr led_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr intake_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motor_cmd_sub_;

    void odometry_update_loop();  // Continuously reads encoders and updates odometry

    // Callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Drive::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDrive> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleDrive> goal_handle);
    void execute(const std::shared_ptr<GoalHandleDrive> goal_handle);
    void motor_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void led_callback(const std_msgs::msg::ColorRGBA::SharedPtr msg);
    void intake_callback(const std_msgs::msg::Int8::SharedPtr msg);

    // // --- ENCODERS (4 Wheels) ---
    // // Atomic variables for thread-safe ISR access
    // std::atomic<long> fl_ticks_{0}, fr_ticks_{0};
    // std::atomic<long> rl_ticks_{0}, rr_ticks_{0};

    
    // --- MOTORS (4 Wheels) ---
    //_motor_pins();
    // New: Individual control for mixing
    void set_tank_power(double left, double right); 
    void stop_motors();

    // --- PIN DEFINITIONS & CONSTANTS ---
    
    // Motors (PWM, IN1, IN2) I am commenting cause i dont think we need this
    // static constexpr int PIN_FL_PWM = 12, PIN_FL_IN1 = 5,  PIN_FL_IN2 = 6;
    // static constexpr int PIN_FR_PWM = 13, PIN_FR_IN1 = 23, PIN_FR_IN2 = 24;
    // static constexpr int PIN_RL_PWM = 18, PIN_RL_IN1 = 25, PIN_RL_IN2 = 8;
    // static constexpr int PIN_RR_PWM = 19, PIN_RR_IN1 = 16, PIN_RR_IN2 = 20;

    // Encoders (Channel A, Channel B)
    // PINS UPDATED
    static constexpr int PIN_FL_ENC_A = 24, PIN_FL_ENC_B = 25;
    static constexpr int PIN_FR_ENC_A = 8, PIN_FR_ENC_B = 7; 
    static constexpr int PIN_RL_ENC_A = 16, PIN_RL_ENC_B = 12;
    static constexpr int PIN_RR_ENC_A = 23,  PIN_RR_ENC_B = 18;

    
};

#endif // HARDWARE_TEAM_ROBOT_CHASSIS_NODE_H