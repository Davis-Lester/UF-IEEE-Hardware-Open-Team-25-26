#ifndef HARDWARE_TEAM_ROBOT_CHASSIS_NODE_H
#define HARDWARE_TEAM_ROBOT_CHASSIS_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hardware_team_robot/action/drive.hpp"
#include "hardware_team_robot/sensors/MPU6050.h" 
#include <pigpio.h>
#include <atomic>

class ChassisNode : public rclcpp::Node {
public:
    using Drive = hardware_team_robot::action::Drive;
    using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

    // Odometry System Interface 
    // Get current position/heading from odometry system
    Hardware::MecanumOdometry::Pose getOdometryPose() const;
    // Reset odometry to origin
    void resetOdometry();

    ChassisNode();
    
    void handle_encoder_tick(int gpio, int level);
private:
    // ROS Action Server & IMU
    rclcpp_action::Server<Drive>::SharedPtr action_server_;
    MPU6050 imu_;

    // Hardware abstraction for encoder and odometry 
    std::shared_ptr<Hardware::EncoderDriver> encoder_driver_;
    std::shared_ptr<Hardware::MecanumOdometry> odometry_;
    void odometry_update_loop();  // Continuously reads encoders and updates odometry

    // Callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Drive::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDrive> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleDrive> goal_handle);
    void execute(const std::shared_ptr<GoalHandleDrive> goal_handle);

    // --- ENCODERS (4 Wheels) ---
    // Atomic variables for thread-safe ISR access
    std::atomic<long> fl_ticks_{0}, fr_ticks_{0};
    std::atomic<long> rl_ticks_{0}, rr_ticks_{0};

    void setup_encoders();
    
    
    // --- MOTORS (4 Wheels) ---
    void setup_motor_pins();
    // New: Individual control for mixing
    void set_mecanum_power(double fl, double fr, double rl, double rr);
    void stop_motors();

    // --- PIN DEFINITIONS (Check your wiring!) ---
    // Motors (PWM, IN1, IN2)
    // Front Left
    static const int FL_PWM = 12, FL_IN1 = 5, FL_IN2 = 6;
    // Front Right
    static const int FR_PWM = 13, FR_IN1 = 16, FR_IN2 = 19;
    // Rear Left
    static const int RL_PWM = 18, RL_IN1 = 20, RL_IN2 = 21;
    // Rear Right
    static const int RR_PWM = 19, RR_IN1 = 26, RR_IN2 = 20; // Note: GPIO 20 used twice? Check free pins!
    // *Correction*: GPIO 20/21 are used above. Let's fix RR to 26 & 4.
    // Fixed Pinout below:

    // Encoders (Channel A, Channel B)
    static const int FL_ENC_A = 17, FL_ENC_B = 27;
    static const int FR_ENC_A = 22, FR_ENC_B = 10; // 10 is SPI MOSI, ensure SPI is off or use other
    static const int RL_ENC_A = 9,  RL_ENC_B = 11; // SPI MISO/SCLK
    static const int RR_ENC_A = 0,  RR_ENC_B = 1;  // ID_SD/SC (Reserved?) -> Use 2, 3
};

#endif