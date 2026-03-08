#ifndef HARDWARE_TEAM_ROBOT_CHASSIS_NODE_H
#define HARDWARE_TEAM_ROBOT_CHASSIS_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hardware_team_robot/action/drive.hpp"
#include "hardware_team_robot/sensors/MPU6050.h"
#include "hardware_team_robot/mecanum_odometry.h"
#include "hardware_team_robot/encoder_driver.h"
#include "hardware_team_robot/UltrasonicDistance.h" // New Ultrasonic Header
#include "hardware_team_robot/sensors/VEML7700.h"  
#include "std_msgs/msg/bool.hpp"
#include <pigpio.h>
#include <atomic>
#include <thread> 



class ChassisNode : public rclcpp::Node {
public:
    using Drive = hardware_team_robot::action::Drive;
    using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

    // Odometry System Interface 
    // Get current position/heading from odometry system
    Hardware::MecanumOdometry::Pose getOdometryPose() const;
    // Reset odometry to origin
    void resetOdometry();

    // Start Light Detection Interface
    bool isStartLightDetected() const { return start_light_detected_; }


    ChassisNode();
    ~ChassisNode();
    
    void handle_encoder_tick(int gpio, int level);
    
private:
    // ROS Action Server & IMU
    rclcpp_action::Server<Drive>::SharedPtr action_server_;
    MPU6050 imu_;

    // Threads for background processes
    std::thread odometry_thread_;  
    std::thread execute_thread_;

    // Hardware abstraction for encoder and odometry 
    std::shared_ptr<Hardware::EncoderDriver> encoder_driver_;
    std::shared_ptr<Hardware::MecanumOdometry> odometry_;
    
    // Hardware abstraction for ultrasonic sensors
    std::shared_ptr<Hardware::UltrasonicDriver> ultrasonic_driver_;

    // VEML7700 light sensor for detecting start LED bar
    std::shared_ptr<VEML7700> start_light_sensor_;
    std::atomic<bool> start_light_detected_{false};
    uint16_t baseline_white_{0};  // Baseline WHITE reading before LEDs turn on
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_light_pub_;

    void odometry_update_loop();  // Continuously reads encoders and updates odometry

    // --- Ultrasonic Odometry Correction ---
    float calculate_y_from_wall(const Hardware::MecanumOdometry::Pose& current_pose, float sensor_distance, float offset_x, float offset_y, float known_wall_y);
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

    // --- PIN DEFINITIONS & CONSTANTS ---
    // Ultrasonic Pins
    static constexpr int PIN_US_LEFT_TRIG = 7;
    static constexpr int PIN_US_LEFT_ECHO = 10;
    static constexpr int PIN_US_RIGHT_TRIG = 14;
    static constexpr int PIN_US_RIGHT_ECHO = 15;

    // Ultrasonic Sensor Displacements (Inches)
    static constexpr float US_LEFT_OFFSET_X = 0.0f;
    static constexpr float US_LEFT_OFFSET_Y = 4.5f;
    static constexpr float US_RIGHT_OFFSET_X = 0.0f;
    static constexpr float US_RIGHT_OFFSET_Y = -4.5f;

    // Known Arena Boundaries (Inches)
    static constexpr float KNOWN_LEFT_WALL_Y = 48.0f;
    static constexpr float KNOWN_RIGHT_WALL_Y = 0.0f;

    // Motors (PWM, IN1, IN2)
    static constexpr int PIN_FL_PWM = 12, PIN_FL_IN1 = 5,  PIN_FL_IN2 = 6;
    static constexpr int PIN_FR_PWM = 13, PIN_FR_IN1 = 23, PIN_FR_IN2 = 24;
    static constexpr int PIN_RL_PWM = 18, PIN_RL_IN1 = 25, PIN_RL_IN2 = 8;
    static constexpr int PIN_RR_PWM = 19, PIN_RR_IN1 = 16, PIN_RR_IN2 = 20;

    // Encoders (Channel A, Channel B)
    static constexpr int PIN_FL_ENC_A = 17, PIN_FL_ENC_B = 27;
    static constexpr int PIN_FR_ENC_A = 22, PIN_FR_ENC_B = 4; 
    static constexpr int PIN_RL_ENC_A = 26, PIN_RL_ENC_B = 21;
    static constexpr int PIN_RR_ENC_A = 9,  PIN_RR_ENC_B = 11;

    // ==================== VEML7700 NOTES ====================
    // VEML7700 uses I2C Bus 1 (shared with MPU6050)
    // Address: 0x10 (no GPIO pins needed)
    // Purpose: ONE-TIME detection of competition start LED bars
    // ========================================================
};

#endif // HARDWARE_TEAM_ROBOT_CHASSIS_NODE_H