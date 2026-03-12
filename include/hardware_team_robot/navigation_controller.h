#ifndef NAVIGATION_CONTROLLER_H
#define NAVIGATION_CONTROLLER_H

#include "hardware_team_robot/tank_odometry.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <cmath>
#include <chrono>

// Forward declarations to prevent circular dependencies
class ChassisNode;
class MPU6050;

namespace Hardware {

class NavigationController {
public:
    // Constructor requires non-null odometry and ROS node
    // chassis_interface is optional but motors won't move without it
    NavigationController(std::shared_ptr<TankOdometry> odom,
                         std::shared_ptr<rclcpp::Node> rclcpp_node,
                         ChassisNode* chassis_interface = nullptr);
    
    ~NavigationController();

    // Returns true if controller is properly initialized
    bool isValid() const { return is_valid_; }

    bool moveToPose(float target_x_inches,
                    float target_y_inches,
                    float target_theta_rad,
                    float max_speed = 150.0f,
                    float tolerance_inches = 1.0f,
                    float angle_tolerance_rad = 0.035f);

    bool moveToPoint(float target_x_inches,
                     float target_y_inches,
                     float max_speed = 150.0f,
                     float tolerance_inches = 1.0f);

    TankOdometry::Pose getCurrentPose() const;
    void resetOdometry();
    void stop();
    void setPIDGains(float distance_kp,
                     float distance_ki,
                     float distance_kd,
                     float heading_kp,
                     float heading_ki,
                     float heading_kd);
    void setDebugLogging(bool enable);
    
    // Set chassis interface after construction if needed
    void setChassisInterface(ChassisNode* chassis_interface);

private:
    std::shared_ptr<TankOdometry> odometry_;
    std::shared_ptr<rclcpp::Node> node_;
    ChassisNode* chassis_;  // Raw pointer - lifetime managed externally
    bool is_valid_;         // True if node_ and odometry_ are non-null
    bool debug_logging_{false};

    struct PIDController {
        float kp{1.0f};
        float ki{0.0f};
        float kd{0.1f};
        float integral{0.0f};
        float last_error{0.0f};
        float output{0.0f};
        float calculate(float error, float dt);
        void reset();
    };

    PIDController heading_pid_;
    PIDController distance_pid_;

    static constexpr float MOVEMENT_TIMEOUT_SEC = 10.0f;

    float calculateDistance(float x1, float y1,
                            float x2, float y2) const;
    float calculateHeading(float dx, float dy) const;
    float normalizeAngle(float angle_rad) const;

    // Internal motion primitives
    bool turnToHeading(float target_heading_rad,
                       float tolerance_rad = 0.035f,
                       float max_speed = 150.0f);
    
    bool driveDistance(float target_distance_inches,
                       float desired_heading_rad,
                       float tolerance_inches = 1.0f,
                       float max_speed = 150.0f);
    
    bool driveToPoint(float target_x_inches,
                      float target_y_inches,
                      float tolerance_inches = 1.0f,
                      float max_speed = 150.0f);
    
    // Helper to send tank commands safely
    void sendTankSpeeds(float left_speed, float right_speed);
};

} // namespace Hardware

#endif // NAVIGATION_CONTROLLER_H