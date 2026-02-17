#ifndef NAVIGATION_CONTROLLER_H
#define NAVIGATION_CONTROLLER_H

#include "hardware_team_robot/mecanum_odometry.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <cmath>
#include <chrono>

// Forward declarations to prevent circular dependencies
class ChassisNode;
class MPU6050;

namespace Hardware {

/*
 * NavigationController
 *
 * High-level motion controller for autonomous robot movement.
 *
 * Provides simple movement primitives built on top of odometry feedback
 * and heading measurements:
 *
 *   • moveToPose(x, y, θ)   Drive to position with specified orientation
 *   • moveToPoint(x, y)     Drive to position without orientation constraint
 *
 * Motion is regulated using PID controllers for:
 *
 *   • Heading correction (rotation)
 *   • Linear distance control (forward)
 *   • Lateral distance control (strafe)
 *
 * Designed to integrate with:
 *
 *   • MecanumOdometry for position feedback
 *   • IMU for absolute heading
 *   • Chassis controller for wheel commands
 *
 * Concurrency Model:
 *
 *   Odometry and sensor reads are thread-safe. Controller execution is
 *   intended for a single control loop thread to maintain deterministic
 *   behavior.
 */

class NavigationController {
public:

    // Requires access to odometry and ROS node utilities
    NavigationController(std::shared_ptr<MecanumOdometry> odom,
                         std::shared_ptr<rclcpp::Node> rclcpp_node);

    ~NavigationController();

    // Drives to a target position and final orientation
    bool moveToPose(float target_x_inches,
                    float target_y_inches,
                    float target_theta_rad,
                    float max_speed = 150.0f,
                    float tolerance_inches = 1.0f,
                    float angle_tolerance_rad = 0.035f);

    // Drives to a target position while ignoring final heading
    bool moveToPoint(float target_x_inches,
                     float target_y_inches,
                     float max_speed = 150.0f,
                     float tolerance_inches = 1.0f);

    // Returns latest pose estimate from odometry
    MecanumOdometry::Pose getCurrentPose() const;

    // Resets odometry state
    void resetOdometry();

    // Immediately halts all commanded motion
    void stop();

    // Updates controller gains for distance and heading loops
    void setPIDGains(float distance_kp,
                     float distance_ki,
                     float distance_kd,
                     float heading_kp,
                     float heading_ki,
                     float heading_kd);

    // Enables or disables diagnostic logging
    void setDebugLogging(bool enable);

private:

    // Shared subsystem interfaces
    std::shared_ptr<MecanumOdometry> odometry_;
    std::shared_ptr<rclcpp::Node> node_;

    // Debug control
    bool debug_logging_{false};

    /*
     * PIDController
     *
     * Minimal PID regulator used for motion control loops.
     * Maintains internal state for integral accumulation and
     * derivative estimation.
     */
    struct PIDController {
        float kp{1.0f};
        float ki{0.0f};
        float kd{0.1f};

        float integral{0.0f};
        float last_error{0.0f};
        float output{0.0f};

        // Computes controller output from error and timestep
        float calculate(float error, float dt);

        // Clears accumulated state
        void reset();
    };

    // Controllers for each motion component
    PIDController heading_pid_;
    PIDController distance_pid_;
    PIDController strafe_pid_;

    // Maximum duration allowed for a single movement command
    static constexpr float MOVEMENT_TIMEOUT_SEC = 10.0f;

    // Basic geometric helpers
    float calculateDistance(float x1, float y1,
                            float x2, float y2) const;

    float calculateHeading(float dx, float dy) const;

    float normalizeAngle(float angle_rad) const;

    // Internal motion primitives
    bool turnToHeading(float target_heading_rad,
                       float tolerance_rad = 0.035f);

    bool driveDistance(float target_distance_inches,
                       float desired_heading_rad,
                       float tolerance_inches = 1.0f);

    bool driveToPoint(float target_x_inches,
                      float target_y_inches,
                      float tolerance_inches = 1.0f);
};

} // namespace Hardware

#endif // NAVIGATION_CONTROLLER_H
