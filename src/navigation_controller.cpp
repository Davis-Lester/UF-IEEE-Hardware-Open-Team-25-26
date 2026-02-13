
#include "Hardware_Team_Robot/navigation_controller.h"
#include <cstdio>
#include <thread>
#include <chrono>
#include <cmath>

namespace Hardware {

float NavigationController::PIDController::calculate(float error, float dt) {
    if (dt <= 0.0f) dt = 0.01f;  // Prevent division by zero
    
    // Proportional term
    float p_term = kp * error;
    
    // Integral term (with anti-windup - don't accumulate if output is saturated)
    integral += error * dt;
    // Clamp integral to reasonable bounds
    if (integral > 10.0f) integral = 10.0f;
    if (integral < -10.0f) integral = -10.0f;
    float i_term = ki * integral;
    
    // Derivative term (with low-pass filter to reduce noise)
    float d_term = 0.0f;
    if (dt > 0.0f) {
        d_term = kd * (error - last_error) / dt;
    }
    last_error = error;
    
    // Calculate total output
    output = p_term + i_term + d_term;
    
    // Clamp output to reasonable motor command range
    if (output > 255.0f) output = 255.0f;
    if (output < -255.0f) output = -255.0f;
    
    return output;
}

void NavigationController::PIDController::reset() {
    integral = 0.0f;
    last_error = 0.0f;
    output = 0.0f;
}

NavigationController::NavigationController(std::shared_ptr<MecanumOdometry> odom,
                                         std::shared_ptr<rclcpp::Node> rclcpp_node)
    : odometry_(odom), node_(rclcpp_node) {
    
    // Initialize PID controllers with reasonable defaults
    // These may need tuning based on actual robot performance
    distance_pid_.kp = 0.5f;
    distance_pid_.ki = 0.05f;
    distance_pid_.kd = 0.2f;
    
    heading_pid_.kp = 2.0f;
    heading_pid_.ki = 0.1f;
    heading_pid_.kd = 0.3f;
    
    strafe_pid_.kp = 0.5f;
    strafe_pid_.ki = 0.05f;
    strafe_pid_.kd = 0.2f;
    
    RCLCPP_INFO(node_->get_logger(), "[NavigationController] Initialized");
}

NavigationController::~NavigationController() {
    stop();
}

bool NavigationController::moveToPose(float target_x_inches, float target_y_inches,
                                      float target_theta_rad,
                                      float max_speed, float tolerance_inches,
                                      float angle_tolerance_rad) {
    if (!odometry_) {
        RCLCPP_ERROR(node_->get_logger(), "Odometry not initialized");
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(),
        "[NavigationController] moveToPose: (%.1f, %.1f) θ=%.3f rad",
        target_x_inches, target_y_inches, target_theta_rad);
    
    // Step 1: Turn to face target
    MecanumOdometry::Pose current = odometry_->getCurrentPose();
    float dx = target_x_inches - current.x_inches;
    float dy = target_y_inches - current.y_inches;
    float target_heading = calculateHeading(dx, dy);
    
    if (!turnToHeading(target_heading, angle_tolerance_rad)) {
        RCLCPP_WARN(node_->get_logger(), "Turn to heading timed out");
        return false;
    }
    
    // Step 2: Drive to target position
    float distance = calculateDistance(current.x_inches, current.y_inches,
                                      target_x_inches, target_y_inches);
    
    if (distance > tolerance_inches) {
        if (!driveDistance(distance, target_heading, tolerance_inches)) {
            RCLCPP_WARN(node_->get_logger(), "Drive distance timed out");
            return false;
        }
    }
    
    // Step 3: Turn to final heading
    if (!turnToHeading(target_theta_rad, angle_tolerance_rad)) {
        RCLCPP_WARN(node_->get_logger(), "Final heading turn timed out");
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "[NavigationController] moveToPose: COMPLETE");
    return true;
}

bool NavigationController::moveToPoint(float target_x_inches, float target_y_inches,
                                       float max_speed, float tolerance_inches) {
    if (!odometry_) {
        RCLCPP_ERROR(node_->get_logger(), "Odometry not initialized");
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(),
        "[NavigationController] moveToPoint: (%.1f, %.1f)",
        target_x_inches, target_y_inches);
    
    if (driveToPoint(target_x_inches, target_y_inches, tolerance_inches)) {
        RCLCPP_INFO(node_->get_logger(), "[NavigationController] moveToPoint: COMPLETE");
        return true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "moveToPoint timed out");
        return false;
    }
}

MecanumOdometry::Pose NavigationController::getCurrentPose() const {
    if (odometry_) {
        return odometry_->getCurrentPose();
    }
    return {0.0f, 0.0f, 0.0f};
}

void NavigationController::resetOdometry() {
    if (odometry_) {
        odometry_->reset();
        RCLCPP_INFO(node_->get_logger(), "[NavigationController] Odometry reset");
    }
}

void NavigationController::stop() {
    // TODO: Send zero velocity commands to chassis
    // This would call into the ChassisNode to stop all motors
    RCLCPP_DEBUG(node_->get_logger(), "[NavigationController] Stop command issued");
}

void NavigationController::setPIDGains(float distance_kp, float distance_ki, float distance_kd,
                                      float heading_kp, float heading_ki, float heading_kd) {
    distance_pid_.kp = distance_kp;
    distance_pid_.ki = distance_ki;
    distance_pid_.kd = distance_kd;
    
    heading_pid_.kp = heading_kp;
    heading_pid_.ki = heading_ki;
    heading_pid_.kd = heading_kd;
    
    RCLCPP_INFO(node_->get_logger(),
        "[NavigationController] PID gains updated - Distance(%.2f,%.2f,%.2f) Heading(%.2f,%.2f,%.2f)",
        distance_kp, distance_ki, distance_kd, heading_kp, heading_ki, heading_kd);
}

void NavigationController::setDebugLogging(bool enable) {
    debug_logging_ = enable;
}

float NavigationController::calculateDistance(float x1, float y1, float x2, float y2) const {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

float NavigationController::calculateHeading(float dx, float dy) const {
    return std::atan2(dy, dx);
}

float NavigationController::normalizeAngle(float angle_rad) const {
    // Normalize to [-π, π]
    while (angle_rad > M_PI) angle_rad -= 2.0f * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0f * M_PI;
    return angle_rad;
}

bool NavigationController::turnToHeading(float target_heading_rad, float tolerance_rad) {
    // IMPORTANT: This is a BLOCKING call. In a real ROS 2 system, this should be refactored
    // to use an action server or timer-based approach. For now, this works for autonomous
    // routines that are OK with blocking movement commands.
    
    auto start_time = std::chrono::high_resolution_clock::now();
    PIDController pid = heading_pid_;  // Local copy for this movement
    
    while (true) {
        // Check timeout
        auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
        float elapsed_sec = std::chrono::duration<float>(elapsed).count();
        if (elapsed_sec > MOVEMENT_TIMEOUT_SEC) {
            RCLCPP_WARN(node_->get_logger(), "turnToHeading: TIMEOUT after %.1f sec", elapsed_sec);
            return false;
        }
        
        // Get current heading
        auto current = odometry_->getCurrentPose();
        float current_heading = current.theta_rad;
        
        // Calculate error (normalize to [-π, π])
        float error = normalizeAngle(target_heading_rad - current_heading);
        
        // Check if at target
        if (std::abs(error) < tolerance_rad) {
            RCLCPP_INFO(node_->get_logger(), "turnToHeading: Reached target heading");
            return true;
        }
        
        // Calculate dt for PID
        float dt = 0.01f;  // 10ms
        
        // PID control
        float control_output = pid.calculate(error, dt);
        
        if (debug_logging_) {
            RCLCPP_INFO(node_->get_logger(),
                "turnToHeading: error=%.3f rad, output=%.1f", error, control_output);
        }
        
        // TODO: Send rotation command to chassis
        // NOTE: This would call a method on ChassisNode or similar to actually control motors
        // For now, this is a framework/planning method
        // chassis_->sendWheelVelocities(fl_speed, fr_speed, rl_speed, rr_speed);
        
        // Sleep briefly to cool CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool NavigationController::driveDistance(float target_distance_inches, float desired_heading_rad,
                                        float tolerance_inches) {
    // IMPORTANT: This is a BLOCKING call. See note in turnToHeading().
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto start_pose = odometry_->getCurrentPose();
    
    while (true) {
        // Check timeout
        auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
        float elapsed_sec = std::chrono::duration<float>(elapsed).count();
        if (elapsed_sec > MOVEMENT_TIMEOUT_SEC) {
            RCLCPP_WARN(node_->get_logger(), "driveDistance: TIMEOUT after %.1f sec", elapsed_sec);
            return false;
        }
        
        // Calculate distance traveled
        auto current = odometry_->getCurrentPose();
        float distance_traveled = calculateDistance(start_pose.x_inches, start_pose.y_inches,
                                                   current.x_inches, current.y_inches);
        
        float error = target_distance_inches - distance_traveled;
        
        // Check if at target
        if (std::abs(error) < tolerance_inches) {
            RCLCPP_INFO(node_->get_logger(), "driveDistance: Reached target");
            return true;
        }
        
        // Calculate dt for PID
        float dt = 0.01f;
        
        // PID for forward motion
        float forward_output = distance_pid_.calculate(error, dt);
        
        // Minor heading correction if needed
        float heading_error = normalizeAngle(desired_heading_rad - current.theta_rad);
        float heading_correction = heading_pid_.calculate(heading_error, dt);
        
        if (debug_logging_) {
            RCLCPP_INFO(node_->get_logger(),
                "driveDistance: traveled=%.1f/%.1f inches, forward_out=%.1f, heading_corr=%.1f",
                distance_traveled, target_distance_inches, forward_output, heading_correction);
        }
        
        // TODO: Send forward and rotation commands to chassis
        // NOTE: Need to actually call motor control methods here
        
        // Sleep briefly
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool NavigationController::driveToPoint(float target_x_inches, float target_y_inches,
                                       float tolerance_inches) {
    // IMPORTANT: This is a BLOCKING call. See note in turnToHeading().
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (true) {
        // Check timeout
        auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
        float elapsed_sec = std::chrono::duration<float>(elapsed).count();
        if (elapsed_sec > MOVEMENT_TIMEOUT_SEC) {
            RCLCPP_WARN(node_->get_logger(), "driveToPoint: TIMEOUT after %.1f sec", elapsed_sec);
            return false;
        }
        
        // Get current position
        auto current = odometry_->getCurrentPose();
        
        // Calculate vector to target
        float dx = target_x_inches - current.x_inches;
        float dy = target_y_inches - current.y_inches;
        float distance = std::sqrt(dx * dx + dy * dy);
        
        // Check if at target
        if (distance < tolerance_inches) {
            RCLCPP_INFO(node_->get_logger(), "driveToPoint: Reached target");
            return true;
        }
        
        // Calculate required heading and strafe/forward decomposition
        float target_heading = calculateHeading(dx, dy);
        float heading_error = normalizeAngle(target_heading - current.theta_rad);
        
        // Calculate dt for PID
        float dt = 0.01f;
        
        // PID control for distance and heading
        float distance_output = distance_pid_.calculate(distance, dt) * 0.5f;  // Scale down
        float heading_output = heading_pid_.calculate(heading_error, dt);
        
        if (debug_logging_) {
            RCLCPP_INFO(node_->get_logger(),
                "driveToPoint: dist=%.1f inches, heading_error=%.3f rad",
                distance, heading_error);
        }
        
        // TODO: Send velocity commands
        // NOTE: Need to actually call motor control methods here
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace Hardware
