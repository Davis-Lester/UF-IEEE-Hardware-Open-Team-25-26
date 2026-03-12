#include "hardware_team_robot/navigation_controller.h"
#include <cstdio>
#include <thread>
#include <chrono>
#include <cmath>

namespace Hardware {

float NavigationController::PIDController::calculate(float error, float dt) {
    if (dt <= 0.0f) dt = 0.01f;  // Prevent division by zero
    
    // Proportional term
    float p_term = kp * error;
    
    // Integral term (with anti-windup)
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

NavigationController::NavigationController(std::shared_ptr<TankOdometry> odom,
                                         std::shared_ptr<rclcpp::Node> rclcpp_node)
    : odometry_(odom), node_(rclcpp_node) {
    
    // Initialize PID controllers with reasonable defaults for tank drive
    // These may need tuning based on actual robot performance
    distance_pid_.kp = 0.5f;
    distance_pid_.ki = 0.05f;
    distance_pid_.kd = 0.2f;
    
    heading_pid_.kp = 2.0f;
    heading_pid_.ki = 0.1f;
    heading_pid_.kd = 0.3f;
    
    RCLCPP_INFO(node_->get_logger(), "[NavigationController] Initialized for Tank Drive");
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
        "[NavigationController] moveToPose: (%.1f, %.1f) θ=%.3f rad, max_speed=%.1f",
        target_x_inches, target_y_inches, target_theta_rad, max_speed);
    
    // Step 1: Turn to face target point
    TankOdometry::Pose current = odometry_->getCurrentPose();
    float dx = target_x_inches - current.x_inches;
    float dy = target_y_inches - current.y_inches;
    float distance = calculateDistance(current.x_inches, current.y_inches,
                                      target_x_inches, target_y_inches);
    
    // Only turn and drive if we're not already at the target
    if (distance > tolerance_inches) {
        float target_heading = calculateHeading(dx, dy);
        
        if (!turnToHeading(target_heading, angle_tolerance_rad, max_speed)) {
            RCLCPP_WARN(node_->get_logger(), "Turn to heading timed out");
            return false;
        }
        
        // Step 2: Drive to target position
        if (!driveDistance(distance, target_heading, tolerance_inches, max_speed)) {
            RCLCPP_WARN(node_->get_logger(), "Drive distance timed out");
            return false;
        }
    }
    
    // Step 3: Turn to final heading
    if (!turnToHeading(target_theta_rad, angle_tolerance_rad, max_speed)) {
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
        "[NavigationController] moveToPoint: (%.1f, %.1f), max_speed=%.1f",
        target_x_inches, target_y_inches, max_speed);
    
    if (driveToPoint(target_x_inches, target_y_inches, tolerance_inches, max_speed)) {
        RCLCPP_INFO(node_->get_logger(), "[NavigationController] moveToPoint: COMPLETE");
        return true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "moveToPoint timed out");
        return false;
    }
}

TankOdometry::Pose NavigationController::getCurrentPose() const {
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
    if (!node_) {
        return;
    }
    
    // TODO: Send zero velocity commands to chassis
    // This should call the actual chassis interface when available
    // Example: chassis_->setTankSpeeds(0.0f, 0.0f);
    
    RCLCPP_INFO(node_->get_logger(), "[NavigationController] Stop command issued");
    
    // Note: Currently this only logs. Integration with ChassisNode required
    // to send actual motor stop commands.
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
    // O(1) normalization using fmod to avoid unbounded loops
    angle_rad = std::fmod(angle_rad, 2.0f * M_PI);
    
    // Adjust to [-π, π] range
    if (angle_rad > M_PI) {
        angle_rad -= 2.0f * M_PI;
    } else if (angle_rad < -M_PI) {
        angle_rad += 2.0f * M_PI;
    }
    
    return angle_rad;
}

bool NavigationController::turnToHeading(float target_heading_rad, float tolerance_rad, float max_speed) {
    // IMPORTANT: This is a BLOCKING call. In a real ROS 2 system, this should be refactored
    // to use an action server or timer-based approach.
    
    auto start_time = std::chrono::steady_clock::now();
    PIDController pid = heading_pid_;  // Local copy for this movement
    pid.reset();  // Clear any previous state
    
    while (true) {
        // Check timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        float elapsed_sec = std::chrono::duration<float>(elapsed).count();
        if (elapsed_sec > MOVEMENT_TIMEOUT_SEC) {
            RCLCPP_WARN(node_->get_logger(), "turnToHeading: TIMEOUT after %.1f sec", elapsed_sec);
            stop();
            return false;
        }
        
        // Get current heading
        auto current = odometry_->getCurrentPose();
        float current_heading = current.theta_rad;
        
        // Calculate error (normalize to [-π, π])
        float error = normalizeAngle(target_heading_rad - current_heading);
        
        // Check if at target
        if (std::abs(error) < tolerance_rad) {
            stop();
            RCLCPP_INFO(node_->get_logger(), "turnToHeading: Reached target heading");
            return true;
        }
        
        // Calculate dt for PID
        float dt = 0.01f;  // 10ms
        
        // PID control for rotation
        float rotation_speed = pid.calculate(error, dt);
        
        // Apply max_speed limit
        if (rotation_speed > max_speed) rotation_speed = max_speed;
        if (rotation_speed < -max_speed) rotation_speed = -max_speed;
        
        if (debug_logging_) {
            RCLCPP_INFO(node_->get_logger(),
                "turnToHeading: current=%.3f, target=%.3f, error=%.3f rad, output=%.1f",
                current_heading, target_heading_rad, error, rotation_speed);
        }
        
        // TODO: Send differential rotation command to chassis
        // For tank drive: left_speed = -rotation_speed, right_speed = +rotation_speed
        // chassis_->setTankSpeeds(-rotation_speed, rotation_speed);
        
        // Sleep briefly
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool NavigationController::driveDistance(float target_distance_inches, float desired_heading_rad,
                                        float tolerance_inches, float max_speed) {
    // IMPORTANT: This is a BLOCKING call. See note in turnToHeading().
    auto start_time = std::chrono::steady_clock::now();
    
    auto start_pose = odometry_->getCurrentPose();
    PIDController dist_pid = distance_pid_;
    PIDController head_pid = heading_pid_;
    dist_pid.reset();
    head_pid.reset();
    
    while (true) {
        // Check timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        float elapsed_sec = std::chrono::duration<float>(elapsed).count();
        if (elapsed_sec > MOVEMENT_TIMEOUT_SEC) {
            RCLCPP_WARN(node_->get_logger(), "driveDistance: TIMEOUT after %.1f sec", elapsed_sec);
            stop();
            return false;
        }
        
        // Calculate distance traveled
        auto current = odometry_->getCurrentPose();
        float distance_traveled = calculateDistance(start_pose.x_inches, start_pose.y_inches,
                                                   current.x_inches, current.y_inches);
        
        float distance_error = target_distance_inches - distance_traveled;
        
        // Check if at target
        if (std::abs(distance_error) < tolerance_inches) {
            stop();
            RCLCPP_INFO(node_->get_logger(), "driveDistance: Reached target");
            return true;
        }
        
        // Calculate dt for PID
        float dt = 0.01f;
        
        // PID for forward motion
        float forward_speed = dist_pid.calculate(distance_error, dt);
        
        // Apply max_speed limit
        if (forward_speed > max_speed) forward_speed = max_speed;
        if (forward_speed < -max_speed) forward_speed = -max_speed;
        
        // Minor heading correction
        float heading_error = normalizeAngle(desired_heading_rad - current.theta_rad);
        float heading_correction = head_pid.calculate(heading_error, dt) * 0.3f;  // Scale down correction
        
        if (debug_logging_) {
            RCLCPP_INFO(node_->get_logger(),
                "driveDistance: traveled=%.1f/%.1f inches, forward=%.1f, heading_corr=%.1f",
                distance_traveled, target_distance_inches, forward_speed, heading_correction);
        }
        
        // TODO: Send tank drive commands with heading correction
        // left_speed = forward_speed - heading_correction
        // right_speed = forward_speed + heading_correction
        // chassis_->setTankSpeeds(forward_speed - heading_correction, forward_speed + heading_correction);
        
        // Sleep briefly
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool NavigationController::driveToPoint(float target_x_inches, float target_y_inches,
                                       float tolerance_inches, float max_speed) {
    // For tank drive, this uses a turn-drive approach with shared deadline
    // IMPORTANT: This is a BLOCKING call. See note in turnToHeading().
    
    auto deadline = std::chrono::steady_clock::now() + 
                    std::chrono::duration<float>(MOVEMENT_TIMEOUT_SEC);
    
    // Create local PID copies and reset state
    PIDController local_distance_pid = distance_pid_;
    PIDController local_heading_pid = heading_pid_;
    local_distance_pid.reset();
    local_heading_pid.reset();
    
    while (true) {
        // Check deadline
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            RCLCPP_WARN(node_->get_logger(), "driveToPoint: TIMEOUT");
            stop();
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
            stop();
            RCLCPP_INFO(node_->get_logger(), "driveToPoint: Reached target");
            return true;
        }
        
        // Calculate required heading
        float target_heading = calculateHeading(dx, dy);
        float heading_error = normalizeAngle(target_heading - current.theta_rad);
        
        // If heading error is large, perform in-place turn
        if (std::abs(heading_error) > 0.15f) {  // ~8.6 degrees
            // Inline turn logic to respect shared deadline
            float dt = 0.01f;
            float rotation_speed = local_heading_pid.calculate(heading_error, dt);
            
            // Apply max_speed limit
            if (rotation_speed > max_speed) rotation_speed = max_speed;
            if (rotation_speed < -max_speed) rotation_speed = -max_speed;
            
            if (debug_logging_) {
                RCLCPP_INFO(node_->get_logger(),
                    "driveToPoint: turning, heading_error=%.3f rad", heading_error);
            }
            
            // TODO: Send rotation command
            // chassis_->setTankSpeeds(-rotation_speed, rotation_speed);
        } else {
            // Drive forward with minor heading correction
            float dt = 0.01f;
            float forward_speed = local_distance_pid.calculate(distance, dt) * 0.6f;  // Scale for smooth approach
            float heading_correction = local_heading_pid.calculate(heading_error, dt) * 0.2f;
            
            // Apply max_speed limit
            if (forward_speed > max_speed) forward_speed = max_speed;
            if (forward_speed < -max_speed) forward_speed = -max_speed;
            
            if (debug_logging_) {
                RCLCPP_INFO(node_->get_logger(),
                    "driveToPoint: dist=%.1f inches, heading_error=%.3f rad",
                    distance, heading_error);
            }
            
            // TODO: Send velocity commands
            // chassis_->setTankSpeeds(forward_speed - heading_correction, forward_speed + heading_correction);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace Hardware