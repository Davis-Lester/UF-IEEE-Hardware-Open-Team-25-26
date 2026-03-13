#include "hardware_team_robot/navigation_controller.h"
#include "hardware_team_robot/chassis_node.h"  // Include actual chassis interface
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
    : odometry_(odom), 
      node_(rclcpp_node), 
      is_valid_(false) {
    
    // REQUIRED: Both node_ and odometry_ must be non-null for controller to function
    if (!node_) {
        fprintf(stderr, "[NavigationController] FATAL: ROS node is null - controller is INVALID\n");
        return;
    }
    
    if (!odometry_) {
        RCLCPP_ERROR(node_->get_logger(), 
            "[NavigationController] FATAL: Odometry is null - controller is INVALID");
        return;
    }
    
    // Create publisher for motor commands
    motor_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "motor_cmd", rclcpp::QoS(10).transient_local().reliable());
    
    // Mark as valid only if required dependencies are present
    is_valid_ = true;
    
    // Initialize PID controllers with reasonable defaults for tank drive
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

void NavigationController::sendTankSpeeds(float left_speed, float right_speed) {
    // Publish motor command message
    if (!motor_cmd_pub_) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "[NavigationController] motor_cmd_pub_ is null! Cannot publish motor command.");
        }
        return;
    }
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = left_speed;
    msg.linear.y = right_speed;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    motor_cmd_pub_->publish(msg);
    if (debug_logging_ && node_) {
        RCLCPP_DEBUG(node_->get_logger(), 
            "[NavigationController] Published motor cmd: L=%.1f R=%.1f", 
            left_speed, right_speed);
    }
}

bool NavigationController::moveToPose(float target_x_inches, float target_y_inches,
                                      float target_theta_rad,
                                      float max_speed, float tolerance_inches,
                                      float angle_tolerance_rad) {
    // Validate controller state (REQUIRED dependencies must be present)
    if (!is_valid_) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), 
                "[NavigationController] Controller is INVALID - missing required dependencies");
        }
        return false;
    }
    
    // Validate speed parameter at API boundary
    if (max_speed <= 0.0f) {
        RCLCPP_ERROR(node_->get_logger(), 
            "[NavigationController] Invalid max_speed=%.1f (must be > 0)", max_speed);
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
    // Validate controller state (REQUIRED dependencies must be present)
    if (!is_valid_) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), 
                "[NavigationController] Controller is INVALID - missing required dependencies");
        }
        return false;
    }
    
    // Validate speed parameter at API boundary
    if (max_speed <= 0.0f) {
        RCLCPP_ERROR(node_->get_logger(), 
            "[NavigationController] Invalid max_speed=%.1f (must be > 0)", max_speed);
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
    if (!is_valid_) {
        return;
    }
    
    odometry_->reset();
    RCLCPP_INFO(node_->get_logger(), "[NavigationController] Odometry reset");
}

void NavigationController::stop() {
    // Send actual stop command to chassis
    sendTankSpeeds(0.0f, 0.0f);
    
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[NavigationController] Stop command sent");
    }
}

void NavigationController::setPIDGains(float distance_kp, float distance_ki, float distance_kd,
                                      float heading_kp, float heading_ki, float heading_kd) {
    distance_pid_.kp = distance_kp;
    distance_pid_.ki = distance_ki;
    distance_pid_.kd = distance_kd;
    
    heading_pid_.kp = heading_kp;
    heading_pid_.ki = heading_ki;
    heading_pid_.kd = heading_kd;
    
    if (node_) {
        RCLCPP_INFO(node_->get_logger(),
            "[NavigationController] PID gains updated - Distance(%.2f,%.2f,%.2f) Heading(%.2f,%.2f,%.2f)",
            distance_kp, distance_ki, distance_kd, heading_kp, heading_ki, heading_kd);
    }
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
    // is_valid_ already checked by caller (moveToPose/moveToPoint)
    
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
        
        // Send differential rotation command to chassis
        // For tank drive: left_speed = -rotation_speed, right_speed = +rotation_speed
        sendTankSpeeds(-rotation_speed, rotation_speed);
        
        // Sleep briefly
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


bool NavigationController::driveDistance(float target_distance_inches, float desired_heading_rad,
                                        float tolerance_inches, float max_speed) {
    // is_valid_ already checked by caller
    
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
        
        // Minor heading correction using named constant
        float heading_error = normalizeAngle(desired_heading_rad - current.theta_rad);
        float heading_correction = head_pid.calculate(heading_error, dt) * HEADING_CORRECTION_SCALE;
        
        if (debug_logging_) {
            RCLCPP_INFO(node_->get_logger(),
                "driveDistance: traveled=%.1f/%.1f inches, forward=%.1f, heading_corr=%.1f",
                distance_traveled, target_distance_inches, forward_speed, heading_correction);
        }
        
        // Send tank drive commands with heading correction
        float left_speed = forward_speed - heading_correction;
        float right_speed = forward_speed + heading_correction;
        sendTankSpeeds(left_speed, right_speed);
        
        // Sleep briefly
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool NavigationController::driveToPoint(float target_x_inches, float target_y_inches,
                                       float tolerance_inches, float max_speed) {
    // is_valid_ already checked by caller
    
    auto deadline = std::chrono::steady_clock::now() + 
                    std::chrono::duration<float>(MOVEMENT_TIMEOUT_SEC);
    
    // Create local PID copies and reset state
    PIDController local_distance_pid = distance_pid_;
    PIDController local_heading_pid = heading_pid_;
    local_distance_pid.reset();
    local_heading_pid.reset();
    
    // Track phase to detect transitions (turning vs driving)
    enum Phase { TURNING, DRIVING };
    Phase current_phase = TURNING;  // Start assuming we need to turn
    
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
        
        // Determine phase based on heading error
        Phase new_phase = (std::abs(heading_error) > 0.15f) ? TURNING : DRIVING;
        
        // Detect phase transition and reset heading PID to prevent integral windup carry-over
        if (new_phase != current_phase) {
            local_heading_pid.reset();
            current_phase = new_phase;
            if (debug_logging_) {
                RCLCPP_DEBUG(node_->get_logger(), 
                    "driveToPoint: Phase transition to %s", 
                    (current_phase == TURNING) ? "TURNING" : "DRIVING");
            }
        }
        
        if (current_phase == TURNING) {
            // In-place turn logic
            float dt = 0.01f;
            float rotation_speed = local_heading_pid.calculate(heading_error, dt);
            
            // Apply max_speed limit
            if (rotation_speed > max_speed) rotation_speed = max_speed;
            if (rotation_speed < -max_speed) rotation_speed = -max_speed;
            
            if (debug_logging_) {
                RCLCPP_INFO(node_->get_logger(),
                    "driveToPoint: turning, heading_error=%.3f rad", heading_error);
            }
            
            // Send rotation command
            sendTankSpeeds(-rotation_speed, rotation_speed);
        } else {
            // Drive forward with minor heading correction
            float dt = 0.01f;
            float forward_speed = local_distance_pid.calculate(distance, dt) * APPROACH_SPEED_SCALE;
            float heading_correction = local_heading_pid.calculate(heading_error, dt) * APPROACH_HEADING_SCALE;
            
            // Apply max_speed limit
            if (forward_speed > max_speed) forward_speed = max_speed;
            if (forward_speed < -max_speed) forward_speed = -max_speed;
            
            if (debug_logging_) {
                RCLCPP_INFO(node_->get_logger(),
                    "driveToPoint: dist=%.1f inches, heading_error=%.3f rad",
                    distance, heading_error);
            }
            
            // Send velocity commands
            float left_speed = forward_speed - heading_correction;
            float right_speed = forward_speed + heading_correction;
            sendTankSpeeds(left_speed, right_speed);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace Hardware
