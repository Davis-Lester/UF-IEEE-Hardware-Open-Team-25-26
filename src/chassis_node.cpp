// Name: Davis Lester
// Date: 2/5/2026
// Description: Tank Drive PID Chassis Node (4 Motor / 4 Encoder)

#include "hardware_team_robot/chassis_node.h"
#include <cmath>
#include <algorithm>
#include <thread>
#include <pigpio.h>

// ISR Wrapper
static void encoder_isr_wrapper(int gpio, int level, uint32_t tick, void *user) {
    ChassisNode *node = (ChassisNode*)user;
    node->handle_encoder_tick(gpio, level);
}

Hardware::TankOdometry::Pose ChassisNode::getOdometryPose() const {
    if (odometry_) {
        return odometry_->getCurrentPose();
    }
    return {0.0f, 0.0f, 0.0f};
}

void ChassisNode::resetOdometry() {
    if (odometry_) {
        odometry_->reset();
        RCLCPP_INFO(this->get_logger(), "Odometry reset to origin");
    }
}

ChassisNode::~ChassisNode() {  
    if (odometry_thread_.joinable()) odometry_thread_.join();  
    if (execute_thread_.joinable()) execute_thread_.join();  
} 

void ChassisNode::odometry_update_loop() {
    rclcpp::Rate loop_rate(100);  // 100 Hz update rate
    
    // Track heading by integrating gyro
    float heading_rad = 0.0f;
    auto last_update = std::chrono::high_resolution_clock::now();
    
    while (rclcpp::ok()) {
        if (encoder_driver_ && odometry_) {
            // Get current encoder counts from hardware abstraction
            int32_t fl = encoder_driver_->getFrontLeftTicks();
            int32_t fr = encoder_driver_->getFrontRightTicks();
            int32_t rl = encoder_driver_->getRearLeftTicks();
            int32_t rr = encoder_driver_->getRearRightTicks();
            
            // Get heading from IMU via gyro integration
            // Sample time before IMU read to prevent stale dt on failure
            auto now = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(now - last_update).count();
            float gx, gy, gz;
            if (imu_.readGyro(&gx, &gy, &gz) == 0) {
                float gz_rad_per_sec = gz * (M_PI / 180.0f);
                heading_rad += gz_rad_per_sec * dt;
                
                // Normalize heading to [-π, π]
                while (heading_rad > M_PI) heading_rad -= 2.0f * M_PI;
                while (heading_rad < -M_PI) heading_rad += 2.0f * M_PI;
            }
            // Always update timestamp to prevent dt accumulation on failure
            last_update = now;
            
            // Update odometry with encoder data and IMU heading
            odometry_->update(fl, fr, rl, rr, heading_rad);
        } 
        
        // REMOVED: VEML7700 Start Light Detection
        // Start light detection now handled by camera_node
        
        loop_rate.sleep();
    }
}

ChassisNode::ChassisNode() : Node("chassis_node"), imu_(1) {
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(this->get_logger(), "Pigpio Init Failed!");
    } else {
        // setup_motor_pins(); Replaced with this 
        motor_driver_ = std::make_shared<Hardware::PCA9685Driver>(1, 0x40);
        if (!motor_driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "PCA9685 Init Failed: %s", 
                        motor_driver_->getLastError().c_str());
            motor_driver_.reset();  // To prevent using failed driver
        } else {
            RCLCPP_INFO(this->get_logger(), "PCA9685 Motor Driver Ready @ 1000 Hz");
            motor_ready_ = true; 
        }
    }

    // Default chassis dimensions for Tank Odmetry 
    // (wheel diameter, horizontal distance, vertical distance)
    odometry_ = std::make_shared<Hardware::TankOdometry>(80.0f, 237.49f, 177.8f);
    encoder_driver_ = std::make_shared<Hardware::EncoderDriver>();
    encoder_driver_->initialize();
    
    //Init IMU
    if (imu_.initialize() == 0) {
        RCLCPP_INFO(this->get_logger(), "IMU Calibrating...");
        imu_.calibrate(500); 
    }

    // REMOVED: VEML7700 Start Light Sensor initialization
    // Start light detection now handled by camera_node
    
    // Launch odometry update loop in a background thread
    odometry_thread_ = std::thread(&ChassisNode::odometry_update_loop, this);

    this->action_server_ = rclcpp_action::create_server<Drive>(
        this, "drive_command",
        std::bind(&ChassisNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ChassisNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&ChassisNode::handle_accepted, this, std::placeholders::_1));

    // Subscribe to motor commands from NavigationController
    motor_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "motor_cmd", 10,
        std::bind(&ChassisNode::motor_cmd_callback, this, std::placeholders::_1));
}

void ChassisNode::execute(const std::shared_ptr<GoalHandleDrive> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Drive::Result>();
    auto feedback = std::make_shared<Drive::Feedback>();

    // Reset Encoders
    fl_ticks_ = 0; fr_ticks_ = 0;
    rl_ticks_ = 0; rr_ticks_ = 0;

    double current_val = 0.0;
    double error = 0.0;
    double kp = 0.0;
    
    // Determine Control Mode
    bool is_turning = (goal->mode == "TURN");

    // Tuning
    if (is_turning) kp = 2.0;
    else kp = 0.6; 

    rclcpp::Rate loop_rate(50);
    double dt = 0.02;
    int settled_count = 0;

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            stop_motors();
            goal_handle->canceled(result);
            return;
        }
        
        if (is_turning) {
            float gx, gy, gz;
            // Check IMU read success before using gyro data
            if (imu_.readGyro(&gx, &gy, &gz) == 0) {
                current_val += (gz * dt);
                error = goal->target_value - current_val;
            } else {
                // IMU read failed - skip integration, keep previous error
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                    "IMU read failed during turn");
            }
        } 
        else {
            // TANK DRIVE: Average all 4 wheels for forward/backward
            double sum = std::abs(fl_ticks_) + std::abs(fr_ticks_) + std::abs(rl_ticks_) + std::abs(rr_ticks_);
            double avg_ticks = sum / 4.0;
            if (goal->target_value < 0) avg_ticks = -avg_ticks;
            
            current_val = avg_ticks;
            error = goal->target_value - current_val;
        }
        
        double output = error * kp;
        if (output > goal->max_speed) output = goal->max_speed;
        if (output < -goal->max_speed) output = -goal->max_speed;
        
        // ========== Tank drive motor mixing (left/right only) ==========
        if (is_turning) {
            // Turn in place: left backward, right forward
            set_tank_power(-output, output);
        } 
        else {
            // Drive straight: both sides same speed
            set_tank_power(output, output);
        }
        // ========================================================================
        
        double tolerance = is_turning ? 2.0 : 25.0;
        if (std::abs(error) < tolerance) settled_count++;
        else settled_count = 0;
        
        if (settled_count > 10) break;
        
        feedback->current_error = error;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }
    
    stop_motors();
    result->success = true;
    goal_handle->succeed(result);
}  

// ========== Simplified tank drive control (4 wheels: 2 per side) ==========
void ChassisNode::set_tank_power(double left, double right) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    
    if (!motor_driver_ || !motor_ready_) return;  // <-- ADD motor_ready_ check
    
    // Convert -255...255 to -100...100 safely
    double left_d = (left * 100.0) / 255.0;
    double right_d = (right * 100.0) / 255.0;
    
    int8_t left_pct = static_cast<int8_t>(std::clamp(left_d, -100.0, 100.0));
    int8_t right_pct = static_cast<int8_t>(std::clamp(right_d, -100.0, 100.0));
    
    // Left side: M1 (FL) + M3 (RL)
    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_1, left_pct);
    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_3, left_pct);
    
    // Right side: M2 (FR) + M4 (RR)
    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_2, right_pct);
    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_4, right_pct);
}

void ChassisNode::setup_encoders() {
    int inputs[] = {
        PIN_FL_ENC_A, PIN_FL_ENC_B, PIN_FR_ENC_A, PIN_FR_ENC_B,
        PIN_RL_ENC_A, PIN_RL_ENC_B, PIN_RR_ENC_A, PIN_RR_ENC_B
    };
    for (int p : inputs) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_UP);
    }

    gpioSetISRFuncEx(PIN_FL_ENC_A, EITHER_EDGE, 0, encoder_isr_wrapper, (void*)this);
    gpioSetISRFuncEx(PIN_FR_ENC_A, EITHER_EDGE, 0, encoder_isr_wrapper, (void*)this);
    gpioSetISRFuncEx(PIN_RL_ENC_A, EITHER_EDGE, 0, encoder_isr_wrapper, (void*)this);
    gpioSetISRFuncEx(PIN_RR_ENC_A, EITHER_EDGE, 0, encoder_isr_wrapper, (void*)this);
}

void ChassisNode::handle_encoder_tick(int gpio, int level) {
    if (gpio == PIN_FL_ENC_A) {
        if (level == gpioRead(PIN_FL_ENC_B)) fl_ticks_++; else fl_ticks_--;
    } 
    else if (gpio == PIN_FR_ENC_A) {
        if (level == gpioRead(PIN_FR_ENC_B)) fr_ticks_--; else fr_ticks_++;
    }
    else if (gpio == PIN_RL_ENC_A) {
        if (level == gpioRead(PIN_RL_ENC_B)) rl_ticks_++; else rl_ticks_--;
    }
    else if (gpio == PIN_RR_ENC_A) {
        if (level == gpioRead(PIN_RR_ENC_B)) rr_ticks_--; else rr_ticks_++;
    }
}

void ChassisNode::stop_motors() {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (motor_driver_) {
        motor_driver_->stopAll();
    }
}

void ChassisNode::motor_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract left and right motor speeds from Twist message
    // Convention: linear.x = left speed, linear.y = right speed
    float left_speed = static_cast<float>(msg->linear.x);
    float right_speed = static_cast<float>(msg->linear.y);
    
    // Apply the motor commands (mutex protection is handled in set_tank_power)
    set_tank_power(static_cast<double>(left_speed), static_cast<double>(right_speed));
}

// Action callbacks
rclcpp_action::GoalResponse ChassisNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const Drive::Goal> goal) {
    
    if (!motor_ready_) {
        RCLCPP_ERROR(this->get_logger(), "Motor driver not initialized - rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (execute_thread_.joinable()) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected - previous action still executing");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ChassisNode::handle_cancel(const std::shared_ptr<GoalHandleDrive> goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ChassisNode::handle_accepted(const std::shared_ptr<GoalHandleDrive> goal_handle) {
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    execute_thread_ = std::thread(std::bind(&ChassisNode::execute, this, goal_handle));
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisNode>());
    rclcpp::shutdown();
    return 0;
}