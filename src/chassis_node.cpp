// Name: Davis Lester
// Date: 2/5/2026
// Description: Mecanum Drive PID Chassis Node (4 Motor / 4 Encoder)

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

Hardware::MecanumOdometry::Pose ChassisNode::getOdometryPose() const {
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

float ChassisNode::calculate_y_from_wall(const Hardware::MecanumOdometry::Pose& current_pose, float sensor_distance, float offset_x, float offset_y, float known_wall_y) {
    float theta = current_pose.theta_rad;
    // Apply the 2D transformation matrix to find the robot's true Y center
    return known_wall_y - (offset_x * std::sin(theta)) - ((offset_y + sensor_distance) * std::cos(theta));
}

void ChassisNode::odometry_update_loop() {
    rclcpp::Rate loop_rate(100);  // 100 Hz update rate
    
    // Track heading by integrating gyro
    float heading_rad = 0.0f;
    auto last_update = std::chrono::high_resolution_clock::now();
    int ultrasonic_counter = 0;
    
    while (rclcpp::ok()) {
        if (encoder_driver_ && odometry_) {
            // Get current encoder counts from hardware abstraction
            int32_t fl = encoder_driver_->getFrontLeftTicks();
            int32_t fr = encoder_driver_->getFrontRightTicks();
            int32_t rl = encoder_driver_->getRearLeftTicks();
            int32_t rr = encoder_driver_->getRearRightTicks();
            
            // Get heading from IMU via gyro integration
            float gx, gy, gz;
            if (imu_.readGyro(&gx, &gy, &gz) == 0) {
                auto now = std::chrono::high_resolution_clock::now();
                float dt = std::chrono::duration<float>(now - last_update).count();
                
                float gz_rad_per_sec = gz * (M_PI / 180.0f);
                heading_rad += gz_rad_per_sec * dt;
                
                // Normalize heading to [-π, π]
                while (heading_rad > M_PI) heading_rad -= 2.0f * M_PI;
                while (heading_rad < -M_PI) heading_rad += 2.0f * M_PI;
                
                last_update = now;
            }
            
            // Update odometry with encoder data and IMU heading
            odometry_->update(fl, fr, rl, rr, heading_rad);

            // --- Ultrasonic Correction Block (Runs at 10Hz) ---
            ultrasonic_counter++;
            if (ultrasonic_counter >= 10 && ultrasonic_driver_) { 
                ultrasonic_counter = 0;

                float left_dist = ultrasonic_driver_->getLeftDistance();
                float right_dist = ultrasonic_driver_->getRightDistance();
                
                bool left_valid = (left_dist > 0.0f && left_dist < 24.0f);
                bool right_valid = (right_dist > 0.0f && right_dist < 24.0f);

                if (left_valid || right_valid) {
                    Hardware::MecanumOdometry::Pose current_pose = odometry_->getCurrentPose();
                    float new_y = current_pose.y_inches;
                    bool update_needed = false;

                    // Calculate proposed Y coordinates
                    if (left_valid && right_valid) {
                        float y_left = calculate_y_from_wall(current_pose, left_dist, US_LEFT_OFFSET_X, US_LEFT_OFFSET_Y, KNOWN_LEFT_WALL_Y);
                        float y_right = calculate_y_from_wall(current_pose, -right_dist, US_RIGHT_OFFSET_X, US_RIGHT_OFFSET_Y, KNOWN_RIGHT_WALL_Y);
                        
                        // FUSION: Average the two valid readings
                        new_y = (y_left + y_right) / 2.0f;
                        update_needed = true;
                    } 
                    else if (left_valid) {
                        new_y = calculate_y_from_wall(current_pose, left_dist, US_LEFT_OFFSET_X, US_LEFT_OFFSET_Y, KNOWN_LEFT_WALL_Y);
                        update_needed = true;
                    } 
                    else if (right_valid) {
                        new_y = calculate_y_from_wall(current_pose, -right_dist, US_RIGHT_OFFSET_X, US_RIGHT_OFFSET_Y, KNOWN_RIGHT_WALL_Y);
                        update_needed = true;
                    }

                    // Apply the update once
                    if (update_needed) {
                        float error = std::abs(current_pose.y_inches - new_y);
                        if (error < 12.0f) { 
                            odometry_->setPose(current_pose.x_inches, new_y, current_pose.theta_rad);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Odom Correction Ignored (Jump too large). Est Y: %.2f | Calc Y: %.2f",
                                        current_pose.y_inches, new_y);
                        }
                    }
                }
                // Ping the sensors for the next cycle
                ultrasonic_driver_->trigger();
            }
        }
        loop_rate.sleep();
    }
}

ChassisNode::ChassisNode() : Node("chassis_node"), imu_(1) {
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(this->get_logger(), "Pigpio Init Failed!");
    } else {
        setup_motor_pins();
        setup_encoders();
        
        // --- Setup Ultrasonic System ---
        Hardware::UltrasonicDriver::Config left_us_cfg{PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO};
        Hardware::UltrasonicDriver::Config right_us_cfg{PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO};
        
        ultrasonic_driver_ = std::make_shared<Hardware::UltrasonicDriver>(left_us_cfg, right_us_cfg);
        if (!ultrasonic_driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Ultrasonic Init Failed!");
        }
    }

    // Default chassis dimensions for MecanumOdometry
    odometry_ = std::make_shared<Hardware::MecanumOdometry>(177.8f, 237.5f, 60.0f);
    encoder_driver_ = std::make_shared<Hardware::EncoderDriver>();
    encoder_driver_->initialize();

    if (imu_.initialize() == 0) {
        RCLCPP_INFO(this->get_logger(), "IMU Calibrating...");
        imu_.calibrate(500); 
    }

    // Launch odometry update loop in a background thread
    std::thread(&ChassisNode::odometry_update_loop, this).detach();

    this->action_server_ = rclcpp_action::create_server<Drive>(
        this, "drive_command",
        std::bind(&ChassisNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ChassisNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&ChassisNode::handle_accepted, this, std::placeholders::_1));
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
    bool is_strafing = (goal->mode == "STRAFE");

    // Tuning
    if (is_turning) kp = 2.0;
    else if (is_strafing) kp = 1.0; 
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

        // --- SENSOR READING & ERROR CALC ---
        if (is_turning) {
            float gx, gy, gz;
            imu_.readGyro(&gx, &gy, &gz);
            current_val += (gz * dt);
            error = goal->target_value - current_val;
        } 
        else if (is_strafing) {
            double fl = std::abs(fl_ticks_.load());
            double fr = std::abs(fr_ticks_.load());
            double rl = std::abs(rl_ticks_.load());
            double rr = std::abs(rr_ticks_.load());
            
            double avg_ticks = (fl + fr + rl + rr) / 4.0;
            if (goal->target_value < 0) avg_ticks = -avg_ticks;
            
            current_val = avg_ticks;
            error = goal->target_value - current_val;
        } 
        else {
            double sum = std::abs(fl_ticks_) + std::abs(fr_ticks_) + std::abs(rl_ticks_) + std::abs(rr_ticks_);
            double avg_ticks = sum / 4.0;
            if (goal->target_value < 0) avg_ticks = -avg_ticks;
            
            current_val = avg_ticks;
            error = goal->target_value - current_val;
        }

        // --- PID OUTPUT ---
        double output = error * kp;
        
        // Clamp Speed
        if (output > goal->max_speed) output = goal->max_speed;
        if (output < -goal->max_speed) output = -goal->max_speed;

        // --- MIXING & MOTOR OUTPUT ---
        if (is_turning) {
            set_mecanum_power(-output, output, -output, output);
        } 
        else if (is_strafing) {
            set_mecanum_power(output, -output, -output, output);
        } 
        else {
            set_mecanum_power(output, output, output, output);
        }

        // --- SETTLE ---
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

void ChassisNode::set_mecanum_power(double fl, double fr, double rl, double rr) {
    auto write_motor = [](int pwm_pin, int in1, int in2, double speed) {
        int pwm_val = std::clamp((int)speed, -255, 255);
        if (pwm_val > 0) {
            gpioWrite(in1, 1); gpioWrite(in2, 0);
        } else if (pwm_val < 0) {
            gpioWrite(in1, 0); gpioWrite(in2, 1);
        } else {
            gpioWrite(in1, 0); gpioWrite(in2, 0);
        }
        gpioPWM(pwm_pin, std::abs(pwm_val));
    };

    write_motor(PIN_FL_PWM, PIN_FL_IN1, PIN_FL_IN2, fl);
    write_motor(PIN_FR_PWM, PIN_FR_IN1, PIN_FR_IN2, fr);
    write_motor(PIN_RL_PWM, PIN_RL_IN1, PIN_RL_IN2, rl);
    write_motor(PIN_RR_PWM, PIN_RR_IN1, PIN_RR_IN2, rr);
}

void ChassisNode::setup_motor_pins() {
    int outputs[] = {
        PIN_FL_PWM, PIN_FL_IN1, PIN_FL_IN2,
        PIN_FR_PWM, PIN_FR_IN1, PIN_FR_IN2,
        PIN_RL_PWM, PIN_RL_IN1, PIN_RL_IN2,
        PIN_RR_PWM, PIN_RR_IN1, PIN_RR_IN2
    };
    for (int p : outputs) gpioSetMode(p, PI_OUTPUT);
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
    set_mecanum_power(0, 0, 0, 0);
}

// Action callbacks
rclcpp_action::GoalResponse ChassisNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Drive::Goal> goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ChassisNode::handle_cancel(const std::shared_ptr<GoalHandleDrive> goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ChassisNode::handle_accepted(const std::shared_ptr<GoalHandleDrive> goal_handle) {
    std::thread{std::bind(&ChassisNode::execute, this, goal_handle)}.detach();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisNode>());
    rclcpp::shutdown();
    return 0;
}