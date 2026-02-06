// Name: Davis Lester
// Date: 2/5/2026
// Description: Mecanum Drive PID Chassis Node (4 Motor / 4 Encoder)

#include "../include/Hardware_Team_Robot/chassis_node.h"
#include <cmath>
#include <algorithm>
#include <thread>
#include <pigpio.h>

// ISR Wrapper
static void encoder_isr_wrapper(int gpio, int level, uint32_t tick, void *user) {
    ChassisNode *node = (ChassisNode*)user;
    node->handle_encoder_tick(gpio, level);
}

// Updated GPIO Pins for 4-Motor Setup (Avoids conflicting pins)
// Front Left
#define PIN_FL_PWM 12
#define PIN_FL_IN1 5
#define PIN_FL_IN2 6
// Front Right
#define PIN_FR_PWM 13
#define PIN_FR_IN1 23
#define PIN_FR_IN2 24
// Rear Left
#define PIN_RL_PWM 18
#define PIN_RL_IN1 25 // 25
#define PIN_RL_IN2 8  // 8
// Rear Right
#define PIN_RR_PWM 19
#define PIN_RR_IN1 16 // 16
#define PIN_RR_IN2 20 // 20

// Encoders (A/B)
#define PIN_FL_ENC_A 17
#define PIN_FL_ENC_B 27
#define PIN_FR_ENC_A 22
#define PIN_FR_ENC_B 4  // Changed from 10 (SPI)
#define PIN_RL_ENC_A 26
#define PIN_RL_ENC_B 21
#define PIN_RR_ENC_A 9  // Check if SPI is disabled
#define PIN_RR_ENC_B 11

ChassisNode::ChassisNode() : Node("chassis_node"), imu_(1) {
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(this->get_logger(), "Pigpio Init Failed!");
    } else {
        setup_motor_pins();
        setup_encoders();
    }

    if (imu_.initialize() == 0) {
        RCLCPP_INFO(this->get_logger(), "IMU Calibrating...");
        imu_.calibrate(500); 
    }

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
    else if (is_strafing) kp = 1.0; // Needs higher power to slide wheels sideways
    else kp = 0.6; // Driving straight

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
            // GYRO TURN
            float gx, gy, gz;
            imu_.readGyro(&gx, &gy, &gz);
            current_val += (gz * dt);
            error = goal->target_angle - current_val;
        } 
        else if (is_strafing) {
            // MECANUM STRAFE KINEMATICS
            // Displacement = (FL - FR - RL + RR) / 4
            // Strafe Right: FL(+), FR(-), RL(-), RR(+)
            double fl = std::abs(fl_ticks_.load());
            double fr = std::abs(fr_ticks_.load());
            double rl = std::abs(rl_ticks_.load());
            double rr = std::abs(rr_ticks_.load());
            
            // Average the 4 wheels to get "Strafe Ticks"
            double avg_ticks = (fl + fr + rl + rr) / 4.0;
            
            // Direction handling
            if (goal->target_ticks < 0) avg_ticks = -avg_ticks;
            
            current_val = avg_ticks;
            error = goal->target_ticks - current_val;
        } 
        else {
            // STRAIGHT DRIVE
            // Displacement = Average of all 4
            double sum = std::abs(fl_ticks_) + std::abs(fr_ticks_) + std::abs(rl_ticks_) + std::abs(rr_ticks_);
            double avg_ticks = sum / 4.0;
            if (goal->target_ticks < 0) avg_ticks = -avg_ticks;
            
            current_val = avg_ticks;
            error = goal->target_ticks - current_val;
        }

        // --- PID OUTPUT ---
        double output = error * kp;
        
        // Clamp Speed
        if (output > goal->max_speed) output = goal->max_speed;
        if (output < -goal->max_speed) output = -goal->max_speed;

        // --- MIXING & MOTOR OUTPUT ---
        if (is_turning) {
            // Turn Left: Left(-), Right(+)
            // Turn Right: Left(+), Right(-)
            // Note: 'output' sign depends on error direction
            // If Target=90, Error=90, Output=High. We want Turn Right.
            // Setup: FL(-), RL(-)  FR(+), RR(+)? No, standard tank turn.
            set_mecanum_power(-output, output, -output, output);
        } 
        else if (is_strafing) {
            // Strafe Right (Positive Output): FL(+), FR(-), RL(-), RR(+)
            // Strafe Left (Negative Output): FL(-), FR(+), RL(+), RR(-)
            set_mecanum_power(output, -output, -output, output);
        } 
        else {
            // Forward: All Positive
            // Optional: Add P-correction for heading using Gyro here
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
    // Helper lambda to write to a specific motor channel
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
    // Outputs
    int outputs[] = {
        PIN_FL_PWM, PIN_FL_IN1, PIN_FL_IN2,
        PIN_FR_PWM, PIN_FR_IN1, PIN_FR_IN2,
        PIN_RL_PWM, PIN_RL_IN1, PIN_RL_IN2,
        PIN_RR_PWM, PIN_RR_IN1, PIN_RR_IN2
    };
    for (int p : outputs) gpioSetMode(p, PI_OUTPUT);
}

void ChassisNode::setup_encoders() {
    // Inputs with Pullups
    int inputs[] = {
        PIN_FL_ENC_A, PIN_FL_ENC_B, PIN_FR_ENC_A, PIN_FR_ENC_B,
        PIN_RL_ENC_A, PIN_RL_ENC_B, PIN_RR_ENC_A, PIN_RR_ENC_B
    };
    for (int p : inputs) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_UP);
    }

    // Attach ISRs to Channel A of each encoder
    gpioSetISRFuncEx(PIN_FL_ENC_A, EITHER_EDGE, 0, encoder_isr_wrapper, (void*)this);
    gpioSetISRFuncEx(PIN_FR_ENC_A, EITHER_EDGE, 0, encoder_isr_wrapper, (void*)this);
    gpioSetISRFuncEx(PIN_RL_ENC_A, EITHER_EDGE, 0, encoder_isr_wrapper, (void*)this);
    gpioSetISRFuncEx(PIN_RR_ENC_A, EITHER_EDGE, 0, encoder_isr_wrapper, (void*)this);
}

void ChassisNode::handle_encoder_tick(int gpio, int level) {
    // Simple logic: if A==B, count up, else down.
    // NOTE: You must verify direction for each wheel experimentally!
    if (gpio == PIN_FL_ENC_A) {
        if (level == gpioRead(PIN_FL_ENC_B)) fl_ticks_++; else fl_ticks_--;
    } 
    else if (gpio == PIN_FR_ENC_A) {
        if (level == gpioRead(PIN_FR_ENC_B)) fr_ticks_--; else fr_ticks_++; // Inverted?
    }
    else if (gpio == PIN_RL_ENC_A) {
        if (level == gpioRead(PIN_RL_ENC_B)) rl_ticks_++; else rl_ticks_--;
    }
    else if (gpio == PIN_RR_ENC_A) {
        if (level == gpioRead(PIN_RR_ENC_B)) rr_ticks_--; else rr_ticks_++; // Inverted?
    }
}

void ChassisNode::stop_motors() {
    set_mecanum_power(0, 0, 0, 0);
}

// Boilerplate Action callbacks... (handle_goal, handle_cancel, handle_accepted same as before)
// (Excluded for brevity, assume standard implementation)

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisNode>());
    rclcpp::shutdown();
    return 0;
}