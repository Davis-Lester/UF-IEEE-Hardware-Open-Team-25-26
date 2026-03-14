// Name: Davis Lester
// Date: 2/5/2026
// Description: Tank Drive PID Chassis Node (4 Motor / 4 Encoder)

#include "hardware_team_robot/chassis_node.h"
#include <cmath>
#include <algorithm>
#include <thread>
#include <lgpio.h>

// PWM frequency for RGB LED (Hz)
static constexpr int RGB_PWM_FREQ = 800;

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

    stop_motors();

    // Turn off RGB LED and release lgpio handle
    if (rgb_lgh_ >= 0) {
        lgTxPwm(rgb_lgh_, RGB_PIN_RED,   RGB_PWM_FREQ, 0.0);
        lgTxPwm(rgb_lgh_, RGB_PIN_GREEN, RGB_PWM_FREQ, 0.0);
        lgTxPwm(rgb_lgh_, RGB_PIN_BLUE,  RGB_PWM_FREQ, 0.0);
        lgGpiochipClose(rgb_lgh_);
        rgb_lgh_ = -1;
    }
}

void ChassisNode::odometry_update_loop() {
    rclcpp::Rate loop_rate(100);  // 100 Hz update rate

    float heading_rad = 0.0f;
    auto last_update = std::chrono::high_resolution_clock::now();

    while (rclcpp::ok()) {
        if (encoder_driver_ && odometry_) {
            int32_t fl = encoder_driver_->getFrontLeftTicks();
            int32_t fr = encoder_driver_->getFrontRightTicks();
            int32_t rl = encoder_driver_->getRearLeftTicks();
            int32_t rr = encoder_driver_->getRearRightTicks();

            auto now = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(now - last_update).count();
            float gx, gy, gz;
            if (imu_.readGyro(&gx, &gy, &gz) == 0) {
                float gz_rad_per_sec = gz * (M_PI / 180.0f);
                heading_rad += gz_rad_per_sec * dt;

                while (heading_rad >  M_PI) heading_rad -= 2.0f * M_PI;
                while (heading_rad < -M_PI) heading_rad += 2.0f * M_PI;
            }
            last_update = now;

            odometry_->update(fl, fr, rl, rr, heading_rad);
        }

        loop_rate.sleep();
    }

    // Safe shutdown on loop exit
    stop_motors();
}

ChassisNode::ChassisNode() : Node("chassis_node"), imu_(1), rgb_lgh_(-1) {
    // --- Motor Driver (I2C — no pigpio needed) ---
    motor_driver_ = std::make_shared<Hardware::PCA9685Driver>(1, 0x40);
    if (!motor_driver_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "PCA9685 Init Failed: %s",
                     motor_driver_->getLastError().c_str());
        motor_driver_.reset();
    } else {
        RCLCPP_INFO(this->get_logger(), "PCA9685 Motor Driver Ready");
        motor_ready_ = true;
    }

    // --- Odometry + Encoder Driver ---
    odometry_ = std::make_shared<Hardware::TankOdometry>(80.0f, 237.49f, 177.8f);
    encoder_driver_ = std::make_shared<Hardware::EncoderDriver>(
        PIN_FL_ENC_A, PIN_FL_ENC_B,
        PIN_FR_ENC_A, PIN_FR_ENC_B,
        PIN_RL_ENC_A, PIN_RL_ENC_B,
        PIN_RR_ENC_A, PIN_RR_ENC_B
    );

    if (encoder_driver_->initialize() != 0) {
        RCLCPP_ERROR(this->get_logger(), "Encoder driver init failed");
    } else {
        RCLCPP_INFO(this->get_logger(), "Encoders initialized");
    }

    // --- RGB LED via lgpio (separate chip handle from encoder_driver) ---
    // encoder_driver owns gpiochip0 for encoder pins.
    // We open a second handle here for the LED pins — lgpio supports this.
    rgb_lgh_ = lgGpiochipOpen(0);
    if (rgb_lgh_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "lgGpiochipOpen failed for RGB LED (error %d)", rgb_lgh_);
    } else {
        int ret = 0;
        ret |= lgGpioClaimOutput(rgb_lgh_, 0, RGB_PIN_RED,   0);
        ret |= lgGpioClaimOutput(rgb_lgh_, 0, RGB_PIN_GREEN, 0);
        ret |= lgGpioClaimOutput(rgb_lgh_, 0, RGB_PIN_BLUE,  0);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to claim RGB LED GPIO pins");
            lgGpiochipClose(rgb_lgh_);
            rgb_lgh_ = -1;
        } else {
            // Start with LED off
            lgTxPwm(rgb_lgh_, RGB_PIN_RED,   RGB_PWM_FREQ, 0.0);
            lgTxPwm(rgb_lgh_, RGB_PIN_GREEN, RGB_PWM_FREQ, 0.0);
            lgTxPwm(rgb_lgh_, RGB_PIN_BLUE,  RGB_PWM_FREQ, 0.0);
            RCLCPP_INFO(this->get_logger(), "RGB LED GPIO ready");
        }
    }

    // --- /led_cmd subscriber ---
    led_sub_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
        "/led_cmd", 10,
        std::bind(&ChassisNode::led_callback, this, std::placeholders::_1));

    // --- /intake_cmd_validated subscriber ---
    rclcpp::QoS intake_qos(10);
    intake_qos.reliable();
    intake_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/intake_cmd_validated", intake_qos,
        std::bind(&ChassisNode::intake_callback, this, std::placeholders::_1));

    // --- IMU ---
    if (imu_.initialize() == 0) {
        RCLCPP_INFO(this->get_logger(), "IMU Calibrating...");
        imu_.calibrate(500);
    } else {
        RCLCPP_ERROR(this->get_logger(), "IMU initialization failed");
    }

    // --- Odometry thread ---
    odometry_thread_ = std::thread(&ChassisNode::odometry_update_loop, this);

    // --- Drive action server ---
    this->action_server_ = rclcpp_action::create_server<Drive>(
        this, "drive_command",
        std::bind(&ChassisNode::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ChassisNode::handle_cancel,   this, std::placeholders::_1),
        std::bind(&ChassisNode::handle_accepted, this, std::placeholders::_1));

    // --- Motor command topic (from NavigationController) ---
    motor_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "motor_cmd", rclcpp::QoS(10).reliable(),
        std::bind(&ChassisNode::motor_cmd_callback, this, std::placeholders::_1));

    stop_motors();
}

// ColorRGBA uses 0.0–1.0 floats; lgTxPwm wants 0.0–100.0 duty cycle
void ChassisNode::led_callback(const std_msgs::msg::ColorRGBA::SharedPtr msg) {
    if (rgb_lgh_ < 0) return;
    auto to_duty = [](float c) -> double {
        return static_cast<double>(std::clamp(c, 0.0f, 1.0f)) * 100.0;
    };
    lgTxPwm(rgb_lgh_, RGB_PIN_RED,   RGB_PWM_FREQ, to_duty(msg->r));
    lgTxPwm(rgb_lgh_, RGB_PIN_GREEN, RGB_PWM_FREQ, to_duty(msg->g));
    lgTxPwm(rgb_lgh_, RGB_PIN_BLUE,  RGB_PWM_FREQ, to_duty(msg->b));
}

void ChassisNode::intake_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (!motor_driver_ || !motor_ready_) return;
    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_5,
        msg->data > 0 ? 75 : msg->data < 0 ? -75 : 0);
}

void ChassisNode::execute(const std::shared_ptr<GoalHandleDrive> goal_handle) {
    action_active_ = true;
    const auto goal = goal_handle->get_goal();
    auto result   = std::make_shared<Drive::Result>();
    auto feedback = std::make_shared<Drive::Feedback>();

    if (encoder_driver_) encoder_driver_->reset();

    double current_val = 0.0;
    double error = 0.0;
    double kp = (goal->mode == "TURN") ? 2.0 : 0.6;

    rclcpp::Rate loop_rate(50);
    double dt = 0.02;
    int settled_count = 0;

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            stop_motors();
            action_active_ = false;
            goal_handle->canceled(result);
            return;
        }

        if (goal->mode == "TURN") {
            float gx, gy, gz;
            if (imu_.readGyro(&gx, &gy, &gz) == 0) {
                current_val += (gz * dt);
                error = goal->target_value - current_val;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "IMU read failed during turn");
            }
        } else {
            double fl = std::abs(encoder_driver_->getFrontLeftTicks());
            double fr = std::abs(encoder_driver_->getFrontRightTicks());
            double rl = std::abs(encoder_driver_->getRearLeftTicks());
            double rr = std::abs(encoder_driver_->getRearRightTicks());

            double avg_ticks = (fl + fr + rl + rr) / 4.0;
            if (goal->target_value < 0) avg_ticks = -avg_ticks;

            current_val = avg_ticks;
            error = goal->target_value - current_val;
        }

        double output = std::clamp(error * kp, -goal->max_speed, goal->max_speed);

        if (goal->mode == "TURN") set_tank_power(-output, output);
        else                      set_tank_power( output, output);

        double tolerance = (goal->mode == "TURN") ? 2.0 : 25.0;
        if (std::abs(error) < tolerance) settled_count++;
        else                             settled_count = 0;

        if (settled_count > 10) break;

        feedback->current_error = error;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    stop_motors();
    result->success = true;
    action_active_ = false;
    goal_handle->succeed(result);
}

void ChassisNode::set_tank_power(double left, double right) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (!motor_driver_ || !motor_ready_) return;

    int8_t left_pct  = static_cast<int8_t>(std::clamp((left  * 100.0) / 255.0, -100.0, 100.0));
    int8_t right_pct = static_cast<int8_t>(std::clamp((right * 100.0) / 255.0, -100.0, 100.0));

    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_1, left_pct);
    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_3, left_pct);
    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_2, right_pct);
    motor_driver_->setMotorSpeed(Hardware::PCA9685Driver::MOTOR_4, right_pct);
}

void ChassisNode::stop_motors() {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (motor_driver_) {
        motor_driver_->stopAll();
    }
}

void ChassisNode::motor_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (action_active_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Ignoring motor_cmd while action is active");
        return;
    }

    float left_speed  = static_cast<float>(msg->linear.x);
    float right_speed = static_cast<float>(msg->linear.y);

    if (!std::isfinite(left_speed) || !std::isfinite(right_speed)) {
        RCLCPP_WARN(this->get_logger(), "Non-finite motor speeds received, ignoring");
        return;
    }

    set_tank_power(static_cast<double>(left_speed), static_cast<double>(right_speed));
}

rclcpp_action::GoalResponse ChassisNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Drive::Goal>)
{
    if (!motor_ready_) {
        RCLCPP_ERROR(this->get_logger(), "Motor driver not ready — rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (execute_thread_.joinable()) {
        RCLCPP_WARN(this->get_logger(), "Previous action still executing — rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ChassisNode::handle_cancel(
    const std::shared_ptr<GoalHandleDrive>)
{
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