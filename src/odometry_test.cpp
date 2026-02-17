#include "hardware_team_robot/encoder_driver.h"
#include "hardware_team_robot/mecanum_odometry.h"
#include "hardware_team_robot/navigation_controller.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace Hardware;

/*
 * Odometry Test Program
 *
 * Basic validation harness for encoder decoding, odometry updates,
 * and navigation controller integration.
 *
 * Test flow:
 *
 *   1. Initialize encoder interface
 *   2. Initialize odometry model
 *   3. Inject simulated encoder values
 *   4. Inspect pose estimates
 *   5. Verify navigation controller wiring
 */

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("odometry_test_node");

    RCLCPP_INFO(node->get_logger(),
                "=== IEEE SoutheastCon 2026 Robot - Odometry Test ===");

    // --------------------------------------------------------------------
    // Encoder Driver Initialization
    // --------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "\n--- Encoder Driver Test ---");

    auto encoders = std::make_shared<EncoderDriver>();

    if (encoders->initialize() != 0) {
        RCLCPP_ERROR(node->get_logger(),
                     "Encoder driver initialization failed");
        return -1;
    }

    RCLCPP_INFO(node->get_logger(),
                "Encoder driver initialized");

    // --------------------------------------------------------------------
    // Odometry Initialization
    // --------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "\n--- Odometry Test ---");

    // Physical drivetrain dimensions
    float wheelbase_mm = 177.8f;
    float track_width_mm = 237.5f;
    float wheel_diameter_mm = 60.0f;

    auto odometry =
        std::make_shared<MecanumOdometry>(wheelbase_mm,
                                          track_width_mm,
                                          wheel_diameter_mm);

    odometry->reset();

    RCLCPP_INFO(node->get_logger(), "Odometry model active");
    RCLCPP_INFO(node->get_logger(),
                "Inches per tick: %.6f",
                odometry->getInchesPerTick());

    // --------------------------------------------------------------------
    // Simulated Straight-Line Motion
    // --------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(),
                "\n--- Straight Drive Simulation ---");

    float inches_per_tick = odometry->getInchesPerTick();
    int32_t ticks_for_24_inches =
        (int32_t)(24.0f / inches_per_tick);

    RCLCPP_INFO(node->get_logger(),
                "Simulated ticks for 24 inches: %d",
                ticks_for_24_inches);

    // Uniform forward motion
    odometry->update(ticks_for_24_inches,
                     ticks_for_24_inches,
                     ticks_for_24_inches,
                     ticks_for_24_inches,
                     0.0f);

    auto pose = odometry->getCurrentPose();

    RCLCPP_INFO(node->get_logger(), "Pose after simulation");
    RCLCPP_INFO(node->get_logger(),
                "X: %.2f   Y: %.2f   θ: %.3f rad",
                pose.x_inches,
                pose.y_inches,
                pose.theta_rad);

    float error_x = std::abs(pose.x_inches - 24.0f);

    if (error_x < 0.5f) {
        RCLCPP_INFO(node->get_logger(),
                    "Forward displacement within tolerance");
    } else {
        RCLCPP_WARN(node->get_logger(),
                    "Forward displacement error: %.2f",
                    error_x);
    }

    // --------------------------------------------------------------------
    // Simulated Strafe Motion
    // --------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(),
                "\n--- Strafe Simulation ---");

    odometry->reset();

    int32_t ticks_for_12_inches =
        (int32_t)(12.0f / inches_per_tick);

    // Right strafe pattern
    odometry->update(ticks_for_12_inches,
                     -ticks_for_12_inches,
                     -ticks_for_12_inches,
                     ticks_for_12_inches,
                     0.0f);

    pose = odometry->getCurrentPose();

    RCLCPP_INFO(node->get_logger(),
                "Pose after strafe simulation");
    RCLCPP_INFO(node->get_logger(),
                "X: %.2f   Y: %.2f",
                pose.x_inches,
                pose.y_inches);

    // --------------------------------------------------------------------
    // Simulated Diagonal Motion
    // --------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(),
                "\n--- Diagonal Motion Simulation ---");

    odometry->reset();

    int32_t forward_ticks =
        (int32_t)(10.0f / inches_per_tick);

    int32_t strafe_ticks =
        (int32_t)(5.0f / inches_per_tick);

    odometry->update(forward_ticks - strafe_ticks,
                     forward_ticks + strafe_ticks,
                     forward_ticks + strafe_ticks,
                     forward_ticks - strafe_ticks,
                     0.0f);

    pose = odometry->getCurrentPose();

    float distance =
        std::sqrt(pose.x_inches * pose.x_inches +
                  pose.y_inches * pose.y_inches);

    RCLCPP_INFO(node->get_logger(),
                "Pose after diagonal simulation");
    RCLCPP_INFO(node->get_logger(),
                "X: %.2f   Y: %.2f   Distance: %.2f",
                pose.x_inches,
                pose.y_inches,
                distance);

    // --------------------------------------------------------------------
    // Navigation Controller Wiring
    // --------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(),
                "\n--- Navigation Controller Test ---");

    auto nav = std::make_shared<NavigationController>(odometry, node);

    nav->setDebugLogging(true);
    nav->resetOdometry();

    auto current = nav->getCurrentPose();

    RCLCPP_INFO(node->get_logger(),
                "Initial pose: (%.2f, %.2f) θ=%.3f",
                current.x_inches,
                current.y_inches,
                current.theta_rad);

    // --------------------------------------------------------------------
    // Live Encoder Inspection
    // --------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(),
                "\n--- Encoder Counter Check ---");

    RCLCPP_INFO(node->get_logger(),
                "FL: %d   FR: %d   RL: %d   RR: %d",
                encoders->getFrontLeftTicks(),
                encoders->getFrontRightTicks(),
                encoders->getRearLeftTicks(),
                encoders->getRearRightTicks());

    RCLCPP_INFO(node->get_logger(),
                "Counters remain zero unless hardware is active");

    // --------------------------------------------------------------------
    // Recommended Next Steps
    // --------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "\nNext steps");
    RCLCPP_INFO(node->get_logger(), "Configure GPIO assignments");
    RCLCPP_INFO(node->get_logger(), "Verify encoder polarity");
    RCLCPP_INFO(node->get_logger(), "Calibrate distance constants");
    RCLCPP_INFO(node->get_logger(), "Validate motion on hardware");
    RCLCPP_INFO(node->get_logger(), "Tune controller gains");

    RCLCPP_INFO(node->get_logger(),
                "\nOdometry test complete");

    rclcpp::shutdown();
    return 0;
}
