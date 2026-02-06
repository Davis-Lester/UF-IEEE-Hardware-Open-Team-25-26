#include "Hardware_Team_Robot/subsystems/Odometry.h"
#include <cmath>

// Convert Degrees to Radians
static double to_rad(double deg) {
    return deg * M_PI / 180.0;
}

Odometry::Odometry() : x_global_(0.0), y_global_(0.0) {}

void Odometry::reset() {
    x_global_ = 0.0;
    y_global_ = 0.0;
}

void Odometry::update(long fl, long fr, long rl, long rr, double gyro_deg) {
    // 1. Calculate Local Robot Movement (Mecanum Kinematics)
    // Forward (Y in robot frame) = Average of all wheels
    double delta_fwd_ticks = (fl + fr + rl + rr) / 4.0;
    
    // Strafe (X in robot frame) = (FL backward + FR forward + RL forward + RR backward) / 4
    // NOTE: Check your motor directions! Standard Mecanum vectoring:
    // Left Strafe: FL(-), FR(+), RL(+), RR(-)
    // Right Strafe: FL(+), FR(-), RL(-), RR(+)
    // Let's assume positive strafe is RIGHT:
    double delta_strafe_ticks = (-fl + fr + rl - rr) / 4.0;

    // 2. Convert Ticks to Physical Distance (Inches/Meters)
    double d_fwd = delta_fwd_ticks * INCHES_PER_TICK;
    double d_strafe = delta_strafe_ticks * INCHES_PER_TICK;

    // 3. Rotate Local Movement to Global Frame
    // Standard Rotation Matrix:
    // x_global = x_local * cos(theta) - y_local * sin(theta)
    // y_global = x_local * sin(theta) + y_local * cos(theta)
    // *Assuming 0 degrees is "North" (Positive Y)*
    
    double theta = to_rad(gyro_deg);
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    // Update Global X/Y
    // Global X += (Strafe * cos - Fwd * sin)  <-- If 0 deg is North
    // Global Y += (Strafe * sin + Fwd * cos)
    
    // Let's assume standard math convention: 0 deg = East (Positive X)
    // Then:
    x_global_ += (d_fwd * cos_t - d_strafe * sin_t);
    y_global_ += (d_fwd * sin_t + d_strafe * cos_t);
}