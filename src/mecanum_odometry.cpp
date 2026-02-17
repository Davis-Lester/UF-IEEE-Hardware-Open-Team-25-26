#include "hardware_team_robot/mecanum_odometry.h"
#include <cmath>
#include <cstdio>

namespace Hardware {

MecanumOdometry::MecanumOdometry(float wheelbase_mm, float track_width_mm, float wheel_diameter_mm)
    : wheelbase_mm_(wheelbase_mm),
      track_width_mm_(track_width_mm),
      wheel_diameter_mm_(wheel_diameter_mm),
      last_fl_ticks_(0),
      last_fr_ticks_(0),
      last_rl_ticks_(0),
      last_rr_ticks_(0) {
    
    // Robot starts at origin
    current_pose_ = {0.0f, 0.0f, 0.0f};
    
    // Pre-calculate constants for efficiency
    // Wheel circumference = π * diameter
    float wheel_circumference_mm = M_PI * wheel_diameter_mm_;
    
    // Motor CPR: 64 (motor) * 18.75 (gearbox) = 1200 CPR at wheel
    const float CPR_PER_REVOLUTION = 1200.0f;
    
    // Distance per encoder tick (in mm)
    mm_per_tick_ = wheel_circumference_mm / CPR_PER_REVOLUTION;
    
    // Convert to inches (1 inch = 25.4 mm)
    inches_per_tick_ = mm_per_tick_ / 25.4f;
    
    // Pre-calculate mecanum kinematic scaling factors
    // These are the denominators in the forward/strafe/rotation equations
    kp_forward_ = 1.0f / 4.0f;       // (VFL + VFR + VRL + VRR) / 4
    kp_strafe_ = 1.0f / 4.0f;        // (-VFL + VFR + VRL - VRR) / 4
    kp_rotation_ = 1.0f / (4.0f * (wheelbase_mm_ + track_width_mm_));
    
    printf("[MecanumOdometry] Initialized:\n");
    printf("  Wheel diameter: %.1f mm\n", wheel_diameter_mm_);
    printf("  Wheelbase: %.1f mm, Track width: %.1f mm\n", wheelbase_mm_, track_width_mm_);
    printf("  Distance per tick: %.4f mm (%.6f inches)\n", mm_per_tick_, inches_per_tick_);
}

MecanumOdometry::~MecanumOdometry() {
    // Cleanup if needed
}

void MecanumOdometry::update(int32_t fl_ticks, int32_t fr_ticks,
                             int32_t rl_ticks, int32_t rr_ticks,
                             float heading_rad) {
    // Calculate deltas since last update
    int32_t dfl = fl_ticks - last_fl_ticks_;
    int32_t dfr = fr_ticks - last_fr_ticks_;
    int32_t drl = rl_ticks - last_rl_ticks_;
    int32_t drr = rr_ticks - last_rr_ticks_;
    
    // Update last readings
    last_fl_ticks_ = fl_ticks;
    last_fr_ticks_ = fr_ticks;
    last_rl_ticks_ = rl_ticks;
    last_rr_ticks_ = rr_ticks;
    
    // Calculate movement in robot frame
    float d_forward_mm = 0.0f;
    float d_strafe_mm = 0.0f;
    float d_rotation_rad = 0.0f;
    
    calculateDelta(dfl, dfr, drl, drr, d_forward_mm, d_strafe_mm, d_rotation_rad);
    
    // Convert to inches
    float d_forward_inches = d_forward_mm / 25.4f;
    float d_strafe_inches = d_strafe_mm / 25.4f;
    
    // Transform from robot frame to global frame using current heading
    float cos_theta = std::cos(current_pose_.theta_rad);
    float sin_theta = std::sin(current_pose_.theta_rad);
    
    // Global frame transformation:
    // x_global = d_forward * cos(θ) - d_strafe * sin(θ)
    // y_global = d_forward * sin(θ) + d_strafe * cos(θ)
    float dx_global = d_forward_inches * cos_theta - d_strafe_inches * sin_theta;
    float dy_global = d_forward_inches * sin_theta + d_strafe_inches * cos_theta;
    
    // Update pose with thread safety
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_.x_inches += dx_global;
        current_pose_.y_inches += dy_global;
        // Heading is absolute from IMU (not accumulated rotation)
        current_pose_.theta_rad = heading_rad;
    }
}

MecanumOdometry::Pose MecanumOdometry::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_pose_;
}

void MecanumOdometry::reset() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = {0.0f, 0.0f, 0.0f};
    last_fl_ticks_ = 0;
    last_fr_ticks_ = 0;
    last_rl_ticks_ = 0;
    last_rr_ticks_ = 0;
}

void MecanumOdometry::setPose(float x_inches, float y_inches, float theta_rad) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = {x_inches, y_inches, theta_rad};
}

float MecanumOdometry::getInchesPerTick() const {
    return inches_per_tick_;
}

void MecanumOdometry::calculateDelta(int32_t dfl, int32_t dfr, int32_t drl, int32_t drr,
                                     float& d_forward_mm, float& d_strafe_mm, float& d_rotation_rad) {
    // Convert tick deltas to distance in mm
    float dfl_mm = dfl * mm_per_tick_;
    float dfr_mm = dfr * mm_per_tick_;
    float drl_mm = drl * mm_per_tick_;
    float drr_mm = drr * mm_per_tick_;
    
    // Apply mecanum kinematics (inverse kinematics)
    // These equations solve for the robot's velocity components given wheel velocities:
    //
    // For mecanum wheels, the relationship is:
    // VFL =  V_forward - V_strafe - (L+W) * ω
    // VFR =  V_forward + V_strafe + (L+W) * ω
    // VRL =  V_forward + V_strafe - (L+W) * ω
    // VRR =  V_forward - V_strafe + (L+W) * ω
    //
    // Solving for V_forward, V_strafe, and ω:
    // V_forward = (VFL + VFR + VRL + VRR) / 4
    // V_strafe  = (-VFL + VFR + VRL - VRR) / 4
    // ω = (-VFL + VFR - VRL + VRR) / (4 * (L + W))
    
    d_forward_mm  = (dfl_mm + dfr_mm + drl_mm + drr_mm) * kp_forward_;
    d_strafe_mm   = (-dfl_mm + dfr_mm + drl_mm - drr_mm) * kp_strafe_;
    d_rotation_rad = (-dfl_mm + dfr_mm - drl_mm + drr_mm) * kp_rotation_;
}

} // namespace Hardware
