#include "hardware_team_robot/tank_odometry.h"
#include <cmath>
#include <cstdio>

namespace Hardware {

TankOdometry::TankOdometry(float track_width_mm, float wheelbase_mm, float wheel_diameter_mm)
    : track_width_mm_(track_width_mm),
      wheelbase_mm_(wheelbase_mm),
      wheel_diameter_mm_(wheel_diameter_mm),
      last_fl_ticks_(0),
      last_fr_ticks_(0),
      last_rl_ticks_(0),
      last_rr_ticks_(0) {
    
    current_pose_ = {0.0f, 0.0f, 0.0f};
    
    float wheel_circumference_mm = M_PI * wheel_diameter_mm_;
    const float CPR_PER_REVOLUTION = 1200.0f;
    
    mm_per_tick_ = wheel_circumference_mm / CPR_PER_REVOLUTION;
    inches_per_tick_ = mm_per_tick_ / 25.4f;
    
    printf("[TankOdometry] Initialized:\n");
    printf("  Wheel diameter: %.1f mm (%.2f inches)\n", wheel_diameter_mm_, wheel_diameter_mm_ / 25.4f);
    printf("  Track width: %.1f mm (%.2f inches)\n", track_width_mm_, track_width_mm_ / 25.4f);
    printf("  Wheelbase: %.1f mm (%.2f inches)\n", wheelbase_mm_, wheelbase_mm_ / 25.4f);
    printf("  Distance per tick: %.4f mm (%.6f inches)\n", mm_per_tick_, inches_per_tick_);
}

TankOdometry::~TankOdometry() {
}

// ========== Thread-safe update with proper mutex protection ==========
void TankOdometry::update(int32_t fl_ticks, int32_t fr_ticks,
                          int32_t rl_ticks, int32_t rr_ticks,
                          float heading_rad) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    
    // Calculate deltas
    int32_t dfl = fl_ticks - last_fl_ticks_;
    int32_t dfr = fr_ticks - last_fr_ticks_;
    int32_t drl = rl_ticks - last_rl_ticks_;
    int32_t drr = rr_ticks - last_rr_ticks_;
    
    // Update encoder baselines
    last_fl_ticks_ = fl_ticks;
    last_fr_ticks_ = fr_ticks;
    last_rl_ticks_ = rl_ticks;
    last_rr_ticks_ = rr_ticks;
    
    // Calculate motion
    float d_forward_mm = 0.0f;
    float d_rotation_rad = 0.0f;
    calculateDelta(dfl, dfr, drl, drr, d_forward_mm, d_rotation_rad);
    
    float d_forward_inches = d_forward_mm / 25.4f;

    // Use midpoint heading for accurate arc projection
    float heading_mid = (current_pose_.theta_rad + heading_rad) * 0.5f;
    float cos_theta = std::cos(heading_mid);
    float sin_theta = std::sin(heading_mid);

    float dx_global = d_forward_inches * cos_theta;
    float dy_global = d_forward_inches * sin_theta;

    // Update pose
    current_pose_.x_inches += dx_global;
    current_pose_.y_inches += dy_global;
    current_pose_.theta_rad = heading_rad;  // Absolute from IMU
}
// =============================================================================

TankOdometry::Pose TankOdometry::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_pose_;
}

void TankOdometry::reset() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = {0.0f, 0.0f, 0.0f};
}

void TankOdometry::setPose(float x_inches, float y_inches, float theta_rad) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = {x_inches, y_inches, theta_rad};
}

float TankOdometry::getInchesPerTick() const {
    return inches_per_tick_;
}

void TankOdometry::calculateDelta(int32_t dfl, int32_t dfr, int32_t drl, int32_t drr,
                                  float& d_forward_mm, float& d_rotation_rad) {
    float dfl_mm = dfl * mm_per_tick_;
    float dfr_mm = dfr * mm_per_tick_;
    float drl_mm = drl * mm_per_tick_;
    float drr_mm = drr * mm_per_tick_;
    
    // Tank drive kinematics: average left side and right side
    float left_avg_mm = (dfl_mm + drl_mm) / 2.0f;
    float right_avg_mm = (dfr_mm + drr_mm) / 2.0f;
    
    d_forward_mm = (left_avg_mm + right_avg_mm) / 2.0f;
    d_rotation_rad = (right_avg_mm - left_avg_mm) / track_width_mm_;
}

} // namespace Hardware