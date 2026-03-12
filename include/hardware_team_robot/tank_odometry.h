#ifndef TANK_ODOMETRY_H
#define TANK_ODOMETRY_H

#include <atomic>
#include <cstdint>
#include <cmath>
#include <mutex>

namespace Hardware {

/*
 * TankOdometry (Differential Drive)
 * 
 * 4-wheel tank drive: 2 left (FL+RL), 2 right (FR+RR)
 * Wheel diameter: 80mm (3.15 inches)
 * Track width: 9.35 inches (237.49mm)
 * Wheelbase: 7 inches (177.8mm)
 * Encoder: 1200 CPR
 */
class TankOdometry {
public:
    struct Pose {
        float x_inches;
        float y_inches;
        float theta_rad;
    };

    TankOdometry(float track_width_mm,
                 float wheelbase_mm,
                 float wheel_diameter_mm);
    
    ~TankOdometry();

    void update(int32_t fl_ticks,
                int32_t fr_ticks,
                int32_t rl_ticks,
                int32_t rr_ticks,
                float heading_rad);

    Pose getCurrentPose() const;
    void reset();
    void setPose(float x_inches, float y_inches, float theta_rad);
    float getInchesPerTick() const;

private:
    float track_width_mm_;
    float wheelbase_mm_;
    float wheel_diameter_mm_;
    float mm_per_tick_;
    float inches_per_tick_;
    bool has_encoder_baseline_; //This tracks if encoder baseline is init

    Pose current_pose_;

    int32_t last_fl_ticks_;
    int32_t last_fr_ticks_;
    int32_t last_rl_ticks_;
    int32_t last_rr_ticks_;

    mutable std::mutex pose_mutex_;

    void calculateDelta(int32_t dfl, int32_t dfr,
                        int32_t drl, int32_t drr,
                        float& d_forward_mm,
                        float& d_rotation_rad);
};

} // namespace Hardware

#endif // TANK_ODOMETRY_H