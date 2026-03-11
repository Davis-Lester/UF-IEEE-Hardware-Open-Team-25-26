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
 * Position estimation for a tank/differential drive robot using wheel encoder
 * data combined with an absolute heading source (IMU).
 *
 * TANK DRIVE CHARACTERISTICS:
 *   • 4 wheels: 2 left, 2 right (mechanically linked per side)
 *   • Forward/backward motion: Both sides move same direction
 *   • Rotation: Left and right sides move opposite directions
 *   • NO STRAFE capability (unlike mecanum)
 *
 * Coordinate Frame:
 *   Origin (0, 0)     Robot starting position
 *   +X direction      Forward
 *   +Y direction      Left
 *   θ = 0             Facing +X
 *   Positive θ        Counter-clockwise rotation
 *
 * Units:
 *   Distance          Inches (external), millimeters (intermediate math)
 *   Angles            Radians
 *
 * Encoder Resolution:
 *   1200 counts per wheel revolution (after gearbox)
 *   80mm wheel diameter → ~0.209 mm per tick
 *
 * Tank Drive Motion Model:
 *   Forward  = (Left_Avg + Right_Avg) / 2
 *   Rotation = (Right_Avg - Left_Avg) / track_width
 *
 *   Where:
 *     Left_Avg  = (FL + RL) / 2
 *     Right_Avg = (FR + RR) / 2
 *
 * Global Frame Update:
 *   x += forward × cos(θ)
 *   y += forward × sin(θ)
 *   θ = from IMU (absolute heading)
 *
 * Thread Safety:
 *   Pose access is protected with a mutex
 */
class TankOdometry {
public:
    struct Pose {
        float x_inches;
        float y_inches;
        float theta_rad;
    };

    // Constructor with tank drive physical parameters
    // track_width_mm: Distance between left and right wheels (center-to-center)
    // wheelbase_mm: Distance between front and rear wheels (for reference only)
    // wheel_diameter_mm: Diameter of drive wheels
    TankOdometry(float track_width_mm,
                 float wheelbase_mm,
                 float wheel_diameter_mm);
    
    ~TankOdometry();

    // Updates pose using latest encoder counts and IMU heading
    // fl/rl = left side wheels, fr/rr = right side wheels
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
    // Fixed physical parameters
    float track_width_mm_;      // Left-right wheel spacing
    float wheelbase_mm_;        // Front-back wheel spacing (reference only)
    float wheel_diameter_mm_;   // Wheel diameter

    // Precomputed scaling constants
    float mm_per_tick_;
    float inches_per_tick_;

    // Current estimated pose
    Pose current_pose_;

    // Previous encoder readings for delta calculation
    int32_t last_fl_ticks_;
    int32_t last_fr_ticks_;
    int32_t last_rl_ticks_;
    int32_t last_rr_ticks_;

    mutable std::mutex pose_mutex_;

    // Converts wheel tick deltas into motion components
    void calculateDelta(int32_t dfl, int32_t dfr,
                        int32_t drl, int32_t drr,
                        float& d_forward_mm,
                        float& d_rotation_rad);
};

} // namespace Hardware

#endif // TANK_ODOMETRY_H