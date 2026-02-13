#ifndef MECANUM_ODOMETRY_H
#define MECANUM_ODOMETRY_H

#include <atomic>
#include <cstdint>
#include <cmath>
#include <mutex>

namespace Hardware {

/*
 * MecanumOdometry
 *
 * Position estimation for a mecanum drive robot using wheel encoder data
 * combined with an absolute heading source.
 *
 * Encoder measurements from the four wheels are converted into local robot
 * motion components, then rotated into the global field frame.
 *
 * Coordinate Frame:
 *
 *   Origin (0, 0)     Robot starting position
 *   +X direction      Along field length
 *   +Y direction      Along field width
 *   θ = 0             Facing +X
 *   Positive θ        Counter-clockwise rotation
 *
 * Units:
 *
 *   Distance          Inches (external), millimeters (intermediate math)
 *   Angles            Radians
 *
 * Encoder Resolution:
 *
 *   1200 counts per wheel revolution (after gearbox)
 *   ≈ 0.00618 inches per tick
 *
 * Mecanum Motion Model:
 *
 *   Forward  = (FL + FR + RL + RR) / 4
 *   Strafe   = (-FL + FR + RL - RR) / 4
 *   Rotation = (-FL + FR - RL + RR) / (4 × (L + W))
 *
 * Global Frame Update:
 *
 *   x += forward × cos(θ) − strafe × sin(θ)
 *   y += forward × sin(θ) + strafe × cos(θ)
 *
 * Heading:
 *
 *   Orientation is treated as absolute and provided by an IMU or similar
 *   sensor rather than integrated from wheel motion.
 *
 * Thread Safety:
 *
 *   Pose access is protected with a mutex to ensure consistent reads and
 *   updates across threads.
 */

class MecanumOdometry {
public:

    // Robot pose expressed in global field coordinates
    struct Pose {
        float x_inches;
        float y_inches;
        float theta_rad;
    };

    // Stores physical dimensions of the robot drivetrain
    MecanumOdometry(float wheelbase_mm,
                    float track_width_mm,
                    float wheel_diameter_mm);

    ~MecanumOdometry();

    // Updates pose using latest encoder counts and IMU heading
    void update(int32_t fl_ticks,
                int32_t fr_ticks,
                int32_t rl_ticks,
                int32_t rr_ticks,
                float heading_rad);

    // Returns a thread-safe copy of the current pose
    Pose getCurrentPose() const;

    // Resets pose and internal encoder history
    void reset();

    // Forces pose to a specified value
    void setPose(float x_inches,
                 float y_inches,
                 float theta_rad);

    // Conversion factor derived from wheel geometry and encoder resolution
    float getInchesPerTick() const;

private:

    // Fixed physical parameters
    float wheelbase_mm_;
    float track_width_mm_;
    float wheel_diameter_mm_;

    // Precomputed scaling constants
    float mm_per_tick_;
    float inches_per_tick_;

    // Kinematic scaling factors
    float kp_forward_;
    float kp_strafe_;
    float kp_rotation_;

    // Current estimated pose
    Pose current_pose_;

    // Previous encoder readings used for delta calculation
    int32_t last_fl_ticks_;
    int32_t last_fr_ticks_;
    int32_t last_rl_ticks_;
    int32_t last_rr_ticks_;

    // Protects pose reads and writes
    mutable std::mutex pose_mutex_;

    // Converts wheel tick deltas into local motion components
    void calculateDelta(int32_t dfl,
                        int32_t dfr,
                        int32_t drl,
                        int32_t drr,
                        float& d_forward_mm,
                        float& d_strafe_mm,
                        float& d_rotation_rad);
};

} // namespace Hardware

#endif // MECANUM_ODOMETRY_H
