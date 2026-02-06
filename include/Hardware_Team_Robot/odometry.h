#ifndef HARDWARE_TEAM_ROBOT_SUBSYSTEMS_ODOMETRY_H
#define HARDWARE_TEAM_ROBOT_SUBSYSTEMS_ODOMETRY_H

#include <cmath>

class Odometry {
public:
    Odometry();

    /**
     * @brief Resets the internal position to (0,0).
     */
    void reset();

    /**
     * @brief Updates the global position based on encoder deltas and current gyro heading.
     * * @param fl_tick_delta Change in Front-Left ticks since last loop
     * @param fr_tick_delta Change in Front-Right ticks since last loop
     * @param rl_tick_delta Change in Rear-Left ticks since last loop
     * @param rr_tick_delta Change in Rear-Right ticks since last loop
     * @param gyro_heading_deg Current absolute heading from IMU in degrees
     */
    void update(long fl_tick_delta, long fr_tick_delta, 
                long rl_tick_delta, long rr_tick_delta, 
                double gyro_heading_deg);

    // Getters
    double getX() const { return x_global_; }
    double getY() const { return y_global_; }

private:
    double x_global_;
    double y_global_;

    // Constants (Tune these to match your wheel size!)
    // Example: 4 inch wheels, 19:1 motor (~1216 ticks/rev)
    // Inches per tick = (4 * PI) / 1216 ~= 0.0103
    static constexpr double INCHES_PER_TICK = 0.01033; 
};

#endif // HARDWARE_TEAM_ROBOT_SUBSYSTEMS_ODOMETRY_H