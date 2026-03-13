#ifndef CAMERA_LIFT_NODE_H
#define CAMERA_LIFT_NODE_H

#include "hardware_team_robot/sensors/pca9685_driver.h"
#include <chrono>
#include <thread>
#include <memory>

// Timing (ms) for lift motion due no hard stop
#define CAMERA_LIFT_MOVEMENT_MS 500

namespace Hardware {

class CameraLiftController {
public:
    // PWM channels for lift motor on PCA9685
    static constexpr uint8_t PWM_CHANNEL_UP = 0;   // forward
    static constexpr uint8_t PWM_CHANNEL_DOWN = 1; // reverse

    // 0-4095 range for PCA9685
    static constexpr uint16_t LIFT_PWM_POWER = 3072; // ~75%

    explicit CameraLiftController(uint8_t i2c_bus = 1, uint8_t i2c_addr = 0x40);
    ~CameraLiftController();

    bool initialize();
    void liftUp(uint32_t duration_ms = CAMERA_LIFT_MOVEMENT_MS);
    void liftDown(uint32_t duration_ms = CAMERA_LIFT_MOVEMENT_MS);
    void stop();

private:
    std::unique_ptr<PCA9685Driver> pca_;    
    void setChannel(uint8_t channel, uint16_t value);
};

} // namespace Hardware

#endif // CAMERA_TILT_NODE_H