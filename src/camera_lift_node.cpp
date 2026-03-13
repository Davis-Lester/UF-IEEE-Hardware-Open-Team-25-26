#include "hardware_team_robot/camera_lift_node.h"
#include <thread>

namespace Hardware {

CameraLiftController::CameraLiftController(uint8_t i2c_bus, uint8_t i2c_addr)
    : pca_(std::make_unique<PCA9685Driver>(i2c_bus, i2c_addr)) {
}

CameraLiftController::~CameraLiftController() {
    stop();
}

bool CameraLiftController::initialize() {
    if (!pca_) return false;
    if (!pca_->initialize()) {
        return false;
    }
    stop();
    return true;
}

void CameraLiftController::setChannel(uint8_t channel, uint16_t value) {
    if (!pca_) return;
    // value range 0-4095; if 0 or 4095 is used for off/on in PCA9685 logic
    pca_->setPWM(channel, 0, value);
}

void CameraLiftController::stop() {
    if (!pca_) return;
    pca_->setPWM(PWM_CHANNEL_UP, 0, 4096);
    pca_->setPWM(PWM_CHANNEL_DOWN, 0, 4096);
}

void CameraLiftController::liftUp(uint32_t duration_ms) {
    if (!pca_) return;
    setChannel(PWM_CHANNEL_DOWN, 4096); // reverse off
    setChannel(PWM_CHANNEL_UP, LIFT_PWM_POWER);
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
    stop();
}

void CameraLiftController::liftDown(uint32_t duration_ms) {
    if (!pca_) return;
    setChannel(PWM_CHANNEL_UP, 4096); // forward off
    setChannel(PWM_CHANNEL_DOWN, LIFT_PWM_POWER);
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
    stop();
}

} // namespace Hardware
