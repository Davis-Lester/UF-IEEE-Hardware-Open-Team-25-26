#include "hardware_team_robot/encoder_driver.h"
#include <iostream>
#include <stdio.h>

namespace Hardware {

// Global instance pointer used by lgpio alert callbacks.
// lgpio expects C-style functions, so this provides a bridge
// back into the class-based handler.
static EncoderDriver* g_encoder_driver_instance = nullptr;

// C-style alert callback required by lgpio.
// lgpio batches alerts — loop through all of them.
// Signature is fixed by lgpio: (int num_alerts, lgGpioAlert_p alerts, void* userdata)
static void lgpio_encoder_callback(int num_alerts, lgGpioAlert_p alerts, void* userdata) {
    if (!g_encoder_driver_instance) return;
    for (int i = 0; i < num_alerts; i++) {
        g_encoder_driver_instance->handleEncoderTick(
            alerts[i].report.gpio,
            alerts[i].report.level
        );
    }
}

EncoderDriver::EncoderDriver(int fl_a, int fl_b, int fr_a, int fr_b,
                             int rl_a, int rl_b, int rr_a, int rr_b)
    : fl_encoder_{fl_a, fl_b}, fr_encoder_{fr_a, fr_b},
      rl_encoder_{rl_a, rl_b}, rr_encoder_{rr_a, rr_b},
      lgh_(-1) {
    g_encoder_driver_instance = this;
}

EncoderDriver::~EncoderDriver() {
    cleanup();
}

int EncoderDriver::initialize() {
    // Prevent duplicate initialization
    if (is_initialized_.load()) {
        return 0;
    }

    // Open gpiochip0 (the main GPIO header on Pi 4)
    lgh_ = lgGpiochipOpen(0);
    if (lgh_ < 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: lgGpiochipOpen failed (error %d)\n", lgh_);
        return -1;
    }

    // Configure all encoder pins as inputs with pull-up resistors
    const int pins[] = {
        fl_encoder_.channel_a, fl_encoder_.channel_b,
        fr_encoder_.channel_a, fr_encoder_.channel_b,
        rl_encoder_.channel_a, rl_encoder_.channel_b,
        rr_encoder_.channel_a, rr_encoder_.channel_b
    };

    for (int pin : pins) {
        int ret = lgGpioClaimInput(lgh_, LG_SET_PULL_UP, pin);
        if (ret < 0) {
            fprintf(stderr, "[EncoderDriver] ERROR: lgGpioClaimInput failed for pin %d (error %d)\n", pin, ret);
            lgGpiochipClose(lgh_);
            lgh_ = -1;
            return -1;
        }
    }

    // Register the single shared callback function for all alert pins
    // Must be done before claiming alert lines
    lgGpioSetAlertsFunc(lgh_, fl_encoder_.channel_a, lgpio_encoder_callback, (void*)this);
    lgGpioSetAlertsFunc(lgh_, fr_encoder_.channel_a, lgpio_encoder_callback, (void*)this);
    lgGpioSetAlertsFunc(lgh_, rl_encoder_.channel_a, lgpio_encoder_callback, (void*)this);
    lgGpioSetAlertsFunc(lgh_, rr_encoder_.channel_a, lgpio_encoder_callback, (void*)this);

    // Claim alert lines on Channel A pins (rising edge only for 1x decoding)
    int ret = 0;

    ret = lgGpioClaimAlert(lgh_, 0, LG_RISING_EDGE, fl_encoder_.channel_a, -1);
    if (ret < 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: FL alert claim failed (error %d)\n", ret);
        lgGpiochipClose(lgh_);
        lgh_ = -1;
        return -1;
    }

    ret = lgGpioClaimAlert(lgh_, 0, LG_RISING_EDGE, fr_encoder_.channel_a, -1);
    if (ret < 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: FR alert claim failed (error %d)\n", ret);
        lgGpiochipClose(lgh_);
        lgh_ = -1;
        return -1;
    }

    ret = lgGpioClaimAlert(lgh_, 0, LG_RISING_EDGE, rl_encoder_.channel_a, -1);
    if (ret < 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: RL alert claim failed (error %d)\n", ret);
        lgGpiochipClose(lgh_);
        lgh_ = -1;
        return -1;
    }

    ret = lgGpioClaimAlert(lgh_, 0, LG_RISING_EDGE, rr_encoder_.channel_a, -1);
    if (ret < 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: RR alert claim failed (error %d)\n", ret);
        lgGpiochipClose(lgh_);
        lgh_ = -1;
        return -1;
    }

    is_initialized_.store(true);
    printf("[EncoderDriver] lgpio alert decoding active (Channel A rising edges)\n");
    return 0;
}

// Atomic reads are lock-free and ISR-safe
int32_t EncoderDriver::getFrontLeftTicks()  const { return fl_ticks_.load(); }
int32_t EncoderDriver::getFrontRightTicks() const { return fr_ticks_.load(); }
int32_t EncoderDriver::getRearLeftTicks()   const { return rl_ticks_.load(); }
int32_t EncoderDriver::getRearRightTicks()  const { return rr_ticks_.load(); }

void EncoderDriver::reset() {
    fl_ticks_.store(0);
    fr_ticks_.store(0);
    rl_ticks_.store(0);
    rr_ticks_.store(0);
}

void EncoderDriver::cleanup() {
    if (is_initialized_.load()) {
        if (lgh_ >= 0) {
            lgGpiochipClose(lgh_);
            lgh_ = -1;
        }
        is_initialized_.store(false);
    }
}

// Determines rotation direction using quadrature phase relationship.
// Channel B is read at the moment Channel A rises to determine direction.
int8_t EncoderDriver::getDirection(int gpio, int level) {
    if (level != 1) return 0;  // Only count on rising edge

    if (gpio == fl_encoder_.channel_a) {
        return (lgGpioRead(lgh_, fl_encoder_.channel_b) == 0) ? 1 : -1;
    }
    if (gpio == fr_encoder_.channel_a) {
        return (lgGpioRead(lgh_, fr_encoder_.channel_b) == 0) ? 1 : -1;
    }
    if (gpio == rl_encoder_.channel_a) {
        return (lgGpioRead(lgh_, rl_encoder_.channel_b) == 0) ? 1 : -1;
    }
    if (gpio == rr_encoder_.channel_a) {
        return (lgGpioRead(lgh_, rr_encoder_.channel_b) == 0) ? 1 : -1;
    }

    return 0;  // Unexpected GPIO source
}

// Instance-level alert handler — called from lgpio_encoder_callback
void EncoderDriver::handleEncoderTick(int gpio, int level) {
    if (!is_initialized_.load()) return;

    int8_t direction = getDirection(gpio, level);
    if (direction == 0) return;

    if      (gpio == fl_encoder_.channel_a) fl_ticks_.fetch_add(direction);
    else if (gpio == fr_encoder_.channel_a) fr_ticks_.fetch_add(direction);
    else if (gpio == rl_encoder_.channel_a) rl_ticks_.fetch_add(direction);
    else if (gpio == rr_encoder_.channel_a) rr_ticks_.fetch_add(direction);
}

} // namespace Hardware