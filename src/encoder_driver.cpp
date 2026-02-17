#include "hardware_team_robot/encoder_driver.h"
#include <iostream>
#include <stdio.h>

namespace Hardware {

// Global instance pointer used by pigpio interrupt callbacks.
// pigpio expects C-style functions, so this provides a bridge
// back into the class-based handler.
static EncoderDriver* g_encoder_driver_instance = nullptr;

// C-style interrupt callback required by pigpio.
// Delegates processing to the active EncoderDriver instance.
static void pigpio_encoder_callback(int gpio, int level, uint32_t tick, void* user_data) {
    if (g_encoder_driver_instance) {
        g_encoder_driver_instance->handleEncoderTickStatic(gpio, level, tick, user_data);
    }
}

EncoderDriver::EncoderDriver() {
    // Register this instance for interrupt routing
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

    // Start pigpio subsystem
    if (gpioInitialise() < 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: pigpio initialization failed\n");
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
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDown(pin, PI_PUD_UP);
    }

    // Attach interrupts to Channel A pins only.
    // Counting on rising edges yields stable 1× decoding
    // and avoids excessive edge-trigger noise.
    int ret = 0;

    ret = gpioSetISRFuncEx(fl_encoder_.channel_a, RISING_EDGE, 0,
                           pigpio_encoder_callback, (void*)this);
    if (ret != 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: FL ISR registration failed\n");
        gpioTerminate();
        return -1;
    }

    ret = gpioSetISRFuncEx(fr_encoder_.channel_a, RISING_EDGE, 0,
                           pigpio_encoder_callback, (void*)this);
    if (ret != 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: FR ISR registration failed\n");
        gpioTerminate();
        return -1;
    }

    ret = gpioSetISRFuncEx(rl_encoder_.channel_a, RISING_EDGE, 0,
                           pigpio_encoder_callback, (void*)this);
    if (ret != 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: RL ISR registration failed\n");
        gpioTerminate();
        return -1;
    }

    ret = gpioSetISRFuncEx(rr_encoder_.channel_a, RISING_EDGE, 0,
                           pigpio_encoder_callback, (void*)this);
    if (ret != 0) {
        fprintf(stderr, "[EncoderDriver] ERROR: RR ISR registration failed\n");
        gpioTerminate();
        return -1;
    }

    is_initialized_.store(true);

    printf("[EncoderDriver] Interrupt decoding active (Channel A rising edges)\n");
    return 0;
}

// Atomic reads are lock-free and ISR-safe
int32_t EncoderDriver::getFrontLeftTicks() const  { return fl_ticks_.load(); }
int32_t EncoderDriver::getFrontRightTicks() const { return fr_ticks_.load(); }
int32_t EncoderDriver::getRearLeftTicks() const   { return rl_ticks_.load(); }
int32_t EncoderDriver::getRearRightTicks() const  { return rr_ticks_.load(); }

void EncoderDriver::reset() {
    fl_ticks_.store(0);
    fr_ticks_.store(0);
    rl_ticks_.store(0);
    rr_ticks_.store(0);
}

void EncoderDriver::cleanup() {
    if (is_initialized_.load()) {
        gpioTerminate();
        is_initialized_.store(false);
    }
}

// Determines rotation direction using quadrature phase relationship.
// Only Channel A rising edges contribute to count updates.
int8_t EncoderDriver::getDirection(int gpio, int level) {

    // Front Left
    if (gpio == fl_encoder_.channel_a) {
        if (level == 1) {
            int b_state = gpioRead(fl_encoder_.channel_b);
            return (b_state == 0) ? 1 : -1;
        }
        return 0;
    }

    // Front Right
    if (gpio == fr_encoder_.channel_a) {
        if (level == 1) {
            int b_state = gpioRead(fr_encoder_.channel_b);
            return (b_state == 0) ? 1 : -1;
        }
        return 0;
    }

    // Rear Left
    if (gpio == rl_encoder_.channel_a) {
        if (level == 1) {
            int b_state = gpioRead(rl_encoder_.channel_b);
            return (b_state == 0) ? 1 : -1;
        }
        return 0;
    }

    // Rear Right
    if (gpio == rr_encoder_.channel_a) {
        if (level == 1) {
            int b_state = gpioRead(rr_encoder_.channel_b);
            return (b_state == 0) ? 1 : -1;
        }
        return 0;
    }

    // Unexpected GPIO source
    return 0;
}

// Static adapter invoked by pigpio
void EncoderDriver::handleEncoderTickStatic(int gpio, int level,
                                            uint32_t tick, void* user_data) {
    EncoderDriver* driver = static_cast<EncoderDriver*>(user_data);
    if (driver) {
        driver->handleEncoderTick(gpio, level);
    }
}

// Instance-level interrupt handler
void EncoderDriver::handleEncoderTick(int gpio, int level) {

    if (!is_initialized_.load()) {
        return;
    }

    int8_t direction = getDirection(gpio, level);

    // Ignore non-counting transitions
    if (direction == 0) {
        return;
    }

    // Update the corresponding wheel counter
    if (gpio == fl_encoder_.channel_a) {
        fl_ticks_.store(fl_ticks_.load() + direction);
    }
    else if (gpio == fr_encoder_.channel_a) {
        fr_ticks_.store(fr_ticks_.load() + direction);
    }
    else if (gpio == rl_encoder_.channel_a) {
        rl_ticks_.store(rl_ticks_.load() + direction);
    }
    else if (gpio == rr_encoder_.channel_a) {
        rr_ticks_.store(rr_ticks_.load() + direction);
    }
}

} // namespace Hardware
