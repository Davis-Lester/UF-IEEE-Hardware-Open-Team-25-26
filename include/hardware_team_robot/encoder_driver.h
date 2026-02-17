#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <atomic>
#include <cstdint>
#include <pigpio.h>

// GPIO PIN CONFIGURATION
// NOTE: Pin numbers below are temporary placeholders.
// Update them once encoder wiring is finalized.
//
// Recommended pins:
//   Use GPIOs with reliable interrupt support such as 4, 17, 22, 23, 24, 25.
//
// Pins to avoid:
//   0–1   Reserved for HAT EEPROM / ID
//   2–3   I2C bus (commonly used by IMU)
//   9–11  SPI bus
//   14–15 UART

namespace Hardware {

/*
 * EncoderDriver
 *
 * Low-level quadrature encoder interface using pigpio interrupts.
 *
 * Four encoders are sampled using a 1× decoding scheme:
 *
 *   • Interrupts are attached to Channel A rising edges only
 *   • Channel B is sampled at each Channel A rising edge
 *   • B = 0 → forward rotation (+1)
 *   • B = 1 → reverse rotation (−1)
 *
 * This approach avoids 4× edge counting noise while maintaining
 * stable direction detection and low ISR overhead.
 *
 * Motor / Encoder Characteristics:
 *
 *   Motor shaft resolution:    64 counts per revolution
 *   Gear reduction:            18.75 : 1
 *   Output shaft resolution:   1200 counts per revolution
 *
 * Wheel geometry:
 *
 *   Diameter:       60 mm
 *   Circumference:  π × 60 mm ≈ 188.5 mm
 *
 * Distance per tick:
 *
 *   ≈ 0.157 mm per tick
 *   ≈ 0.00618 inches per tick
 *
 * Thread Safety:
 *
 *   Tick counters are atomic to allow safe updates from ISRs
 *   without locks or shared mutable state.
 */

class EncoderDriver {
public:
    EncoderDriver();
    ~EncoderDriver();

    // Initializes pigpio and registers GPIO interrupts
    // Returns 0 on success, −1 on failure
    int initialize();

    // Individual wheel tick accessors
    int32_t getFrontLeftTicks() const;
    int32_t getFrontRightTicks() const;
    int32_t getRearLeftTicks() const;
    int32_t getRearRightTicks() const;

    // Resets all counters to zero
    void reset();

    // Releases pigpio and GPIO resources
    void cleanup();

    // Static wrapper required by pigpio callback API (Public so it is accessible by pigpio)
    static void handleEncoderTickStatic(int gpio, int level, uint32_t tick, void* user_data);
private:

    // GPIO pair for one quadrature encoder
    struct EncoderPins {
        int channel_a;
        int channel_b;
    };

    // Temporary pin assignments
    EncoderPins fl_encoder_ = {17, 27};
    EncoderPins fr_encoder_ = {22, 4};
    EncoderPins rl_encoder_ = {26, 21};
    EncoderPins rr_encoder_ = {9, 11};

    // Indicates driver initialization state
    std::atomic<bool> is_initialized_{false};

    // Tick counters updated from interrupt handlers
    std::atomic<int32_t> fl_ticks_{0};
    std::atomic<int32_t> fr_ticks_{0};
    std::atomic<int32_t> rl_ticks_{0};
    std::atomic<int32_t> rr_ticks_{0};

    // Determines rotation direction using quadrature logic
    int8_t getDirection(int gpio, int level);

    

    // Instance-level interrupt handler
    void handleEncoderTick(int gpio, int level);
};

} // namespace Hardware

#endif // ENCODER_DRIVER_H
