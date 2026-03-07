#ifndef HARDWARE_TEAM_ROBOT_GY31_H
#define HARDWARE_TEAM_ROBOT_GY31_H

#include <cstdint>
#include <atomic>

class GY31 {
public:
    struct RGB {
        int r;
        int g;
        int b;
    };

    /**
     * @brief Constructor defines the wiring.
     * @param s0_pin Scale selection pin 0
     * @param s1_pin Scale selection pin 1
     * @param s2_pin Filter selection pin 2
     * @param s3_pin Filter selection pin 3
     * @param out_pin Frequency output pin from sensor
     */
    GY31(int s0_pin, int s1_pin, int s2_pin, int s3_pin, int out_pin);
    ~GY31();

    // Setup GPIO and Scaling
    bool initialize();

    // Reads raw frequency values for all 3 channels + Clear
    // This is a blocking call (takes ~40-50ms total)
    RGB readRGB();

    // Calibrate black/white levels (simple min/max normalization)
    // Place sensor on white surface and call this.
    void calibrateWhite();
    // Place sensor on black surface and call this.
    void calibrateBlack();

private:
    int pin_s0, pin_s1, pin_s2, pin_s3, pin_out;
    
    // Calibration storage
    RGB white_cal_ = {255, 255, 255}; 
    RGB black_cal_ = {0, 0, 0};

    // Internal helper to select filter and measure frequency
    int readChannel(int s2_state, int s3_state);

    // Static ISR wrapper for pigpio
    static void pulseISR(int gpio, int level, uint32_t tick, void* user);
    void handlePulse();

    // Atomic counter for the ISR
    std::atomic<volatile int> pulse_count_{0};
};

#endif // HARDWARE_TEAM_ROBOT_GY31_H