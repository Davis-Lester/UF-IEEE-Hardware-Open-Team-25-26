// Name: Davis Lester
// Date: 2/5/2026
// Description: Creating a Color Sensor Library for a ROS2 Robot

#include "hardware_team_robot/sensors/GY31.h"
#include <pigpio.h>
#include <unistd.h> // for usleep
#include <algorithm> // for std::clamp

// Filter Selection Logic Table
// Red:    S2=L, S3=L
// Blue:   S2=L, S3=H
// Clear:  S2=H, S3=L
// Green:  S2=H, S3=H

GY31::GY31(int s0, int s1, int s2, int s3, int out) 
    : pin_s0(s0), pin_s1(s1), pin_s2(s2), pin_s3(s3), pin_out(out) {
}

GY31::~GY31() {
    // Detach ISR to stop counting
    gpioSetISRFunc(pin_out, 0, 0, NULL);
}

bool GY31::initialize() {
    if (gpioInitialise() < 0) return false;

    // Output Pins
    gpioSetMode(pin_s0, PI_OUTPUT);
    gpioSetMode(pin_s1, PI_OUTPUT);
    gpioSetMode(pin_s2, PI_OUTPUT);
    gpioSetMode(pin_s3, PI_OUTPUT);

    // Input Pin (The Frequency)
    gpioSetMode(pin_out, PI_INPUT);
    gpioSetPullUpDown(pin_out, PI_PUD_OFF); // Output is usually driven, no pull needed

    // Frequency Scaling
    // Set to 20% (S0=H, S1=L) common for Arduinos/Pis to ensure stability
    // Set to 100% (S0=H, S1=H) for highest sensitivity/speed
    gpioWrite(pin_s0, 1);
    gpioWrite(pin_s1, 0); // 20% scaling

    // Attach ISR to count rising edges
    gpioSetISRFuncEx(pin_out, RISING_EDGE, 0, pulseISR, (void*)this);

    return true;
}

// Static Wrapper
void GY31::pulseISR(int gpio, int level, uint32_t tick, void* user) {
    GY31* sensor = (GY31*)user;
    sensor->handlePulse();
}

// FIX THIS
void GY31::handlePulse() {
    //pulse_count_++;
}

int GY31::readChannel(int s2_state, int s3_state) {
    // 1. Select Filter
    gpioWrite(pin_s2, s2_state);
    gpioWrite(pin_s3, s3_state);

    // 2. Wait for sensor to stabilize filter switch
    usleep(2000); // 2ms

    // 3. Reset Counter
    pulse_count_ = 0;

    // 4. Sample Window (e.g., 20ms)
    // Longer window = More accurate, slower update rate
    usleep(20000); 

    // 5. Get count
    int count = pulse_count_.load();
    
    // Normalize to Hz (Counts per second) if desired, 
    // or just return raw counts for comparison.
    return count; 
}

GY31::RGB GY31::readRGB() {
    RGB val;

    // Read Red (S2=L, S3=L)
    int raw_r = readChannel(0, 0);
    
    // Read Green (S2=H, S3=H)
    int raw_g = readChannel(1, 1);
    
    // Read Blue (S2=L, S3=H)
    int raw_b = readChannel(0, 1);

    // Map Raw Freq to 0-255 using Calibration values
    // Using standard mapping: (x - min) * (255 / (max - min))
    
    auto map_val = [](int x, int min, int max) {
        if (max == min) return 0;
        int res = (int)((float)(x - min) * 255.0f / (float)(max - min));
        return std::clamp(res, 0, 255);
    };

    val.r = map_val(raw_r, black_cal_.r, white_cal_.r);
    val.g = map_val(raw_g, black_cal_.g, white_cal_.g);
    val.b = map_val(raw_b, black_cal_.b, white_cal_.b);

    return val;
}

void GY31::calibrateWhite() {
    // Read Raw values assuming we are looking at white
    white_cal_.r = readChannel(0, 0);
    white_cal_.g = readChannel(1, 1);
    white_cal_.b = readChannel(0, 1);
}

void GY31::calibrateBlack() {
    // Read Raw values assuming we are looking at black/air
    black_cal_.r = readChannel(0, 0);
    black_cal_.g = readChannel(1, 1);
    black_cal_.b = readChannel(0, 1);
}