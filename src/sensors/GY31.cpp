// Name: Davis Lester
// Date: 3/2/2026
// Description: Creating a Color Sensor Library for a ROS2 Robot

// ╭━━━┳╮╱╱╭┳━━━╮╭╮╱╭━━━╮╱╱╭╮╱╱╱╱╱╱╭━━━╮
// ┃╭━╮┃╰╮╭╯┃╭━╮┣╯┃╱┃╭━╮┃╱╱┃┃╱╱╱╱╱╱┃╭━╮┃
// ┃┃╱╰┻╮╰╯╭┻╯╭╯┣╮┃╱┃┃╱╰╋━━┫┃╭━━┳━╮┃╰━━┳━━┳━╮╭━━┳━━┳━╮
// ┃┃╭━╮╰╮╭╯╭╮╰╮┃┃┃╱┃┃╱╭┫╭╮┃┃┃╭╮┃╭╯╰━━╮┃┃━┫╭╮┫━━┫╭╮┃╭╯
// ┃╰┻━┃╱┃┃╱┃╰━╯┣╯╰╮┃╰━╯┃╰╯┃╰┫╰╯┃┃╱┃╰━╯┃┃━┫┃┃┣━━┃╰╯┃┃
// ╰━━━╯╱╰╯╱╰━━━┻━━╯╰━━━┻━━┻━┻━━┻╯╱╰━━━┻━━┻╯╰┻━━┻━━┻╯

#include "hardware_team_robot/sensors/GY31.h"
#include <pigpio.h>
#include <unistd.h>
#include <algorithm>

// Filter Selection Logic Table
// Red:    S2=L, S3=L
// Blue:   S2=L, S3=H
// Clear:  S2=H, S3=L
// Green:  S2=H, S3=H

// Initalize GY31 Object
GY31::GY31(int s0, int s1, int s2, int s3, int out) : pin_s0(s0), pin_s1(s1), pin_s2(s2), pin_s3(s3), pin_out(out) {}

// Destruct
GY31::~GY31() {
    // Detach ISR cleanly using the Ex function and a nullptr callback
    gpioSetISRFuncEx(pin_out, RISING_EDGE, 0, nullptr, nullptr);
}

// Initalize Sensor
bool GY31::initialize() {

    // No pin attached
    if (gpioInitialise() < 0) return false;

    // Output Pins
    gpioSetMode(pin_s0, PI_OUTPUT);
    gpioSetMode(pin_s1, PI_OUTPUT);
    gpioSetMode(pin_s2, PI_OUTPUT);
    gpioSetMode(pin_s3, PI_OUTPUT);

    // Input Pin (The Frequency)
    gpioSetMode(pin_out, PI_INPUT);
    gpioSetPullUpDown(pin_out, PI_PUD_OFF); 

    // Frequency Scaling
    // Set to 20% (S0=H, S1=L) common for Arduinos/Pis to ensure stability
    gpioWrite(pin_s0, 1);
    gpioWrite(pin_s1, 0); 

    // Attach ISR to count rising edges
    gpioSetISRFuncEx(pin_out, RISING_EDGE, 0, pulseISR, (void*)this);

    // Function worked
    return true;
}

// Static Wrapper, check color
void GY31::pulseISR(int gpio, int level, uint32_t tick, void* user) {
    if (user) {
        GY31* sensor = static_cast<GY31*>(user);
        sensor->handlePulse();
    }
}

// Simple counter to run in the background
void GY31::handlePulse() {
    pulse_count_++;
}

// Read colot channel
int GY31::readChannel(int s2_state, int s3_state) {

    // Select Filter
    gpioWrite(pin_s2, s2_state);
    gpioWrite(pin_s3, s3_state);

    // Wait for sensor to stabilize filter switch
    usleep(2000); // 2ms

    // Reset Counter (using safe atomic store)
    pulse_count_.store(0);

    // Sample Window, 20 ms
    usleep(20000); 

    // Get count
    int count = pulse_count_.load();
    
    return count; 
}

// Read full color
GY31::RGB GY31::readRGB() {
    RGB val;

    // Read Red (S2=L, S3=L)
    int raw_r = readChannel(0, 0);
    
    // Read Green (S2=H, S3=H)
    int raw_g = readChannel(1, 1);
    
    // Read Blue (S2=L, S3=H)
    int raw_b = readChannel(0, 1);

    // Map Raw Freq to 0-255 using Calibration values
    auto map_val = [](int x, int min, int max) {
        if (max == min) return 0;
        int res = static_cast<int>(static_cast<float>(x - min) * 255.0f / static_cast<float>(max - min));
        return std::clamp(res, 0, 255);
    };

    // Write values from registers to memory
    val.r = map_val(raw_r, black_cal_.r, white_cal_.r);
    val.g = map_val(raw_g, black_cal_.g, white_cal_.g);
    val.b = map_val(raw_b, black_cal_.b, white_cal_.b);

    return val;
}

// Read white for debugging
void GY31::calibrateWhite() {
    white_cal_.r = readChannel(0, 0);
    white_cal_.g = readChannel(1, 1);
    white_cal_.b = readChannel(0, 1);
}

// Read black for debugging
void GY31::calibrateBlack() {
    black_cal_.r = readChannel(0, 0);
    black_cal_.g = readChannel(1, 1);
    black_cal_.b = readChannel(0, 1);
}