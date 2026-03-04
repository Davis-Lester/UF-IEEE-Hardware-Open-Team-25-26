// Name: Davis Lester
// Date: 3/4/2026
// Description: Creating an Ultrasonic Distance Sensor Library for a ROS2 Robot

// ╭╮╱╭┳╮╭╮╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭━━━╮╱╱╱╭╮╱╱╱╱╱╱╱╱╱╱╱╱╱╭━━━╮╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭╮╱╱╱╭╮
// ┃┃╱┃┃┣╯╰╮╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╰╮╭╮┃╱╱╭╯╰╮╱╱╱╱╱╱╱╱╱╱╱╱┃╭━╮┃╱╱╱╱╱╱╱╱╱╱╱╱╱╱┃┃╱╱╱┃┃
// ┃┃╱┃┃┣╮╭╋━┳━━┳━━┳━━┳━╮╭┳━━╮╱┃┃┃┣┳━┻╮╭╋━━┳━╮╭━━┳━━╮┃╰━━┳━━┳━╮╭━━┳━━┳━╮┃┃╱╱╭┫╰━┳━┳━━┳━┳╮╱╭╮
// ┃┃╱┃┃┃┃┃┃╭┫╭╮┃━━┫╭╮┃╭╮╋┫╭━╯╱┃┃┃┣┫━━┫┃┃╭╮┃╭╮┫╭━┫┃━┫╰━━╮┃┃━┫╭╮┫━━┫╭╮┃╭╯┃┃╱╭╋┫╭╮┃╭┫╭╮┃╭┫┃╱┃┃
// ┃╰━╯┃╰┫╰┫┃┃╭╮┣━━┃╰╯┃┃┃┃┃╰━╮╭╯╰╯┃┣━━┃╰┫╭╮┃┃┃┃╰━┫┃━┫┃╰━╯┃┃━┫┃┃┣━━┃╰╯┃┃╱┃╰━╯┃┃╰╯┃┃┃╭╮┃┃┃╰━╯┃
// ╰━━━┻━┻━┻╯╰╯╰┻━━┻━━┻╯╰┻┻━━╯╰━━━┻┻━━┻━┻╯╰┻╯╰┻━━┻━━╯╰━━━┻━━┻╯╰┻━━┻━━┻╯╱╰━━━┻┻━━┻╯╰╯╰┻╯╰━╮╭╯
// ╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭━╯┃
// ╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╰━━╯

#include "hardware_team_robot/ultrasonic_driver.h"
#include <pigpio.h>

namespace Hardware {

// Custom Constructor
UltrasonicDriver::UltrasonicDriver(Config left, Config right) : left_(left), right_(right) {}

// Destructor
UltrasonicDriver::~UltrasonicDriver() {
    // Detach ISRs cleanly upon destruction
    gpioSetISRFuncEx(left_.echo_pin, EITHER_EDGE, 0, nullptr, nullptr);
    gpioSetISRFuncEx(right_.echo_pin, EITHER_EDGE, 0, nullptr, nullptr);
}

// Initalization Sequence
bool UltrasonicDriver::initialize() {
    // Initialize Left Sensor Pins
    gpioSetMode(left_.trig_pin, PI_OUTPUT);
    gpioWrite(left_.trig_pin, 0); // Ensure trigger starts LOW
    gpioSetMode(left_.echo_pin, PI_INPUT);
    if (gpioSetISRFuncEx(left_.echo_pin, EITHER_EDGE, 0, echoISR, (void*)this) < 0) return false;

    // Initialize Right Sensor Pins
    gpioSetMode(right_.trig_pin, PI_OUTPUT);
    gpioWrite(right_.trig_pin, 0);
    gpioSetMode(right_.echo_pin, PI_INPUT);
    if (gpioSetISRFuncEx(right_.echo_pin, EITHER_EDGE, 0, echoISR, (void*)this) < 0) return false;

    return true;
}

// Trigger interrupt
void UltrasonicDriver::trigger() {
    // Fire 10-microsecond pulses; pigpio handles this asynchronously in the background
    gpioTrigger(left_.trig_pin, 10, 1);
    gpioTrigger(right_.trig_pin, 10, 1);
}

// Function for reading left distance from wall
float UltrasonicDriver::getLeftDistance() {
    // .exchange() atomically reads the current value AND replaces it with -1.0f
    return left_distance_.exchange(-1.0f); 
}

// Function for reading right distance from wall
float UltrasonicDriver::getRightDistance() {
    return right_distance_.exchange(-1.0f);
}

// Use echo pin from sensor
void UltrasonicDriver::echoISR(int gpio, int level, uint32_t tick, void* user) {
    if (user) {
        static_cast<UltrasonicDriver*>(user)->handleEcho(gpio, level, tick);
    }
}

// Handle 
void UltrasonicDriver::handleEcho(int gpio, int level, uint32_t tick) {
    // tick is the system microsecond counter
    if (gpio == left_.echo_pin) {
        if (level == 1) { 
            left_start_tick_ = tick;
        } else if (level == 0) { 
            // Convert round-trip microseconds to one-way distance in inches
            left_distance_.store((tick - left_start_tick_) / 148.0f);
        }
    } 
    else if (gpio == right_.echo_pin) {
        if (level == 1) { 
            right_start_tick_ = tick;
        } else if (level == 0) { 
            right_distance_.store((tick - right_start_tick_) / 148.0f);
        }
    }
}

} // namespace Hardware