#ifndef HARDWARE_TEAM_ROBOT_ULTRASONICDISTANCE_H
#define HARDWARE_TEAM_ROBOT_ULTRASONICDISTANCE_H

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

#include <atomic>
#include <cstdint>

namespace Hardware {

class UltrasonicDriver {
public:
    // Struct to cleanly pass pin configurations
    struct Config {
        int trig_pin;
        int echo_pin;
    };

    UltrasonicDriver(Config left_config, Config right_config);
    ~UltrasonicDriver();

    // Sets up GPIO modes and attaches the interrupts
    bool initialize();
    
    // Fires the 10-microsecond trigger pulse for both sensors via DMA
    void trigger();

    // Returns distance in inches. 
    // Automatically resets the stored value to -1.0 after reading to prevent stale data.
    float getLeftDistance();
    float getRightDistance();

private:
    Config left_, right_;
    
    // Timers for pulse width calculation
    uint32_t left_start_tick_{0};
    uint32_t right_start_tick_{0};
    
    // Thread-safe storage for the latest calculated distances
    std::atomic<float> left_distance_{-1.0f};
    std::atomic<float> right_distance_{-1.0f};

    // Static wrapper and instance method for pigpio interrupts
    static void echoISR(int gpio, int level, uint32_t tick, void* user);
    void handleEcho(int gpio, int level, uint32_t tick);
};

} // namespace Hardware

#endif // HARDWARE_TEAM_ROBOT_ULTRASONICDISTANCE_H