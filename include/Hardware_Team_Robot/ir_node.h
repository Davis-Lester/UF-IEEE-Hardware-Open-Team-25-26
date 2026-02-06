#ifndef IR_NODE_H
#define IR_NODE_H

// Name: Davis Lester
// Date: 2/5/2026
// Description: Creating a NEC IR Library for ROS2 Robot

//╭━╮╱╭┳━━━┳━━━╮╭━━┳━━━╮╭╮╱╱╱╭╮
//┃┃╰╮┃┃╭━━┫╭━╮┃╰┫┣┫╭━╮┃┃┃╱╱╱┃┃
//┃╭╮╰╯┃╰━━┫┃╱╰╯╱┃┃┃╰━╯┃┃┃╱╱╭┫╰━┳━┳━━┳━┳╮╱╭╮
//┃┃╰╮┃┃╭━━┫┃╱╭╮╱┃┃┃╭╮╭╯┃┃╱╭╋┫╭╮┃╭┫╭╮┃╭┫┃╱┃┃
//┃┃╱┃┃┃╰━━┫╰━╯┃╭┫┣┫┃┃╰╮┃╰━╯┃┃╰╯┃┃┃╭╮┃┃┃╰━╯┃
//╰╯╱╰━┻━━━┻━━━╯╰━━┻╯╰━╯╰━━━┻┻━━┻╯╰╯╰┻╯╰━╮╭╯
//╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭━╯┃
//╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╰━━╯

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <pigpio.h>
#include <vector>

class IRNode : public rclcpp::Node {
public:
    IRNode();
    ~IRNode();

private:
    // ROS 2 Subscription
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;

    // Callback
    void topic_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    // Hardware / Waveform Helpers
    void add_carrier_burst(std::vector<gpioPulse_t>& pulses, int duration_us);
    void add_space(std::vector<gpioPulse_t>& pulses, int duration_us);
    void add_byte(std::vector<gpioPulse_t>& pulses, uint8_t byte);
    void send_nec_dma(uint8_t address, uint8_t command);

    // Constants
    static const int IR_PIN = 18;
    static const int CARRIER_FREQ = 38000;
};

#endif // IR_NODE_H