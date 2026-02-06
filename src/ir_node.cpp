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
#include <chrono>

// --- Configuration ---
// GPIO Pin for the IR LED (Must support DMA, GPIO 18 is standard)
#define IR_LED_PIN 18 

// NEC Protocol Constants (Microseconds)
const int CARRIER_FREQ = 38000;
const int NEC_HDR_MARK = 9000;
const int NEC_HDR_SPACE = 4500;
const int NEC_BIT_MARK = 562;
const int NEC_ONE_SPACE = 1687;
const int NEC_ZERO_SPACE = 562;
const int NEC_END_MARK = 562;

class IRNode : public rclcpp::Node {
public:
    IRNode() : Node("ir_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing IR Node...");

        // 1. Initialize pigpio
        // NOTE: This requires sudo/root privileges to access DMA registers
        if (gpioInitialise() < 0) {
            RCLCPP_FATAL(this->get_logger(), "pigpio initialization failed! Did you run with sudo?");
            throw std::runtime_error("pigpio init failed");
        }

        // 2. Setup Pin
        gpioSetMode(IR_LED_PIN, PI_OUTPUT);

        // 3. Create Subscription
        // Listens for a UInt8 command on 'ir_command' topic
        subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
            "ir_command", 10, std::bind(&IRNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IR Node Ready. Listening on /ir_command");
    }

    ~IRNode() {
        RCLCPP_INFO(this->get_logger(), "Terminating pigpio...");
        gpioTerminate();
    }

private:
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;

    // Callback: Triggered when a message is received
    void topic_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        uint8_t command = msg->data;
        uint8_t address = 0xBB; // Hardcoded Address from your spec

        RCLCPP_INFO(this->get_logger(), "TX NEC: Addr=0x%X Cmd=0x%X", address, command);
        
        // Send the signal using DMA
        send_nec_dma(address, command);
    }

    // --- Waveform Helpers ---

    /**
     * @brief Adds a 38kHz carrier burst to the pulse vector.
     * 38kHz = ~26us period. We use 13us High / 13us Low.
     */
    void add_carrier_burst(std::vector<gpioPulse_t>& pulses, int duration_us) {
        int cycles = duration_us / 26; 

        for (int i = 0; i < cycles; i++) {
            gpioPulse_t p_on, p_off;

            // ON Pulse (13us)
            p_on.gpioOn = (1 << IR_LED_PIN);
            p_on.gpioOff = 0;
            p_on.usDelay = 13;

            // OFF Pulse (13us)
            p_off.gpioOn = 0;
            p_off.gpioOff = (1 << IR_LED_PIN);
            p_off.usDelay = 13;

            pulses.push_back(p_on);
            pulses.push_back(p_off);
        }
    }

    /**
     * @brief Adds a silence (space) to the pulse vector.
     */
    void add_space(std::vector<gpioPulse_t>& pulses, int duration_us) {
        gpioPulse_t p;
        p.gpioOn = 0;
        p.gpioOff = (1 << IR_LED_PIN); // Ensure pin is low
        p.usDelay = duration_us;
        pulses.push_back(p);
    }

    /**
     * @brief Encodes a single byte into NEC pulses.
     */
    void add_byte(std::vector<gpioPulse_t>& pulses, uint8_t byte) {
        for (int i = 0; i < 8; i++) {
            // Check LSB
            bool is_one = (byte >> i) & 1;

            // Bit always starts with a Mark
            add_carrier_burst(pulses, NEC_BIT_MARK);

            // Length of Space determines 0 or 1
            if (is_one) {
                add_space(pulses, NEC_ONE_SPACE);
            } else {
                add_space(pulses, NEC_ZERO_SPACE);
            }
        }
    }

    /**
     * @brief Constructs the full NEC waveform and sends it via DMA.
     */
    void send_nec_dma(uint8_t address, uint8_t command) {
        // Safety: Don't interrupt an existing transmission
        if (gpioWaveTxBusy()) {
            RCLCPP_WARN(this->get_logger(), "TX Busy! Dropping command.");
            return;
        }

        std::vector<gpioPulse_t> pulses;
        // Reserve memory to prevent reallocations during push_back
        pulses.reserve(600); 

        // 1. Header (9ms Mark, 4.5ms Space)
        add_carrier_burst(pulses, NEC_HDR_MARK);
        add_space(pulses, NEC_HDR_SPACE);

        // 2. Address (8 bits)
        add_byte(pulses, address);

        // 3. Inverse Address (8 bits)
        add_byte(pulses, (uint8_t)~address);

        // 4. Command (8 bits)
        add_byte(pulses, command);

        // 5. Inverse Command (8 bits)
        add_byte(pulses, (uint8_t)~command);

        // 6. End Pulse (562us Mark) + Trailing Silence
        add_carrier_burst(pulses, NEC_END_MARK);
        add_space(pulses, 1000); // Wait 1ms to ensure clean stop

        // --- DMA Transmission ---
        
        // Clear previous waveform to free memory
        gpioWaveClear();

        // Create new waveform from our vector
        gpioWaveAddGeneric(pulses.size(), pulses.data());
        int wave_id = gpioWaveCreate();

        if (wave_id >= 0) {
            // Send One-Shot (Non-blocking call to DMA engine)
            gpioWaveTxSend(wave_id, PI_WAVE_MODE_ONE_SHOT);

            // Optional: Wait for finish if you want to ensure sequential commands
            // Or remove this while loop to make it truly non-blocking.
            // Given the ~60ms transmission time, a short wait is usually fine.
            while (gpioWaveTxBusy()) {
                rclcpp::sleep_for(std::chrono::milliseconds(5));
            }
            
            // Clean up the wave ID
            gpioWaveDelete(wave_id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pigpio wave!");
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRNode>());
    rclcpp::shutdown();
    return 0;
}