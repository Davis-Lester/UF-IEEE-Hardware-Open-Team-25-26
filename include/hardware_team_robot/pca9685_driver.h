#ifndef HARDWARE_TEAM_ROBOT_PCA9685_DRIVER_H
#define HARDWARE_TEAM_ROBOT_PCA9685_DRIVER_H

#include <cstdint>
#include <string>

namespace Hardware {

class PCA9685Driver {
public:
    // Motor IDs matching your wiring diagram
    enum Motor {
        MOTOR_1 = 0,  // LED0/LED1
        MOTOR_2 = 1,  // LED2/LED3
        MOTOR_3 = 2,  // LED4/LED5
        MOTOR_4 = 3,  // LED6/LED7
        MOTOR_6 = 4,  // LED8/LED9  (Note: M6 uses channels 8/9)
        MOTOR_5 = 5   // LED10/LED11
    };

    PCA9685Driver(uint8_t i2c_bus = 1, uint8_t i2c_addr = 0x40);
    ~PCA9685Driver();

    // Initialize PCA9685 with optimal settings for DRV8871
    bool initialize();

    // Motor control (-100 to +100, 0 = stop)
    void setMotorSpeed(Motor motor, int8_t speed);
    
    // Stop specific motor
    void stopMotor(Motor motor);
    
    // Emergency stop all motors
    void stopAll();

    // Low-level PWM control (0-4095)
    void setPWM(uint8_t channel, uint16_t on_time, uint16_t off_time);
    
    // Get last error message
    std::string getLastError() const { return last_error_; }

private:
    int i2c_fd_;
    uint8_t i2c_bus_;
    uint8_t i2c_addr_;
    std::string last_error_;

    // Register addresses from datasheet [1]
    static constexpr uint8_t REG_MODE1 = 0x00;
    static constexpr uint8_t REG_MODE2 = 0x01;
    static constexpr uint8_t REG_PRESCALE = 0xFE;
    static constexpr uint8_t REG_LED0_ON_L = 0x06;
    
    // Mode bits from datasheet [1]
    static constexpr uint8_t MODE1_SLEEP = 0x10;
    static constexpr uint8_t MODE1_AI = 0x20;      // Auto-increment
    static constexpr uint8_t MODE1_RESTART = 0x80;
    
    static constexpr uint8_t MODE2_OUTDRV = 0x04;  // Totem-pole output [1]
    
    // I2C helpers
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    
    // Get PWM register base address for channel
    uint8_t getChannelBase(uint8_t channel) const {
        return REG_LED0_ON_L + (4 * channel);
    }
    
    // Motor channel mapping (matches your wiring diagram)
    void getMotorChannels(Motor motor, uint8_t& fwd_ch, uint8_t& rev_ch) const;
};

} // namespace Hardware

#endif // HARDWARE_TEAM_ROBOT_PCA9685_DRIVER_H