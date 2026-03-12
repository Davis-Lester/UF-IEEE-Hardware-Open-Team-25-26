#include "hardware_team_robot/pca9685_driver.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstring>

namespace Hardware {

PCA9685Driver::PCA9685Driver(uint8_t i2c_bus, uint8_t i2c_addr)
    : i2c_fd_(-1), i2c_bus_(i2c_bus), i2c_addr_(i2c_addr) {}

PCA9685Driver::~PCA9685Driver() {
    if (i2c_fd_ >= 0) {
        stopAll();
        close(i2c_fd_);
    }
}

bool PCA9685Driver::initialize() {
    // Open I2C bus
    char filename[20];
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", i2c_bus_);
    
    i2c_fd_ = open(filename, O_RDWR);
    if (i2c_fd_ < 0) {
        last_error_ = "Failed to open I2C bus";
        return false;
    }

    // Set I2C slave address [1]
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        last_error_ = "Failed to set I2C address";
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // Reset device (set to sleep mode first) [1]
    if (!writeRegister(REG_MODE1, MODE1_SLEEP)) {
        last_error_ = "Failed to enter sleep mode";
        return false;
    }
    usleep(500); // Wait for oscillator to stop [1]

    // Set PWM frequency to 1000 Hz (recommended for DC motors)
    // Formula from datasheet [1]: prescale = round(25MHz / (4096 * freq)) - 1
    uint8_t prescale = static_cast<uint8_t>(std::round(25000000.0 / (4096.0 * 1000.0)) - 1);
    
    if (!writeRegister(REG_PRESCALE, prescale)) {
        last_error_ = "Failed to set PWM frequency";
        return false;
    }

    // Configure MODE2: Totem-pole outputs for DRV8871 [1]
    if (!writeRegister(REG_MODE2, MODE2_OUTDRV)) {
        last_error_ = "Failed to set output mode";
        return false;
    }

    // Wake up and enable auto-increment [1]
    if (!writeRegister(REG_MODE1, MODE1_AI)) {
        last_error_ = "Failed to wake up device";
        return false;
    }
    usleep(500); // Wait for oscillator to stabilize [1]

    // Clear restart bit if set [1]
    uint8_t mode1;
    if (readRegister(REG_MODE1, mode1) && (mode1 & MODE1_RESTART)) {
        if (!writeRegister(REG_MODE1, MODE1_AI | MODE1_RESTART)) {
            last_error_ = "Failed to clear restart";
            return false;
        }
    }

    // Initialize all channels to off
    stopAll();

    return true;
}

void PCA9685Driver::getMotorChannels(Motor motor, uint8_t& fwd_ch, uint8_t& rev_ch) const {
    // Map motors to PCA9685 channels based on your wiring diagram
    switch (motor) {
        case MOTOR_1: fwd_ch = 0;  rev_ch = 1;  break;  // M1: LED0/LED1
        case MOTOR_2: fwd_ch = 2;  rev_ch = 3;  break;  // M2: LED2/LED3
        case MOTOR_3: fwd_ch = 4;  rev_ch = 5;  break;  // M3: LED4/LED5
        case MOTOR_4: fwd_ch = 6;  rev_ch = 7;  break;  // M4: LED6/LED7
        case MOTOR_6: fwd_ch = 8;  rev_ch = 9;  break;  // M6: LED8/LED9
        case MOTOR_5: fwd_ch = 10; rev_ch = 11; break;  // M5: LED10/LED11
        default: fwd_ch = 0; rev_ch = 0; break;
    }
}

void PCA9685Driver::setMotorSpeed(Motor motor, int8_t speed) {
    if (i2c_fd_ < 0) return;

    // Clamp speed to [-100, 100]
    speed = std::max(static_cast<int8_t>(-100), std::min(speed, static_cast<int8_t>(100)));

    uint8_t fwd_ch, rev_ch;
    getMotorChannels(motor, fwd_ch, rev_ch);

    // Convert speed (-100 to +100) to PWM duty cycle (0-4095)
    uint16_t pwm_value = static_cast<uint16_t>((std::abs(speed) * 4095) / 100);

    if (speed > 0) {
        // Forward: FWD channel active, REV off [1]
        setPWM(fwd_ch, 0, pwm_value);
        setPWM(rev_ch, 0, 0);
    } else if (speed < 0) {
        // Reverse: REV channel active, FWD off [1]
        setPWM(fwd_ch, 0, 0);
        setPWM(rev_ch, 0, pwm_value);
    } else {
        // Stop: Both channels off (coast mode for DRV8871)
        setPWM(fwd_ch, 0, 0);
        setPWM(rev_ch, 0, 0);
    }
}

void PCA9685Driver::stopMotor(Motor motor) {
    setMotorSpeed(motor, 0);
}

void PCA9685Driver::stopAll() {
    if (i2c_fd_ < 0) return;

    // Turn off all 16 channels [1]
    for (uint8_t ch = 0; ch < 16; ch++) {
        setPWM(ch, 0, 0);
    }
}

void PCA9685Driver::setPWM(uint8_t channel, uint16_t on_time, uint16_t off_time) {
    if (i2c_fd_ < 0 || channel > 15) return;

    uint8_t base = getChannelBase(channel);
    
    // Write 4 bytes: ON_L, ON_H, OFF_L, OFF_H [1]
    uint8_t data[4] = {
        static_cast<uint8_t>(on_time & 0xFF),        // ON_L
        static_cast<uint8_t>((on_time >> 8) & 0x0F), // ON_H (only 4 bits)
        static_cast<uint8_t>(off_time & 0xFF),       // OFF_L
        static_cast<uint8_t>((off_time >> 8) & 0x0F) // OFF_H (only 4 bits)
    };

    // Write all 4 registers at once using auto-increment [1]
    uint8_t buffer[5] = {base, data[0], data[1], data[2], data[3]};
    if (write(i2c_fd_, buffer, sizeof(buffer)) != sizeof(buffer)) {
        last_error_ = "Failed to write PWM values";
    }
}

bool PCA9685Driver::writeRegister(uint8_t reg, uint8_t value) {
    if (i2c_fd_ < 0) return false;

    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd_, buffer, 2) != 2) {
        last_error_ = std::string("Failed to write register 0x") + 
                      std::to_string(reg);
        return false;
    }
    return true;
}

bool PCA9685Driver::readRegister(uint8_t reg, uint8_t& value) {
    if (i2c_fd_ < 0) return false;

    if (write(i2c_fd_, &reg, 1) != 1) {
        last_error_ = "Failed to write register address";
        return false;
    }

    if (read(i2c_fd_, &value, 1) != 1) {
        last_error_ = "Failed to read register value";
        return false;
    }

    return true;
}

} // namespace Hardware