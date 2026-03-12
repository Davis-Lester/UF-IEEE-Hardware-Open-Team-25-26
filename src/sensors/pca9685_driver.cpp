#include "hardware_team_robot/sensors/pca9685_driver.h"
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

void PCA9685Driver::cleanupOnFailure() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool PCA9685Driver::initialize() {
    char filename[20];
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", i2c_bus_);
    
    i2c_fd_ = open(filename, O_RDWR);
    if (i2c_fd_ < 0) {
        last_error_ = "Failed to open I2C bus";
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        last_error_ = "Failed to set I2C address";
        cleanupOnFailure();
        return false;
    }

    if (!writeRegister(REG_MODE1, MODE1_SLEEP)) {
        last_error_ = "Failed to enter sleep mode";
        cleanupOnFailure();
        return false;
    }
    usleep(500);

    uint8_t prescale = static_cast<uint8_t>(std::round(25000000.0 / (4096.0 * 1000.0)) - 1);
    
    if (!writeRegister(REG_PRESCALE, prescale)) {
        last_error_ = "Failed to set PWM frequency";
        cleanupOnFailure();
        return false;
    }

    if (!writeRegister(REG_MODE2, MODE2_OUTDRV)) {
        last_error_ = "Failed to set output mode";
        cleanupOnFailure();
        return false;
    }

    if (!writeRegister(REG_MODE1, MODE1_AI)) {
        last_error_ = "Failed to wake up device";
        cleanupOnFailure();
        return false;
    }
    usleep(500);

    uint8_t mode1;
    if (!readRegister(REG_MODE1, mode1)) {
        last_error_ = "Failed to read MODE1 register";
        cleanupOnFailure();
        return false;
    }

    if (mode1 & MODE1_RESTART) {
        if (!writeRegister(REG_MODE1, MODE1_AI | MODE1_RESTART)) {
            last_error_ = "Failed to clear restart";
            cleanupOnFailure();
            return false;
        }
    }

    stopAll();
    return true;
}

void PCA9685Driver::getMotorChannels(Motor motor, uint8_t& fwd_ch, uint8_t& rev_ch) {
    switch (motor) {
        case MOTOR_1: fwd_ch = 0;  rev_ch = 1;  break;
        case MOTOR_2: fwd_ch = 2;  rev_ch = 3;  break;
        case MOTOR_3: fwd_ch = 4;  rev_ch = 5;  break;
        case MOTOR_4: fwd_ch = 6;  rev_ch = 7;  break;
        case MOTOR_6: fwd_ch = 8;  rev_ch = 9;  break;
        case MOTOR_5: fwd_ch = 10; rev_ch = 11; break;
        default: 
            // Invalid motor ID - set to sentinel values
            fwd_ch = 0xFF; 
            rev_ch = 0xFF;
            last_error_ = "Invalid motor ID in getMotorChannels";
            break;
    }
}

void PCA9685Driver::setMotorSpeed(Motor motor, int8_t speed) {
    if (i2c_fd_ < 0) return;

    speed = std::max(static_cast<int8_t>(-100), std::min(speed, static_cast<int8_t>(100)));

    uint8_t fwd_ch, rev_ch;
    getMotorChannels(motor, fwd_ch, rev_ch);

    uint16_t pwm_value = static_cast<uint16_t>((std::abs(speed) * 4095) / 100);

    // Helper lambda for setting PWM with proper full-on/full-off handling
    auto set_channel = [this](uint8_t ch, uint16_t value) {
        if (value == 0) {
            // Full OFF: use bit 4 of OFF_H register [1]
            setPWM(ch, 0, 4096);
        } else if (value >= 4095) {
            // Full ON: use bit 4 of ON_H register [1]
            setPWM(ch, 4096, 0);
        } else {
            // Normal PWM
            setPWM(ch, 0, value);
        }
    };

    if (speed > 0) {
        set_channel(fwd_ch, pwm_value);
        setPWM(rev_ch, 0, 4096);  // Full OFF [1]
    } else if (speed < 0) {
        setPWM(fwd_ch, 0, 4096);  // Full OFF [1]
        set_channel(rev_ch, pwm_value);
    } else {
        // Both OFF for coast mode
        setPWM(fwd_ch, 0, 4096);
        setPWM(rev_ch, 0, 4096);
    }
}

void PCA9685Driver::stopMotor(Motor motor) {
    setMotorSpeed(motor, 0);
}

void PCA9685Driver::stopAll() {
    if (i2c_fd_ < 0) return;

    // Turn off all 16 channels using proper full-OFF [1]
    for (uint8_t ch = 0; ch < 16; ch++) {
        setPWM(ch, 0, 4096);
    }
}

void PCA9685Driver::setPWM(uint8_t channel, uint16_t on_time, uint16_t off_time) {
    if (i2c_fd_ < 0 || channel > 15 || on_time > 4096 || off_time > 4096) return;

    uint8_t base = getChannelBase(channel);
    
    // Support full-on (4096) and full-off (4096) via bit 4 [1]
    // Bit 4 is at position 0x10 in the high byte
    uint8_t buffer[5] = {
        base,
        static_cast<uint8_t>(on_time & 0xFF),
        static_cast<uint8_t>((on_time >> 8) & 0x1F),  // Allow bit 4 (0x10) for full-ON [1]
        static_cast<uint8_t>(off_time & 0xFF),
        static_cast<uint8_t>((off_time >> 8) & 0x1F)  // Allow bit 4 (0x10) for full-OFF [1]
    };

    if (write(i2c_fd_, buffer, sizeof(buffer)) != sizeof(buffer)) {
        last_error_ = "Failed to write PWM values";
    }
}

bool PCA9685Driver::writeRegister(uint8_t reg, uint8_t value) {
    if (i2c_fd_ < 0) return false;

    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd_, buffer, 2) != 2) {
        last_error_ = std::string("Failed to write register 0x") + std::to_string(reg);
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