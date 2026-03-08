// Name: Michael Girgis
// Date: 3/7/2026
// Description: VEML7700 Ambient Light Sensor Library


#include "hardware_team_robot/sensors/VEML7700.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

extern "C" {
    #include <i2c/smbus.h>
}

VEML7700::VEML7700(int i2c_bus, uint8_t address)
    : i2c_fd_(-1)
    , i2c_address_(address)
    , initialized_(false)
    , current_gain_(VEML7700_ALS_GAIN_1_8)
    , current_it_(VEML7700_ALS_IT_25MS)
    , last_error_("") {
    
    // Open I2C bus
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", i2c_bus);
    i2c_fd_ = open(filename, O_RDWR);
    
    if (i2c_fd_ < 0) {
        last_error_ = "Failed to open I2C bus: " + std::string(strerror(errno));
        return;
    }
    
    // Set I2C slave address
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
        last_error_ = "Failed to set I2C address: " + std::string(strerror(errno));
        close(i2c_fd_);
        i2c_fd_ = -1;
        return;
    }
}

VEML7700::~VEML7700() {
    if (i2c_fd_ >= 0) {
        powerDown();  // Put sensor to sleep before closing
        close(i2c_fd_);
    }
}

bool VEML7700::begin(uint16_t gain, uint16_t integration_time) {
    if (i2c_fd_ < 0) {
        last_error_ = "I2C bus not opened";
        return false;
    }
    
    // Verify device ID
    if (!verifyDeviceID()) {
        last_error_ = "Device ID verification failed - not a VEML7700 or communication error";
        return false;
    }
    
    current_gain_ = gain;
    current_it_ = integration_time;
    
    // Configure ALS: Gain + Integration Time + Power On
    // Reserved bits 15:13 = 000, bit 10 = 0, bits 5:4 = 00 (persistence), 
    // bits 3:2 = 00, bit 1 = 0 (INT disabled)
    uint16_t config = gain | integration_time | VEML7700_ALS_POWER_ON;
    
    if (!writeRegister(VEML7700_REG_ALS_CONF, config)) {
        last_error_ = "Failed to configure sensor";
        return false;
    }
    
    // Wait for sensor to stabilize based on integration time
    // Add buffer to ensure at least one complete measurement cycle
    int wait_ms = 35;  // Default for 25ms IT
    if (integration_time == VEML7700_ALS_IT_50MS) wait_ms = 60;
    else if (integration_time == VEML7700_ALS_IT_100MS) wait_ms = 110;
    else if (integration_time == VEML7700_ALS_IT_200MS) wait_ms = 210;
    else if (integration_time == VEML7700_ALS_IT_400MS) wait_ms = 410;
    else if (integration_time == VEML7700_ALS_IT_800MS) wait_ms = 810;
    
    usleep(wait_ms * 1000);
    
    initialized_ = true;
    last_error_ = "";
    return true;
}

bool VEML7700::verifyDeviceID() {
    uint16_t device_id = 0;
    if (!readRegister(VEML7700_REG_ID, &device_id)) {
        return false;
    }
    
    // Check low byte for device ID (should be 0x81 for VEML7700)
    uint8_t id_low = device_id & 0xFF;
    if (id_low != 0x81) {
        return false;
    }
    
    return true;
}

bool VEML7700::readALS(uint16_t& value) {
    return readRegister(VEML7700_REG_ALS, &value);
}

bool VEML7700::readWhite(uint16_t& value) {
    return readRegister(VEML7700_REG_WHITE, &value);
}


float VEML7700::readLux() {
    uint16_t raw_als = readALS();
    return calculateLux(raw_als);
}

void VEML7700::powerDown() {
    uint16_t config = current_gain_ | current_it_ | VEML7700_ALS_SHUTDOWN;
    writeRegister(VEML7700_REG_ALS_CONF, config);
}

void VEML7700::powerUp() {
    uint16_t config = current_gain_ | current_it_ | VEML7700_ALS_POWER_ON;
    writeRegister(VEML7700_REG_ALS_CONF, config);
}

bool VEML7700::writeRegister(uint8_t reg, uint16_t value) {
    // VEML7700 expects little-endian 16-bit writes (LSB first)
    // Linux i2c_smbus_write_word_data handles this automatically
    int result = i2c_smbus_write_word_data(i2c_fd_, reg, value);
    
    if (result < 0) {
        last_error_ = "I2C write failed: " + std::string(strerror(errno));
        return false;
    }
    return true;
}

bool VEML7700::readRegister(uint8_t reg, uint16_t* value) {
    int result = i2c_smbus_read_word_data(i2c_fd_, reg);
    
    if (result < 0) {
        last_error_ = "I2C read failed: " + std::string(strerror(errno));
        return false;
    }
    
    *value = static_cast<uint16_t>(result);
    return true;
}

float VEML7700::calculateLux(uint16_t raw_als) {
    float resolution = getResolution();
    return static_cast<float>(raw_als) * resolution;
}

float VEML7700::getResolution() {
    // Resolution lookup table from VEML7700 datasheet Table on page 9
    // Values are measured and documented by Vishay for Gain x2
    // Other gains are calculated proportionally based on gain ratios
    
    float base_resolution = 0.2688f;  // Default fallback (gain x1/8, IT 100ms)
    
    // Gain x2 values from datasheet (page 9)
    if (current_gain_ == VEML7700_ALS_GAIN_2) {
        if (current_it_ == VEML7700_ALS_IT_800MS) return 0.0042f;
        if (current_it_ == VEML7700_ALS_IT_400MS) return 0.0084f;
        if (current_it_ == VEML7700_ALS_IT_200MS) return 0.0168f;
        if (current_it_ == VEML7700_ALS_IT_100MS) return 0.0336f;
        if (current_it_ == VEML7700_ALS_IT_50MS) return 0.0672f;
        if (current_it_ == VEML7700_ALS_IT_25MS) return 0.1344f;
    }
    
    // Gain x1 (multiply gain x2 values by 2)
    if (current_gain_ == VEML7700_ALS_GAIN_1) {
        if (current_it_ == VEML7700_ALS_IT_800MS) return 0.0084f;
        if (current_it_ == VEML7700_ALS_IT_400MS) return 0.0168f;
        if (current_it_ == VEML7700_ALS_IT_200MS) return 0.0336f;
        if (current_it_ == VEML7700_ALS_IT_100MS) return 0.0672f;
        if (current_it_ == VEML7700_ALS_IT_50MS) return 0.1344f;
        if (current_it_ == VEML7700_ALS_IT_25MS) return 0.2688f;
    }
    
     //Gain x1/4 is 16x the gain x2 values
    if (current_gain_ == VEML7700_ALS_GAIN_1_8) {
        if (current_it_ == VEML7700_ALS_IT_800MS) return 0.0672f;   // 0.0042 * 16
        if (current_it_ == VEML7700_ALS_IT_400MS) return 0.1344f;   // 0.0084 * 16
        if (current_it_ == VEML7700_ALS_IT_200MS) return 0.2688f;   // 0.0168 * 16
        if (current_it_ == VEML7700_ALS_IT_100MS) return 0.5376f;   // 0.0336 * 16
        if (current_it_ == VEML7700_ALS_IT_50MS) return 1.0752f;    // 0.0672 * 16
        if (current_it_ == VEML7700_ALS_IT_25MS) return 2.1504f;    // 0.1344 * 16
    }
    
    //Gain x1/4 is 8x the gain x2 values
    if (current_gain_ == VEML7700_ALS_GAIN_1_4) {
        if (current_it_ == VEML7700_ALS_IT_800MS) return 0.0336f;   // 0.0042 * 8
        if (current_it_ == VEML7700_ALS_IT_400MS) return 0.0672f;   // 0.0084 * 8
        if (current_it_ == VEML7700_ALS_IT_200MS) return 0.1344f;   // 0.0168 * 8
        if (current_it_ == VEML7700_ALS_IT_100MS) return 0.2688f;   // 0.0336 * 8
        if (current_it_ == VEML7700_ALS_IT_50MS) return 0.5376f;    // 0.0672 * 8
        if (current_it_ == VEML7700_ALS_IT_25MS) return 1.0752f;    // 0.1344 * 8
    }
    
    // Fallback
    return base_resolution;
}