// Name: Michael Girgis
// Date: 3/7/2026
// Description: VEML7700 Ambient Light Sensor Library


#ifndef VEML7700_H
#define VEML7700_H

#include <cstdint>
#include <string>

// VEML7700 I2C Address (7-bit: 0x10)
#define VEML7700_I2C_ADDR 0x10

// Register Addresses
#define VEML7700_REG_ALS_CONF    0x00  // Configuration register
#define VEML7700_REG_ALS_WH      0x01  // High threshold window setting
#define VEML7700_REG_ALS_WL      0x02  // Low threshold window setting
#define VEML7700_REG_POWER_SAVE  0x03  // Power saving mode
#define VEML7700_REG_ALS         0x04  // ALS high resolution output data
#define VEML7700_REG_WHITE       0x05  // White channel output data
#define VEML7700_REG_ALS_INT     0x06  // Interrupt status
#define VEML7700_REG_ID          0x07  // Device ID register

// Configuration bit masks (bits 12:11) - ALS_GAIN
#define VEML7700_ALS_GAIN_1      0x0000  // Gain x1
#define VEML7700_ALS_GAIN_2      0x0800  // Gain x2
#define VEML7700_ALS_GAIN_1_8    0x1000  // Gain x1/8
#define VEML7700_ALS_GAIN_1_4    0x1800  // Gain x1/4

// Integration time settings (bits 9:6) - ALS_IT
#define VEML7700_ALS_IT_25MS     0x0300  // Integration time 25ms (bits 9:6 = 1100b)
#define VEML7700_ALS_IT_50MS     0x0200  // Integration time 50ms (bits 9:6 = 1000b)
#define VEML7700_ALS_IT_100MS    0x0000  // Integration time 100ms (bits 9:6 = 0000b)
#define VEML7700_ALS_IT_200MS    0x0040  // Integration time 200ms (bits 9:6 = 0001b)
#define VEML7700_ALS_IT_400MS    0x0080  // Integration time 400ms (bits 9:6 = 0010b)
#define VEML7700_ALS_IT_800MS    0x00C0  // Integration time 800ms (bits 9:6 = 0011b)

// Power control (bit 0) - ALS_SD
#define VEML7700_ALS_POWER_ON    0x0000  // Power on
#define VEML7700_ALS_SHUTDOWN    0x0001  // Shutdown

// Expected Device ID
#define VEML7700_DEVICE_ID       0x0081  // Device ID for slave address 0x10

class VEML7700 {
public:
    /**
     * @brief Constructor
     * @param i2c_bus I2C bus number (typically 1 for /dev/i2c-1 on RPi)
     * @param address I2C device address (default 0x10)
     */
    VEML7700(int i2c_bus = 1, uint8_t address = VEML7700_I2C_ADDR);
    
    /**
     * @brief Destructor - closes I2C file descriptor
     */
    ~VEML7700();
    
    /**
     * @brief Initialize and configure the sensor with device ID verification
     * @param gain Gain setting (use VEML7700_ALS_GAIN_* constants)
     * @param integration_time Integration time (use VEML7700_ALS_IT_* constants)
     * @return true if successful, false otherwise
     */
    bool begin(uint16_t gain = VEML7700_ALS_GAIN_1_8, 
               uint16_t integration_time = VEML7700_ALS_IT_25MS);
    
    /**
     * @brief Read raw ALS value (0-65535)
     * @return Raw ALS count value
     */
    uint16_t readALS();
    
    /**
     * @brief Read raw white channel value (0-65535)
     * RECOMMENDED for LED detection - more sensitive to white LED spikes
     * @return Raw white channel count value
     */
    uint16_t readWhite();
    
    /**
     * @brief Read calculated lux value from ALS channel
     * @return Ambient light in lux
     */
    float readLux();
    
    /**
     * @brief Power down the sensor
     */
    void powerDown();
    
    /**
     * @brief Power up the sensor
     */
    void powerUp();
    
    /**
     * @brief Check if sensor is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const { return last_error_; }
    
    /**
     * @brief Get current gain setting
     * @return Current gain value
     */
    uint16_t getCurrentGain() const { return current_gain_; }
    
    /**
     * @brief Get current integration time setting
     * @return Current integration time value
     */
    uint16_t getCurrentIT() const { return current_it_; }

private:
    int i2c_fd_;                  // I2C file descriptor
    uint8_t i2c_address_;         // I2C device address
    bool initialized_;            // Initialization flag
    uint16_t current_gain_;       // Current gain setting
    uint16_t current_it_;         // Current integration time
    std::string last_error_;      // Last error message
    
    /**
     * @brief Write 16-bit value to register
     * @param reg Register address
     * @param value 16-bit value to write
     * @return true if successful
     */
    bool writeRegister(uint8_t reg, uint16_t value);
    
    /**
     * @brief Read 16-bit value from register
     * @param reg Register address
     * @param value Pointer to store read value
     * @return true if successful
     */
    bool readRegister(uint8_t reg, uint16_t* value);
    
    /**
     * @brief Calculate lux from raw ALS value using datasheet resolution table
     * @param raw_als Raw ALS count
     * @return Calculated lux value
     */
    float calculateLux(uint16_t raw_als);
    
    /**
     * @brief Get resolution for current settings (lux per bit) from datasheet
     * @return Resolution value in lx/bit
     */
    float getResolution();
    
    /**
     * @brief Verify device ID matches expected VEML7700 ID
     * @return true if ID matches, false otherwise
     */
    bool verifyDeviceID();
};

#endif // VEML7700_H