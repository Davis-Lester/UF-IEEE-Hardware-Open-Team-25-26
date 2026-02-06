#ifndef MPU6050_H
#define MPU6050_H

// Name: Michael Girgis
// Date: 2/5/2026
// Description: Creating an IMU Library for ROS2 Robot

//╭━━━╮╱╱╱╱╱╱╱╱╭╮╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭╮╱╱╱╱╱╱╭╮╱╱╱╭╮
//┃╭━╮┃╱╱╱╱╱╱╱╱┃┃╱╱╱╱╱╱╱╱╱╱╱╱╱╭╯╰╮╱╱╱╱╱┃┃╱╱╱┃┃
//┃┃╱┃┣━━┳━━┳━━┫┃╭━━┳━┳━━┳╮╭┳━┻╮╭╋━━┳━╮┃┃╱╱╭┫╰━┳━┳━━┳━┳╮╱╭╮
//┃╰━╯┃╭━┫╭━┫┃━┫┃┃┃━┫╭┫╭╮┃╰╯┃┃━┫┃┃┃━┫╭╯┃┃╱╭╋┫╭╮┃╭┫╭╮┃╭┫┃╱┃┃
//┃╭━╮┃╰━┫╰━┫┃━┫╰┫┃━┫┃┃╰╯┃┃┃┃┃━┫╰┫┃━┫┃╱┃╰━╯┃┃╰╯┃┃┃╭╮┃┃┃╰━╯┃
//╰╯╱╰┻━━┻━━┻━━┻━┻━━┻╯╰━━┻┻┻┻━━┻━┻━━┻╯╱╰━━━┻┻━━┻╯╰╯╰┻╯╰━╮╭╯
//╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭━╯┃
//╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╰━━╯

#include <cstdint>
#include <mutex>

// MPU6050 IMU Driver for Raspberry Pi 4
// IMPORTANT: Connect AD0 pin to GND for I2C address 0x68

class MPU6050 {
public:
    // Constructor - default uses /dev/i2c-1
    explicit MPU6050(int i2c_bus = 1);
    ~MPU6050();
    
    // Initialize and calibrate
    int initialize();                    // Returns 0 on success, -1 on error
    int calibrate(int samples = 1000);   // Calibrate with sensor at rest on flat surface
                                          // X/Y axes will read ~0g, Z-axis will read ~+1g
    
    // Raw sensor readings (16-bit values from registers)
    int readRawAccel(int16_t* x, int16_t* y, int16_t* z);
    int readRawGyro(int16_t* x, int16_t* y, int16_t* z);
    
    // Converted readings (real units: g's)
    int readAccel(float* x_g, float* y_g, float* z_g);
    int readGyro(float* x_dps, float* y_dps, float* z_dps);
    
    // Read everything in one efficient burst
    int readAll(float* ax_g, float* ay_g, float* az_g,
                float* gx_dps, float* gy_dps, float* gz_dps);
    
    // Utility functions
    bool testConnection();
    int whoAmI();  // Should return 0x68

private:
    int fd_;                    // I2C device file descriptor
    int i2c_bus_;              // I2C bus number
    mutable std::mutex i2c_mutex_;  // Thread-safe I2C access
    
    // Calibration offsets
    int16_t accel_offset_x_, accel_offset_y_, accel_offset_z_;
    int16_t gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;
    
    // Helper methods
    int writeRegister(uint8_t reg, uint8_t value);
    int readRegister(uint8_t reg, uint8_t* value);
    int readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
    int16_t combineBytes(uint8_t hi, uint8_t lo);
    
    // MPU6050 Register Addresses
    static constexpr uint8_t MPU6050_ADDR = 0x68;
    static constexpr uint8_t PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t SMPLRT_DIV = 0x19;
    static constexpr uint8_t CONFIG = 0x1A;
    static constexpr uint8_t GYRO_CONFIG = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t GYRO_XOUT_H = 0x43;
    static constexpr uint8_t WHO_AM_I = 0x75;
    
    // Configuration values
    static constexpr uint8_t DLPF_CFG_2 = 0x02;     // ~50Hz bandwidth
    static constexpr uint8_t AFS_SEL_4G = 0x08;     // ±4g range
    static constexpr uint8_t FS_SEL_500 = 0x08;     // ±500°/s range
    
    // Conversion factors (from datasheet)
    static constexpr float ACCEL_SENSITIVITY_4G = 8192.0f;
    static constexpr float GYRO_SENSITIVITY_500DPS = 65.5f;
};

#endif // MPU6050_H