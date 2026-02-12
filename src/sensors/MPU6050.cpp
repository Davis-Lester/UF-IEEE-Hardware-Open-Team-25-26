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

#include "hardware_team_robot/sensors/MPU6050.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cstdio>
#include <cerrno>

MPU6050::MPU6050(int i2c_bus)
    : fd_(-1)
    , i2c_bus_(i2c_bus)
    , accel_offset_x_(0)
    , accel_offset_y_(0)
    , accel_offset_z_(0)
    , gyro_offset_x_(0)
    , gyro_offset_y_(0)
    , gyro_offset_z_(0)
{
}

MPU6050::~MPU6050()
{
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

int MPU6050::initialize()
{
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    // Open I2C device
    char path[64];
    snprintf(path, sizeof(path), "/dev/i2c-%d", i2c_bus_);
    
    fd_ = open(path, O_RDWR);
    if (fd_ < 0) {
        return -1;
    }
    
    // Set I2C slave address to 0x68
    // IMPORTANT: AD0 pin must be connected to GND for address 0x68
    if (ioctl(fd_, I2C_SLAVE, MPU6050_ADDR) < 0) {
        close(fd_);
        fd_ = -1;
        return -1;
    }
    
    // Verify device identity (readRegister is called, which needs lock)
    uint8_t who_am_i_value;
    if (readRegister(WHO_AM_I, &who_am_i_value) != 0) {
        close(fd_);
        fd_ = -1;
        return -1;
    }
    if (who_am_i_value != 0x68) {
        close(fd_);
        fd_ = -1;
        return -1;
    }
    
    // Wake up MPU6050 (write 0x00 to PWR_MGMT_1)
    // This clears the sleep bit and selects internal 8MHz oscillator as clock source
    if (writeRegister(PWR_MGMT_1, 0x00) != 0) {
        close(fd_);
        fd_ = -1;
        return -1;
    }
    
    // Set sample rate divider to 19 for ~50Hz
    // Sample rate = 1kHz / (1 + SMPLRT_DIV) = 1000 / (1 + 19) = 50 Hz
    if (writeRegister(SMPLRT_DIV, 19) != 0) {
        close(fd_);
        fd_ = -1;
        return -1;
    }
    
    // Configure DLPF (CONFIG register = 0x02)
    // DLPF_CFG = 2 provides ~50Hz bandwidth
    if (writeRegister(CONFIG, DLPF_CFG_2) != 0) {
        close(fd_);
        fd_ = -1;
        return -1;
    }
    
    // Set gyro range to ±500°/s (GYRO_CONFIG = 0x08)
    // FS_SEL = 1 (bits 4:3 = 01) sets ±500°/s range
    if (writeRegister(GYRO_CONFIG, FS_SEL_500) != 0) {
        close(fd_);
        fd_ = -1;
        return -1;
    }
    
    // Set accel range to ±4g (ACCEL_CONFIG = 0x08)
    // AFS_SEL = 1 (bits 4:3 = 01) sets ±4g range
    if (writeRegister(ACCEL_CONFIG, AFS_SEL_4G) != 0) {
        close(fd_);
        fd_ = -1;
        return -1;
    }
    
    // Wait 100ms for sensor to stabilize
    usleep(100000);
    
    return 0;
}

int MPU6050::calibrate(int samples)
{
    if (fd_ < 0) {
        return -1;
    }
    
    if (samples < 1) {
        samples = 1000;
    }
    
    // Initialize accumulators
    int64_t accel_sum_x = 0;
    int64_t accel_sum_y = 0;
    int64_t accel_sum_z = 0;
    int64_t gyro_sum_x = 0;
    int64_t gyro_sum_y = 0;
    int64_t gyro_sum_z = 0;
    
    // Take samples with sensor at rest on a flat surface
    // IMPORTANT: Read raw register values directly (without offset subtraction)
    for (int i = 0; i < samples; ++i) {
        uint8_t accel_buffer[6];
        uint8_t gyro_buffer[6];
        
        {
            std::lock_guard<std::mutex> lock(i2c_mutex_);
            
            // Read raw accelerometer registers directly
            if (readRegisters(ACCEL_XOUT_H, accel_buffer, 6) != 0) {
                return -1;
            }
            
            // Read raw gyroscope registers directly
            if (readRegisters(GYRO_XOUT_H, gyro_buffer, 6) != 0) {
                return -1;
            }
        }
        
        // Combine bytes to get raw 16-bit values (no offset applied)
        int16_t ax_raw = combineBytes(accel_buffer[0], accel_buffer[1]);
        int16_t ay_raw = combineBytes(accel_buffer[2], accel_buffer[3]);
        int16_t az_raw = combineBytes(accel_buffer[4], accel_buffer[5]);
        int16_t gx_raw = combineBytes(gyro_buffer[0], gyro_buffer[1]);
        int16_t gy_raw = combineBytes(gyro_buffer[2], gyro_buffer[3]);
        int16_t gz_raw = combineBytes(gyro_buffer[4], gyro_buffer[5]);
        
        accel_sum_x += ax_raw;
        accel_sum_y += ay_raw;
        accel_sum_z += az_raw;
        gyro_sum_x += gx_raw;
        gyro_sum_y += gy_raw;
        gyro_sum_z += gz_raw;
        
        // Small delay between samples
        usleep(2000); // 2ms delay
    }
    
    // Calculate average raw values
    int16_t accel_avg_x = static_cast<int16_t>(accel_sum_x / samples);
    int16_t accel_avg_y = static_cast<int16_t>(accel_sum_y / samples);
    int16_t accel_avg_z = static_cast<int16_t>(accel_sum_z / samples);
    int16_t gyro_avg_x = static_cast<int16_t>(gyro_sum_x / samples);
    int16_t gyro_avg_y = static_cast<int16_t>(gyro_sum_y / samples);
    int16_t gyro_avg_z = static_cast<int16_t>(gyro_sum_z / samples);
    
    // Calculate offsets (thread-safe update)
    // For accelerometer: X and Y should read 0g when level, Z should read +1g
    // For ±4g range: +1g = +8192 LSB
    // So Z offset = average - 8192 (to make calibrated Z read +1g)
    {
        std::lock_guard<std::mutex> lock(i2c_mutex_);
        accel_offset_x_ = accel_avg_x;  // Zero out X (should read 0g)
        accel_offset_y_ = accel_avg_y;  // Zero out Y (should read 0g)
        accel_offset_z_ = accel_avg_z - static_cast<int16_t>(ACCEL_SENSITIVITY_4G);  // Z should read +1g
        gyro_offset_x_ = gyro_avg_x;    // Zero out X (at rest, no rotation)
        gyro_offset_y_ = gyro_avg_y;    // Zero out Y (at rest, no rotation)
        gyro_offset_z_ = gyro_avg_z;    // Zero out Z (at rest, no rotation)
    }
    
    return 0;
}

int MPU6050::readRawAccel(int16_t* x, int16_t* y, int16_t* z)
{
    if (fd_ < 0) {
        return -1;
    }
    
    uint8_t buffer[6];
    int16_t offset_x, offset_y, offset_z;
    
    {
        std::lock_guard<std::mutex> lock(i2c_mutex_);
        
        if (readRegisters(ACCEL_XOUT_H, buffer, 6) != 0) {
            return -1;
        }
        
        offset_x = accel_offset_x_;
        offset_y = accel_offset_y_;
        offset_z = accel_offset_z_;
    }
    
    if (x) *x = combineBytes(buffer[0], buffer[1]) - offset_x;
    if (y) *y = combineBytes(buffer[2], buffer[3]) - offset_y;
    if (z) *z = combineBytes(buffer[4], buffer[5]) - offset_z;
    
    return 0;
}

int MPU6050::readRawGyro(int16_t* x, int16_t* y, int16_t* z)
{
    if (fd_ < 0) {
        return -1;
    }
    
    uint8_t buffer[6];
    int16_t offset_x, offset_y, offset_z;
    
    {
        std::lock_guard<std::mutex> lock(i2c_mutex_);
        
        if (readRegisters(GYRO_XOUT_H, buffer, 6) != 0) {
            return -1;
        }
        
        offset_x = gyro_offset_x_;
        offset_y = gyro_offset_y_;
        offset_z = gyro_offset_z_;
    }
    
    if (x) *x = combineBytes(buffer[0], buffer[1]) - offset_x;
    if (y) *y = combineBytes(buffer[2], buffer[3]) - offset_y;
    if (z) *z = combineBytes(buffer[4], buffer[5]) - offset_z;
    
    return 0;
}

int MPU6050::readAccel(float* x_g, float* y_g, float* z_g)
{
    int16_t ax, ay, az;
    if (readRawAccel(&ax, &ay, &az) != 0) {
        return -1;
    }
    
    if (x_g) *x_g = static_cast<float>(ax) / ACCEL_SENSITIVITY_4G;
    if (y_g) *y_g = static_cast<float>(ay) / ACCEL_SENSITIVITY_4G;
    if (z_g) *z_g = static_cast<float>(az) / ACCEL_SENSITIVITY_4G;
    
    return 0;
}

int MPU6050::readGyro(float* x_dps, float* y_dps, float* z_dps)
{
    int16_t gx, gy, gz;
    if (readRawGyro(&gx, &gy, &gz) != 0) {
        return -1;
    }
    
    if (x_dps) *x_dps = static_cast<float>(gx) / GYRO_SENSITIVITY_500DPS;
    if (y_dps) *y_dps = static_cast<float>(gy) / GYRO_SENSITIVITY_500DPS;
    if (z_dps) *z_dps = static_cast<float>(gz) / GYRO_SENSITIVITY_500DPS;
    
    return 0;
}

int MPU6050::readAll(float* ax_g, float* ay_g, float* az_g,
                     float* gx_dps, float* gy_dps, float* gz_dps)
{
    if (fd_ < 0) {
        return -1;
    }
    
    uint8_t buffer[14];
    int16_t accel_off_x, accel_off_y, accel_off_z;
    int16_t gyro_off_x, gyro_off_y, gyro_off_z;
    
    {
        std::lock_guard<std::mutex> lock(i2c_mutex_);
        
        // Read all 14 bytes in one burst: accel (6) + temp (2, skipped) + gyro (6)
        if (readRegisters(ACCEL_XOUT_H, buffer, 14) != 0) {
            return -1;
        }
        
        // Copy offsets while holding lock
        accel_off_x = accel_offset_x_;
        accel_off_y = accel_offset_y_;
        accel_off_z = accel_offset_z_;
        gyro_off_x = gyro_offset_x_;
        gyro_off_y = gyro_offset_y_;
        gyro_off_z = gyro_offset_z_;
    }
    
    // Parse accelerometer data (bytes 0-5)
    int16_t ax = combineBytes(buffer[0], buffer[1]) - accel_off_x;
    int16_t ay = combineBytes(buffer[2], buffer[3]) - accel_off_y;
    int16_t az = combineBytes(buffer[4], buffer[5]) - accel_off_z;
    
    // Parse gyroscope data (bytes 8-13, skipping temperature bytes at 6-7)
    int16_t gx = combineBytes(buffer[8], buffer[9]) - gyro_off_x;
    int16_t gy = combineBytes(buffer[10], buffer[11]) - gyro_off_y;
    int16_t gz = combineBytes(buffer[12], buffer[13]) - gyro_off_z;
    
    // Convert to physical units
    if (ax_g) *ax_g = static_cast<float>(ax) / ACCEL_SENSITIVITY_4G;
    if (ay_g) *ay_g = static_cast<float>(ay) / ACCEL_SENSITIVITY_4G;
    if (az_g) *az_g = static_cast<float>(az) / ACCEL_SENSITIVITY_4G;
    
    if (gx_dps) *gx_dps = static_cast<float>(gx) / GYRO_SENSITIVITY_500DPS;
    if (gy_dps) *gy_dps = static_cast<float>(gy) / GYRO_SENSITIVITY_500DPS;
    if (gz_dps) *gz_dps = static_cast<float>(gz) / GYRO_SENSITIVITY_500DPS;
    
    return 0;
}

bool MPU6050::testConnection()
{
    return (whoAmI() == 0x68);
}

int MPU6050::whoAmI()
{
    if (fd_ < 0) {
        return -1;
    }
    
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    uint8_t value;
    if (readRegister(WHO_AM_I, &value) != 0) {
        return -1;
    }
    
    return static_cast<int>(value);
}

int MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
    if (fd_ < 0) {
        return -1;
    }
    
    uint8_t buffer[2] = {reg, value};
    if (write(fd_, buffer, 2) != 2) {
        return -1;
    }
    
    return 0;
}

int MPU6050::readRegister(uint8_t reg, uint8_t* value)
{
    if (fd_ < 0 || value == nullptr) {
        return -1;
    }
    
    if (write(fd_, &reg, 1) != 1) {
        return -1;
    }
    
    if (read(fd_, value, 1) != 1) {
        return -1;
    }
    
    return 0;
}

int MPU6050::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length)
{
    if (fd_ < 0 || buffer == nullptr) {
        return -1;
    }
    
    // Write register address
    if (write(fd_, &reg, 1) != 1) {
        return -1;
    }
    
    // Read data
    if (read(fd_, buffer, length) != length) {
        return -1;
    }
    
    return 0;
}

int16_t MPU6050::combineBytes(uint8_t hi, uint8_t lo)
{
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

