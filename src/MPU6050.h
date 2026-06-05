#ifndef MPU6050_COMPLEMENTARY_H
#define MPU6050_COMPLEMENTARY_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MPU6050Complementary {
public:
    struct RawData {
        int16_t ax;
        int16_t ay;
        int16_t az;
        int16_t gx;
        int16_t gy;
        int16_t gz;
    };

    struct ProcessedData {
        // Accelerometer values in g
        float ax;
        float ay;
        float az;

        // Gyroscope values in deg/s
        float gx;
        float gy;
        float gz;
    };

    struct Angles {
        // Degrees
        float x; // roll-like angle about x-axis
        float y; // pitch-like angle about y-axis
        float z; // yaw angle, gyro-only
    };

    explicit MPU6050Complementary(TwoWire &wire = Wire, uint8_t address = 0x68);

    bool begin(int sdaPin = -1,
               int sclPin = -1,
               uint32_t i2cClockHz = 400000UL);

    bool calibrate(uint16_t durationMs = 500,
                   uint16_t sampleDelayMs = 5);

    bool update();

    bool readRawData(RawData &raw);
    bool readProcessedData(ProcessedData &data);

    Angles getAngles() const;
    ProcessedData getLastProcessedData() const;

    void resetAngles();

    void setComplementaryAlpha(float alpha);
    void setTau(float alpha);

private:
    static constexpr uint8_t MPU_REG_WHO_AM_I     = 0x75;
    static constexpr uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t MPU_REG_CONFIG       = 0x1A;
    static constexpr uint8_t MPU_REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;

    static constexpr float RAD_TO_DEG_F = 57.2957795131f;

    // These match:
    // Gyro ±250 deg/s  -> 131 LSB/(deg/s)
    // Accel ±2g        -> 16384 LSB/g
    static constexpr float GYRO_SCALE_250DPS = 131.0f;
    static constexpr float ACCEL_SCALE_2G    = 16384.0f;

    struct Axis3f {
        float x;
        float y;
        float z;
    };

    TwoWire *_wire;
    uint8_t _address;

    float _gyroScaleFactor;
    float _accelScaleFactor;

    float _alpha;

    Axis3f _gyroBiasRaw;
    Axis3f _accelAngleOffsetDeg;

    Angles _angles;
    ProcessedData _lastProcessedData;

    uint32_t _lastUpdateMicros;
    bool _initialized;
    bool _calibrated;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length);

    static float computeAccelAngleX(float ax_g, float ay_g, float az_g);
    static float computeAccelAngleY(float ax_g, float ay_g, float az_g);
};

#endif