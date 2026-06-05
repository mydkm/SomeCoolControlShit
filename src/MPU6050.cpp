#include "MPU6050Complementary.h"

MPU6050Complementary::MPU6050Complementary(TwoWire &wire, uint8_t address)
    : _wire(&wire),
      _address(address),
      _gyroScaleFactor(GYRO_SCALE_250DPS),
      _accelScaleFactor(ACCEL_SCALE_2G),
      _alpha(0.98f),
      _gyroBiasRaw{0.0f, 0.0f, 0.0f},
      _accelAngleOffsetDeg{0.0f, 0.0f, 0.0f},
      _angles{0.0f, 0.0f, 0.0f},
      _lastProcessedData{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      _lastUpdateMicros(0),
      _initialized(false),
      _calibrated(false) {}

bool MPU6050Complementary::begin(int sdaPin, int sclPin, uint32_t i2cClockHz) {
#if defined(ARDUINO_ARCH_STM32)
    if (sdaPin >= 0) {
        _wire->setSDA(sdaPin);
    }

    if (sclPin >= 0) {
        _wire->setSCL(sclPin);
    }
#else
    (void)sdaPin;
    (void)sclPin;
#endif

    _wire->begin();
    _wire->setClock(i2cClockHz);

    delay(100);

    uint8_t whoAmI = 0;
    if (!readRegisters(MPU_REG_WHO_AM_I, &whoAmI, 1)) {
        return false;
    }

    if (whoAmI != 0x68) {
        return false;
    }

    // Wake the MPU6050. It starts in sleep mode.
    if (!writeRegister(MPU_REG_PWR_MGMT_1, 0x00)) {
        return false;
    }

    delay(100);

    // Digital low-pass filter:
    // 0x03 gives approximately 44 Hz accel bandwidth and 42 Hz gyro bandwidth.
    if (!writeRegister(MPU_REG_CONFIG, 0x03)) {
        return false;
    }

    // Gyroscope full scale: ±250 deg/s.
    if (!writeRegister(MPU_REG_GYRO_CONFIG, 0x00)) {
        return false;
    }

    _gyroScaleFactor = GYRO_SCALE_250DPS;

    // Accelerometer full scale: ±2g.
    if (!writeRegister(MPU_REG_ACCEL_CONFIG, 0x00)) {
        return false;
    }

    _accelScaleFactor = ACCEL_SCALE_2G;

    _initialized = true;

    // Moving-average calibration over the first 0.5 seconds.
    if (!calibrate(500, 5)) {
        return false;
    }

    _lastUpdateMicros = micros();

    return true;
}

bool MPU6050Complementary::calibrate(uint16_t durationMs, uint16_t sampleDelayMs) {
    if (!_initialized) {
        return false;
    }

    if (durationMs == 0) {
        durationMs = 500;
    }

    if (sampleDelayMs == 0) {
        sampleDelayMs = 1;
    }

    int64_t sumAx = 0;
    int64_t sumAy = 0;
    int64_t sumAz = 0;
    int64_t sumGx = 0;
    int64_t sumGy = 0;
    int64_t sumGz = 0;

    uint32_t sampleCount = 0;
    uint32_t startMs = millis();

    while ((uint32_t)(millis() - startMs) < durationMs) {
        RawData raw;

        if (readRawData(raw)) {
            sumAx += raw.ax;
            sumAy += raw.ay;
            sumAz += raw.az;

            sumGx += raw.gx;
            sumGy += raw.gy;
            sumGz += raw.gz;

            sampleCount++;
        }

        delay(sampleDelayMs);
    }

    if (sampleCount == 0) {
        return false;
    }

    const float invN = 1.0f / static_cast<float>(sampleCount);

    const float avgAxRaw = static_cast<float>(sumAx) * invN;
    const float avgAyRaw = static_cast<float>(sumAy) * invN;
    const float avgAzRaw = static_cast<float>(sumAz) * invN;

    _gyroBiasRaw.x = static_cast<float>(sumGx) * invN;
    _gyroBiasRaw.y = static_cast<float>(sumGy) * invN;
    _gyroBiasRaw.z = static_cast<float>(sumGz) * invN;

    const float avgAxG = avgAxRaw / _accelScaleFactor;
    const float avgAyG = avgAyRaw / _accelScaleFactor;
    const float avgAzG = avgAzRaw / _accelScaleFactor;

    // The accelerometer sees gravity, so do not simply subtract avgAx/avgAy/avgAz
    // from future accel readings. Instead, use the averaged accel vector to define
    // the initial roll/pitch angles as zero.
    _accelAngleOffsetDeg.x = computeAccelAngleX(avgAxG, avgAyG, avgAzG);
    _accelAngleOffsetDeg.y = computeAccelAngleY(avgAxG, avgAyG, avgAzG);
    _accelAngleOffsetDeg.z = 0.0f;

    resetAngles();

    _lastProcessedData.ax = avgAxG;
    _lastProcessedData.ay = avgAyG;
    _lastProcessedData.az = avgAzG;
    _lastProcessedData.gx = 0.0f;
    _lastProcessedData.gy = 0.0f;
    _lastProcessedData.gz = 0.0f;

    _lastUpdateMicros = micros();
    _calibrated = true;

    return true;
}

bool MPU6050Complementary::update() {
    if (!_initialized || !_calibrated) {
        return false;
    }

    const uint32_t nowMicros = micros();

    float dt = static_cast<float>((uint32_t)(nowMicros - _lastUpdateMicros)) / 1000000.0f;
    _lastUpdateMicros = nowMicros;

    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.005f;
    }

    ProcessedData data;
    if (!readProcessedData(data)) {
        return false;
    }

    _lastProcessedData = data;

    const float accelAngleX =
        computeAccelAngleX(data.ax, data.ay, data.az) - _accelAngleOffsetDeg.x;

    const float accelAngleY =
        computeAccelAngleY(data.ax, data.ay, data.az) - _accelAngleOffsetDeg.y;

    // Complementary filter:
    //
    // Gyro term:
    //   Good short-term response, but drifts over time.
    //
    // Accelerometer term:
    //   Good long-term reference to gravity, but noisy/vibration-sensitive.
    //
    // If either angle moves opposite of what you expect, flip the sign of
    // data.gx or data.gy below to match your physical mounting convention.
    _angles.x = _alpha * (_angles.x + data.gx * dt) + (1.0f - _alpha) * accelAngleX;
    _angles.y = _alpha * (_angles.y + data.gy * dt) + (1.0f - _alpha) * accelAngleY;

    // MPU6050 has no magnetometer, so yaw is gyro-only and will drift.
    _angles.z += data.gz * dt;

    return true;
}

bool MPU6050Complementary::readRawData(RawData &raw) {
    uint8_t buffer[14];

    if (!readRegisters(MPU_REG_ACCEL_XOUT_H, buffer, 14)) {
        return false;
    }

    raw.ax = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    raw.ay = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    raw.az = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);

    // buffer[6] and buffer[7] are temperature.
    raw.gx = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
    raw.gy = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
    raw.gz = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);

    return true;
}

bool MPU6050Complementary::readProcessedData(ProcessedData &data) {
    RawData raw;

    if (!readRawData(raw)) {
        return false;
    }

    data.ax = static_cast<float>(raw.ax) / _accelScaleFactor;
    data.ay = static_cast<float>(raw.ay) / _accelScaleFactor;
    data.az = static_cast<float>(raw.az) / _accelScaleFactor;

    data.gx = (static_cast<float>(raw.gx) - _gyroBiasRaw.x) / _gyroScaleFactor;
    data.gy = (static_cast<float>(raw.gy) - _gyroBiasRaw.y) / _gyroScaleFactor;
    data.gz = (static_cast<float>(raw.gz) - _gyroBiasRaw.z) / _gyroScaleFactor;

    return true;
}

MPU6050Complementary::Angles MPU6050Complementary::getAngles() const {
    return _angles;
}

MPU6050Complementary::ProcessedData MPU6050Complementary::getLastProcessedData() const {
    return _lastProcessedData;
}

void MPU6050Complementary::resetAngles() {
    _angles.x = 0.0f;
    _angles.y = 0.0f;
    _angles.z = 0.0f;
}

void MPU6050Complementary::setComplementaryAlpha(float alpha) {
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    _alpha = alpha;
}

void MPU6050Complementary::setTau(float alpha) {
    setComplementaryAlpha(alpha);
}

bool MPU6050Complementary::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);

    return _wire->endTransmission(true) == 0;
}

bool MPU6050Complementary::readRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length) {
    _wire->beginTransmission(_address);
    _wire->write(startReg);

    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    const uint8_t bytesReceived =
        _wire->requestFrom(_address, length, static_cast<uint8_t>(true));

    if (bytesReceived != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = static_cast<uint8_t>(_wire->read());
    }

    return true;
}

float MPU6050Complementary::computeAccelAngleX(float ax_g, float ay_g, float az_g) {
    (void)ax_g;

    // Roll-like angle about x-axis.
    return atan2f(ay_g, az_g) * RAD_TO_DEG_F;
}

float MPU6050Complementary::computeAccelAngleY(float ax_g, float ay_g, float az_g) {
    // Pitch-like angle about y-axis.
    return atan2f(-ax_g, sqrtf((ay_g * ay_g) + (az_g * az_g))) * RAD_TO_DEG_F;
}