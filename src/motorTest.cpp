#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <math.h>

// ================================================================
// Hardware configuration
// ================================================================

#define MPU_ADDR 0x68

// MPU6050 I2C pins on STM32
constexpr uint8_t MPU_SDA = PB7;
constexpr uint8_t MPU_SCL = PB6;

// DRV8313 / 3-PWM motor pins
constexpr uint8_t MOTOR_IN1 = PA8;
constexpr uint8_t MOTOR_IN2 = PA9;
constexpr uint8_t MOTOR_IN3 = PA10;

// Your gimbal motor pole-pair count
constexpr int MOTOR_POLE_PAIRS = 11;

// ================================================================
// Control configuration
// ================================================================

// Choose which IMU axis this one motor is currently controlling.
// Start with AXIS_X or AXIS_Y. MPU6050 yaw, AXIS_Z, will drift because
// there is no magnetometer correction.
enum ControlAxis {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
};

constexpr ControlAxis CONTROL_AXIS = AXIS_X;

// Desired gimbal angle for this test motor
constexpr float ANGLE_SETPOINT_DEG = 0.0f;

// If the motor runs away from level instead of correcting toward level,
// change this from +1.0f to -1.0f.
constexpr float MOTOR_DIRECTION = 1.0f;

// Complementary filter coefficient
constexpr float ALPHA = 0.98f;

// Control loop timing
constexpr uint32_t CONTROL_PERIOD_US = 5000;   // 200 Hz

// Safety limits
constexpr float MAX_TARGET_RATE_DPS = 80.0f;   // outer PID output limit
constexpr float MAX_MOTOR_CMD_RAD_S = 20.0f;   // open-loop motor command limit
constexpr float MOTOR_VOLTAGE_LIMIT = 2.0f;    // start low for safety

// ================================================================
// Raw sensor values
// ================================================================

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Gyroscope calibration offsets, in raw LSB
float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

// Estimated angles, degrees
float angle_x = 0.0f;
float angle_y = 0.0f;
float angle_z = 0.0f;

// Converted IMU values
float ax_g = 0.0f;
float ay_g = 0.0f;
float az_g = 0.0f;

float gx_dps = 0.0f;
float gy_dps = 0.0f;
float gz_dps = 0.0f;

// ================================================================
// SimpleFOC motor objects
// ================================================================

BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

// Current open-loop command sent to the motor
float motor_cmd_rad_s = 0.0f;

// ================================================================
// Simple PID class
// ================================================================

class SimplePID {
public:
    SimplePID(float kp, float ki, float kd, float output_min, float output_max)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          output_min_(output_min),
          output_max_(output_max) {}

    float update(float setpoint, float measurement, float dt) {
        if (dt <= 0.0f) {
            return previous_output_;
        }

        float error = setpoint - measurement;

        // Integral with basic anti-windup
        integral_ += error * dt;

        if (ki_ > 0.0f) {
            float integral_min = output_min_ / ki_;
            float integral_max = output_max_ / ki_;
            integral_ = constrain(integral_, integral_min, integral_max);
        }

        float derivative = 0.0f;
        if (!first_update_) {
            derivative = (error - previous_error_) / dt;
        } else {
            first_update_ = false;
        }

        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        output = constrain(output, output_min_, output_max_);

        previous_error_ = error;
        previous_output_ = output;

        return output;
    }

    void reset() {
        integral_ = 0.0f;
        previous_error_ = 0.0f;
        previous_output_ = 0.0f;
        first_update_ = true;
    }

private:
    float kp_;
    float ki_;
    float kd_;

    float output_min_;
    float output_max_;

    float integral_ = 0.0f;
    float previous_error_ = 0.0f;
    float previous_output_ = 0.0f;

    bool first_update_ = true;
};

// ================================================================
// Cascaded PID controllers
// ================================================================
//
// Outer PID:
//     angle error [deg] -> desired angular rate [deg/s]
//
// Inner PID:
//     angular-rate error [deg/s] -> open-loop motor command [rad/s]
//
// These gains are intentionally conservative starting values.
// Tune them on hardware.

SimplePID angle_pid(
    3.0f,                       // Kp: deg error -> deg/s command
    0.0f,                       // Ki
    0.0f,                       // Kd
    -MAX_TARGET_RATE_DPS,
     MAX_TARGET_RATE_DPS
);

SimplePID rate_pid(
    0.12f,                      // Kp: deg/s error -> motor rad/s command
    0.02f,                      // Ki
    0.0f,                       // Kd
    -MAX_MOTOR_CMD_RAD_S,
     MAX_MOTOR_CMD_RAD_S
);

// ================================================================
// MPU6050 functions
// ================================================================

void mpu_init() {
    Wire.setSDA(MPU_SDA);
    Wire.setSCL(MPU_SCL);
    Wire.begin();
    Wire.setClock(400000);

    delay(100);

    // Wake MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);       // PWR_MGMT_1
    Wire.write(0x00);       // wake
    Wire.endTransmission(true);

    // Gyro full scale: +/- 250 deg/s, sensitivity 131 LSB/(deg/s)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);       // GYRO_CONFIG
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Accelerometer full scale: +/- 2g, sensitivity 16384 LSB/g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);       // ACCEL_CONFIG
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Digital low-pass filter: roughly 44 Hz accel / 42 Hz gyro
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);       // CONFIG
    Wire.write(0x03);
    Wire.endTransmission(true);
}

bool mpu_read() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);       // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t bytes_read = Wire.requestFrom(
        (uint8_t)MPU_ADDR,
        (uint8_t)14,
        (uint8_t)true
    );

    if (bytes_read != 14) {
        return false;
    }

    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());

    Wire.read();
    Wire.read();            // skip temperature

    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());

    return true;
}

void convert_imu_units() {
    ax_g = ax / 16384.0f;
    ay_g = ay / 16384.0f;
    az_g = az / 16384.0f;

    gx_dps = (gx - gx_offset) / 131.0f;
    gy_dps = (gy - gy_offset) / 131.0f;
    gz_dps = (gz - gz_offset) / 131.0f;
}

void calibrate_gyro() {
    Serial.println("Calibrating gyro. Hold the system still...");

    while (true) {
        if (!mpu_read()) {
            Serial.println("MPU read failed during stillness check.");
            delay(100);
            continue;
        }

        float ax_temp = ax / 16384.0f;
        float ay_temp = ay / 16384.0f;
        float az_temp = az / 16384.0f;

        float accel_mag = sqrtf(
            ax_temp * ax_temp +
            ay_temp * ay_temp +
            az_temp * az_temp
        );

        if (fabsf(accel_mag - 1.0f) < 0.05f) {
            break;
        }

        Serial.println("Moving or tilted hard. Waiting for stillness...");
        delay(100);
    }

    constexpr int samples = 500;

    int32_t gx_sum = 0;
    int32_t gy_sum = 0;
    int32_t gz_sum = 0;

    for (int i = 0; i < samples; i++) {
        if (mpu_read()) {
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
        }
        delay(2);
    }

    gx_offset = gx_sum / (float)samples;
    gy_offset = gy_sum / (float)samples;
    gz_offset = gz_sum / (float)samples;

    Serial.println("Gyro calibration complete.");
    Serial.print("gx_offset: "); Serial.println(gx_offset);
    Serial.print("gy_offset: "); Serial.println(gy_offset);
    Serial.print("gz_offset: "); Serial.println(gz_offset);
}

void initialize_attitude_from_accel() {
    if (!mpu_read()) {
        Serial.println("Failed to initialize attitude from accelerometer.");
        return;
    }

    convert_imu_units();

    angle_x = atan2f(ay_g, az_g) * 180.0f / PI;
    angle_y = atan2f(-ax_g, az_g) * 180.0f / PI;
    angle_z = 0.0f;
}

void update_attitude(float dt) {
    convert_imu_units();

    float acc_x_ang = atan2f(ay_g, az_g) * 180.0f / PI;
    float acc_y_ang = atan2f(-ax_g, az_g) * 180.0f / PI;

    float gyro_x_ang = angle_x + gx_dps * dt;
    float gyro_y_ang = angle_y + gy_dps * dt;

    angle_x = ALPHA * gyro_x_ang + (1.0f - ALPHA) * acc_x_ang;
    angle_y = ALPHA * gyro_y_ang + (1.0f - ALPHA) * acc_y_ang;

    // Yaw has no accelerometer correction on an MPU6050.
    // It will drift over time.
    angle_z += gz_dps * dt;
}

// ================================================================
// Axis helpers
// ================================================================

float get_control_angle_deg() {
    switch (CONTROL_AXIS) {
        case AXIS_X:
            return angle_x;
        case AXIS_Y:
            return angle_y;
        case AXIS_Z:
            return angle_z;
        default:
            return angle_x;
    }
}

float get_control_rate_dps() {
    switch (CONTROL_AXIS) {
        case AXIS_X:
            return gx_dps;
        case AXIS_Y:
            return gy_dps;
        case AXIS_Z:
            return gz_dps;
        default:
            return gx_dps;
    }
}

// ================================================================
// Arduino setup / loop
// ================================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("Starting MPU6050...");
    mpu_init();

    delay(100);

    calibrate_gyro();
    initialize_attitude_from_accel();

    Serial.println("Starting motor driver...");

    driver.voltage_power_supply = 12.0f;
    driver.init();

    motor.linkDriver(&driver);

    // No magnetic encoder is present, so use open-loop velocity mode.
    motor.controller = MotionControlType::velocity_openloop;

    // Start low. Increase carefully only after verifying direction and stability.
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor.velocity_limit = MAX_MOTOR_CMD_RAD_S;

    motor.init();

    Serial.println("Setup complete.");
    Serial.println("If the motor drives away from level, flip MOTOR_DIRECTION.");
}

void loop() {
    static uint32_t last_control_time = micros();
    static uint32_t last_print_time = millis();

    // SimpleFOC open-loop mode needs move() called as often as possible.
    motor.move(motor_cmd_rad_s);

    uint32_t now = micros();

    if ((uint32_t)(now - last_control_time) < CONTROL_PERIOD_US) {
        return;
    }

    float dt = (now - last_control_time) * 1.0e-6f;
    last_control_time = now;

    // Reject unreasonable dt values, for example after Serial delays or startup.
    if (dt <= 0.0f || dt > 0.05f) {
        angle_pid.reset();
        rate_pid.reset();
        motor_cmd_rad_s = 0.0f;
        return;
    }

    if (!mpu_read()) {
        motor_cmd_rad_s = 0.0f;
        return;
    }

    update_attitude(dt);

    float measured_angle_deg = get_control_angle_deg();
    float measured_rate_dps = get_control_rate_dps();

    // -----------------------------
    // Outer PID: angle -> target rate
    // -----------------------------
    float target_rate_dps = angle_pid.update(
        ANGLE_SETPOINT_DEG,
        measured_angle_deg,
        dt
    );

    // -----------------------------
    // Inner PID: rate -> motor command
    // -----------------------------
    float raw_motor_cmd_rad_s = rate_pid.update(
        target_rate_dps,
        measured_rate_dps,
        dt
    );

    motor_cmd_rad_s = MOTOR_DIRECTION * raw_motor_cmd_rad_s;
    motor_cmd_rad_s = constrain(
        motor_cmd_rad_s,
        -MAX_MOTOR_CMD_RAD_S,
         MAX_MOTOR_CMD_RAD_S
    );

    // // Optional deadband to reduce jitter near level
    // if (fabsf(measured_angle_deg - ANGLE_SETPOINT_DEG) < 0.5f &&
    //     fabsf(measured_rate_dps) < 2.0f) {
    //     motor_cmd_rad_s = 0.0f;
    //     rate_pid.reset();
    // }

    if (millis() - last_print_time > 500) {
        last_print_time = millis();

        Serial.print("angle_x: "); Serial.print(angle_x, 2);
        Serial.print("  angle_y: "); Serial.print(angle_y, 2);
        Serial.print("  angle_z: "); Serial.print(angle_z, 2);

        Serial.print("  gx: "); Serial.print(gx_dps, 2);
        Serial.print("  gy: "); Serial.print(gy_dps, 2);
        Serial.print("  gz: "); Serial.print(gz_dps, 2);

        Serial.print("  target_rate: "); Serial.print(target_rate_dps, 2);
        Serial.print("  motor_cmd_rad_s: "); Serial.println(motor_cmd_rad_s, 2);
    }
}