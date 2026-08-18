#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SimpleFOC.h>

#define MPU_ADDR 0x68

// Raw sensor output
int16_t ax, ay, az, gx, gy, gz;

unsigned long prev_time = 0;
const float Alpha = 0.98f; 
float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

// Complementary filter angles (degrees)
float angle_x = 0.0f;
float angle_y = 0.0f;
float angle_z = 0.0f;

//Control parameters
float Kp_ang_x = 3.0f;
float Kp_vel_x = 0.2f;
float Ki_vel_x = 0.01f;
float Kd_vel_x = 0.0001f; // DESTINY 2 REFERENCE!
float vel_integral_x = 0.0f;
float prev_vel_error_x = 0.0f;

void initAS5048A() {
    pinMode(PA4, OUTPUT);
    digitalWrite(PA4, HIGH);
    
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.setSCLK(PA5);
    SPI.begin();
}

float readAS5048A() {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(PA4, LOW);
    
    uint16_t raw_data = SPI.transfer16(0xFFFF);
    
    digitalWrite(PA4, HIGH);
    SPI.endTransaction();

    uint16_t raw_angle = raw_data & 0x3FFF;

    return ((float)raw_angle / 16384.0f) * _2PI; 
}

GenericSensor encoder_x = GenericSensor(readAS5048A, initAS5048A);
BLDCDriver3PWM driver_x = BLDCDriver3PWM(PA0, PA1, PA2);
BLDCMotor motor_x = BLDCMotor(11);

void initMPU6050() { 
    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    Wire.begin(); 
    delay(100); 

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0x00); 
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission(true);
}

void readMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); 
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
}

void calibrate() {
    Serial.println("Calibrating - hold still...");

    while (true) {
        readMPU6050();
        float ax_g = ax / 16384.0f;
        float ay_g = ay / 16384.0f;
        float az_g = az / 16384.0f;
        float magnitude = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
        if (abs(magnitude - 1.0f) < 0.25f) break;
        Serial.print("Moving - waiting for stillness... Magnitude: ");
        Serial.println(magnitude, 4);
        delay(100);
    }

    long gx_sum = 0;
    long gy_sum = 0;
    long gz_sum = 0;
    int samples = 500;

    for (int i = 0; i < samples; i++) {
        readMPU6050();
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        delay(2);
    }

    gx_offset = gx_sum / (float)samples;  
    gy_offset = gy_sum / (float)samples;
    gz_offset = gz_sum / (float)samples;
}

void setup() {
    Serial.begin(115200);
    delay(2000); 

    // Initialize MPU6050
    initMPU6050();
    calibrate();

    // Initialize AS5048A Encoder
    encoder_x.init();
    motor_x.linkSensor(&encoder_x);

    // Initialize DRV8313 Driver
    driver_x.voltage_power_supply = 12;
    driver_x.pwm_frequency = 32000;
    driver_x.init();
    motor_x.linkDriver(&driver_x);
    
    // Motor settings
    motor_x.controller = MotionControlType::torque;
    motor_x.torque_controller = TorqueControlType::voltage;
    motor_x.foc_modulation = FOCModulationType::Trapezoid_120;
    motor_x.voltage_limit = 6.0f;


    motor_x.init();
    motor_x.initFOC();
    
    motor_x.P_angle.reset();
    motor_x.PID_velocity.reset();

    prev_time = micros();
}

void loop() {
    // 1. FOC Update
    motor_x.loopFOC();
    
    // 2. MPU Math
    unsigned long now = micros();
    float dt = (now - prev_time) / 1000000.0f;

    if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.001f;
    }

    prev_time = now;
    
    readMPU6050();
        
    float ax_g = ax / 16384.0f ;
    float ay_g = ay / 16384.0f ;
    float az_g = az / 16384.0f ;
    float gx_dps = (gx - gx_offset) / 131.0f;
    float gy_dps = (gy - gy_offset) / 131.0f;
    float gz_dps = (gz - gz_offset) / 131.0f;

    float acc_x_ang = atan2(ay_g, az_g) * 180 / PI; 
    float acc_y_ang = atan2(-ax_g, az_g) * 180 / PI; 

    angle_x = angle_x + gx_dps * dt; 
    angle_y = angle_y + gy_dps * dt; 
    angle_z = angle_z + gz_dps * dt; 

    float theta_x = Alpha * angle_x + (1 - Alpha) * acc_x_ang;
    float theta_y = Alpha * angle_y + (1 - Alpha) * acc_y_ang;

    angle_x = theta_x;
    angle_y = theta_y;

    // 3. Telemetry Output
    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        // Serial.print("ax: "); Serial.print(ax_g, 4);
        // Serial.print("  ay: "); Serial.print(ay_g, 4);
        // Serial.print("  az: "); Serial.print(az_g, 4);
        // Serial.print("  gx: "); Serial.print(gx_dps, 4);
        // Serial.print("  gy: "); Serial.print(gy_dps, 4);
        // Serial.print("  gz: "); Serial.println(gz_dps, 4);

        // Serial.print(" gyro_angle_x: "); Serial.print(angle_x, 2);
        // Serial.print(" gyro_angle_y: "); Serial.print(angle_y, 2);
        // Serial.print(" acc_angle_x: "); Serial.print(acc_x_ang, 2);
        // Serial.print(" acc_angle_y: "); Serial.println(acc_y_ang, 2);
        
        Serial.print("comp_angle_x: \n"); Serial.print(theta_x, 2);
        // Serial.print("\n  comp_angle_y: \n"); Serial.print(theta_y, 2);
        // Serial.print("  comp_angle_z: "); Serial.println(angle_z, 2);
        last_print = millis();
    }
    
    float theta_x_rad = theta_x * (PI / 180.0f);
    float gx_rads = gx_dps * (PI / 180.0f);

    float target_angle_rad_x = 0.0f; 
    float angle_error_x = target_angle_rad_x - theta_x_rad;
    float target_velocity_x = Kp_ang_x * angle_error_x;

    float velocity_error_x = target_velocity_x - gx_rads;


    vel_integral_x += velocity_error_x * dt;
    vel_integral_x = constrain(vel_integral_x, -motor_x.voltage_limit, motor_x.voltage_limit); 

    float derivative_x = (velocity_error_x - prev_vel_error_x) / dt;
    prev_vel_error_x = velocity_error_x;

    float target_voltage_x = (Kp_vel_x * velocity_error_x) + (Ki_vel_x * vel_integral_x) + (Kd_vel_x * derivative_x);
    target_voltage_x = constrain(target_voltage_x, -motor_x.voltage_limit, motor_x.voltage_limit);

    motor_x.move(target_voltage_x);
}