#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SimpleFOC.h>

#define MPU_ADDR 0x68

// Raw sensor output
int16_t ax, ay, az, gx, gy, gz;

// Complementary filter angles (degrees)
float angle_x = 0.0f;
float angle_y = 0.0f;
float angle_z = 0.0f;

BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2);
BLDCMotor motor = BLDCMotor(11);
MagneticSensorSPI encoder = MagneticSensorSPI(AS5048_SPI, PA4);

unsigned long prev_time = 0;

const float Alpha = 0.98f; // complementary filter coefficient

void mpu_init() { 
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

void mpu_read() {
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

float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

void calibrate() {
    Serial.println("Calibrating - hold still...");

    while (true) {
        mpu_read();
        float ax_g = ax / 16384.0f;
        float ay_g = ay / 16384.0f;
        float az_g = az / 16384.0f;
        float magnitude = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
        if (abs(magnitude - 1.0f) < 0.05f) break;
        Serial.println("Moving - waiting for stillness...");
        delay(100);
    }

    long gx_sum = 0;
    long gy_sum = 0;
    long gz_sum = 0;
    int samples = 500;

    for (int i = 0; i < samples; i++) {
        mpu_read();
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
    delay(2000); // Give serial monitor time to connect

    // Enable SimpleFOC debugging so we can see if initFOC succeeds
    SimpleFOCDebug::enable(&Serial);

    // 1. Initialize MPU6050
    mpu_init();
    calibrate();
    prev_time = micros();

    // 2. Initialize Encoder (The SPI Fix)
    pinMode(PA4, OUTPUT);
    digitalWrite(PA4, HIGH);

    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.setSCLK(PA5);
    SPI.begin();

    encoder.init(&SPI);
    motor.linkSensor(&encoder); // Link the fixed encoder to the motor

    // 3. Initialize Motor & Driver
    driver.voltage_power_supply = 12;
    driver.pwm_frequency = 32000;
    driver.init();
    motor.linkDriver(&driver);
    
    // Closed-loop torque control (voltage based)
    motor.controller = MotionControlType::torque;
    motor.voltage_limit = 6.0f;
    motor.init();

    // 4. Initialize FOC
    Serial.println("Starting FOC Calibration...");
    motor.initFOC();
    Serial.println("FOC Ready!");
}

void loop() {
    // 1. Main FOC algorithm function (must run as fast as possible)
    motor.loopFOC();

    // 2. Motor control (0.0f means 0 Volts applied, motor should hold still with 0 resistance)
    // Change this to a low number like 1.0f or 2.0f to make it spin later
    motor.move(0.0f); 

    // 3. MPU Math
    unsigned long now = micros();
    float dt = (now - prev_time) / 1000000.0f;
    prev_time = now;
    mpu_read();
        
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

    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        Serial.print("ax: "); Serial.print(ax_g, 4);
        Serial.print("  ay: "); Serial.print(ay_g, 4);
        Serial.print("  az: "); Serial.print(az_g, 4);
        Serial.print("  gx: "); Serial.print(gx_dps, 4);
        Serial.print("  gy: "); Serial.print(gy_dps, 4);
        Serial.print("  gz: "); Serial.println(gz_dps, 4);

        Serial.print(" gyro_angle_x: "); Serial.print(angle_x, 2);
        Serial.print(" gyro_angle_y: "); Serial.print(angle_y, 2);
        Serial.print(" acc_angle_x: "); Serial.print(acc_x_ang, 2);
        Serial.print(" acc_angle_y: "); Serial.println(acc_y_ang, 2);
        
        Serial.print("comp_angle_x: "); Serial.print(theta_x, 2);
        Serial.print("  comp_angle_y: "); Serial.print(theta_y, 2);
        Serial.print("  comp_angle_z: "); Serial.println(angle_z, 2);
        last_print = millis();
    }
}