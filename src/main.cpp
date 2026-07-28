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


GenericSensor encoder = GenericSensor(readAS5048A, initAS5048A);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2);
BLDCMotor motor = BLDCMotor(11);

unsigned long prev_time = 0;
const float Alpha = 0.98f; 
float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;


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

void calibrate() {
    Serial.println("Calibrating - hold still...");

    while (true) {
        mpu_read();
        float ax_g = ax / 16384.0f;
        float ay_g = ay / 16384.0f;
        float az_g = az / 16384.0f;
        float magnitude = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
        if (abs(magnitude - 1.0f) < 0.25f) break;
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
    delay(2000); 

    // Initialize MPU6050
    mpu_init();
    calibrate();
    prev_time = micros();

    // Initialize Custom Encoder
    encoder.init();
    motor.linkSensor(&encoder);

    // Initialize Driver
    driver.voltage_power_supply = 12;
    driver.pwm_frequency = 32000;
    driver.init();
    motor.linkDriver(&driver);
    
    // Motor settings
    motor.controller = MotionControlType::torque;
    motor.voltage_limit = 6.0f;
    motor.voltage_sensor_align = 6.0f; 
    
    motor.init();
    motor.initFOC();
}

void loop() {
    // 1. FOC Update
    motor.loopFOC();
    
    

    // 2. MPU Math
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

    // 3. Telemetry Output
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

    // 1. Set a "Proportional Gain" (Kp) 
    // This dictates how aggressively the motor fights back.
    float Kp = 0.2f; 
    
    // 2. Calculate the target voltage based on the IMU tilt
    // (Assuming you want to balance around the X-axis. Change theta_x to theta_y if needed)
    float target_voltage = -Kp * theta_x; 
    
    // 3. Constrain the voltage for safety so it doesn't exceed your 6V limit
    if(target_voltage > 6.0f) target_voltage = 6.0f;
    if(target_voltage < -6.0f) target_voltage = -6.0f;
    
    // 4. Send the calculated voltage to the motor
    motor.move(target_voltage);
}