#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68

// Raw sensor output
int16_t ax, ay, az, gx, gy, gz;

// Complementary filter angles (degrees)
float angle_x = 0.0f;
float angle_y = 0.0f;

unsigned long prev_time = 0;

const int Alpha = 0.99; // complementary filter coefficient

void mpu_init() {
    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    Wire.begin(); 
    delay(100); // Wait for MPU6050 to power up
    // Wake the MPU6050 — it boots in sleep mode
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x00); // Clear sleep bit
    Wire.endTransmission(true);

    // Gyro full scale: ±250°/s → sensitivity 131 LSB/°/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Accel full scale: ±2g → sensitivity 16384 LSB/g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Digital low pass filter: 44Hz bandwidth — reduces vibration noise
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
    Wire.read(); Wire.read(); // skip temperature
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
}

void setup() {
    Serial.begin(115200);
    mpu_init();
    delay(100);
    prev_time = micros();
    Serial.println("MPU6050 ready");
}


void loop() {
    unsigned long now = micros();
    float dt = (now - prev_time) / 1000000.0f;
    prev_time = now;
    mpu_read();

    angle_x = angle_x + gx * dt; //gyroscope
    angle_y = angle_y + gy * dt; //gyroscope

    float acc_x_ang = atan2(ay, az) * 180 / PI; // angle about x
    float acc_y_ang = atan2(ax, az) * 180 / PI; // angle about y
    float ax_g = ax / 16384.0f ;
    float ay_g = ay / 16384.0f ;
    float az_g = az / 16384.0f ;
    float gx_dps = gx / 131.0f;
    float gy_dps = gy / 131.0f; 

    float theta_x = Alpha * angle_x + (1 - Alpha) * acc_x_ang;
    float theta_y = Alpha * angle_y + (1 - Alpha) * acc_y_ang;

    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        /*
        Serial.print("ax: "); Serial.print(ax_g, 4);
        Serial.print("  ay: "); Serial.print(ay_g, 4);
        Serial.print("  az: "); Serial.print(az_g, 4);
        Serial.print("  gx: "); Serial.print(gx_dps, 4);
        Serial.print("  gy: "); Serial.println(gy_dps, 4);
        
        Serial.print(" gyro_angle_x: "); Serial.print(angle_x, 2);
        Serial.print(" gyro_angle_y: "); Serial.println(angle_y, 2);
        Serial.print(" acc_angle_x: "); Serial.print(acc_x_ang, 2);
        Serial.print(" acc_angle_y: "); Serial.println(acc_y_ang, 2);
        */
        Serial.print("comp_angle_x: "); Serial.print(theta_x, 2);
        Serial.print("  comp_angle_y: "); Serial.println(theta_y, 2);
        last_print = millis();
    }

    delay(5); 
}
