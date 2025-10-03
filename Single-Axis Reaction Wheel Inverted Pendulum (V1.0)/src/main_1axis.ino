#include <Wire.h>

// ===================== Pin Definitions ====================== //
#define RPWM_PIN 5    // Right motor PWM input
#define LPWM_PIN 6    // Left motor PWM input
#define R_EN_PIN 8    // Right motor enable pin
#define L_EN_PIN 9    // Left motor enable pin
#define REVERSE_MOTOR false // Set to true if motor direction is inverted
#define DEADBAND 0.2        // Deadband in degrees

// ==================== MPU6050 Constants ===================== //
#define MPU6050_ADDR 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

// Gyroscope scale factor for ±500 dps
const float GYRO_SCALE = 65.5f;

// ==================== State Variables ======================= //
uint8_t i2cData[14];
int16_t AcX, AcY, AcZ, GyZ_raw;
float gyro_offset = 0.0f;
float accel_offset = 0.0f;
float angle = 0.0f;
float gyro_rate = 0.0f;
float gyroZfilt = 0.0f;
float integral = 0.0f;
float raw_u = 0.0f;

// ======================== PID Gains ========================= //
float Kp = 8.0f; // Proportional gain
float Ki = 0.0f;  // Integral gain
float Kd = 0.0f;  // Derivative gain

// ==================== Filter Parameters ===================== //
float K_comp = 0.95f;       // Complementary filter weight
float gyro_lpf_alpha = 0.3f; // Low-pass filter for gyro

// =========================== Timing ========================= //
const unsigned long loop_time_us = 5000UL; // 5ms = 200Hz
unsigned long lastMicros = 0;
unsigned long lastPrintMicros = 0;

// ==================== Motor Control ======================= //
int pwm_s = 0;

// ================== I2C Helper Functions ================== //
int i2cWrite(uint8_t reg, uint8_t *data, uint8_t len, bool wait) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission(wait);
}

int i2cRead(uint8_t reg, uint8_t *data, uint8_t nbytes) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, nbytes, true);
  for (uint8_t i = 0; i < nbytes; i++) data[i] = Wire.read();
  return 0;
}

// ================== Sensor Initialization ================== //
void angle_setup() {
  Wire.begin();
  Wire.setWireTimeout(2500, true);

  // Wake up MPU6050
  uint8_t d = 0x00;
  while (i2cWrite(PWR_MGMT_1, &d, 1, false));
  while (i2cWrite(PWR_MGMT_2, &d, 1, false));

  // Configure accelerometer ±2g
  d = 0x00; while (i2cWrite(ACCEL_CONFIG, &d, 1, false));

  // Configure gyroscope ±500 dps
  d = 0x08; while (i2cWrite(GYRO_CONFIG, &d, 1, false));

  // Gyro calibration
  Serial.println("Calibrating gyro: keep sensor absolutely still...");
  long sum = 0;
  const int Ngyro = 2000;
  for (int i = 0; i < Ngyro; i++) {
    while (i2cRead(0x3B, i2cData, 14));
    int16_t gz = (int16_t)((i2cData[12] << 8) | i2cData[13]);
    sum += (long)gz;
    delay(2);
  }
  gyro_offset = (float)sum / (float)Ngyro;
  Serial.print("gyro_offset (LSB) = "); Serial.println(gyro_offset, 3);

  // Accelerometer calibration
  Serial.println("Calibrating accel: hold vertical...");
  delay(1500);
  const int Nacc = 500;
  float sumA = 0.0f;
  for (int i = 0; i < Nacc; i++) {
    while (i2cRead(0x3B, i2cData, 14));
    int16_t ax = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    int16_t ay = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    float aang = atan2((float)ax, (float)ay) * 180.0f / PI;
    sumA += aang;
    delay(4);
  }
  accel_offset = -(sumA / (float)Nacc);
  Serial.print("accel_offset(deg) = "); Serial.println(accel_offset, 3);

  // Initialize angle
  while (i2cRead(0x3B, i2cData, 14));
  int16_t ax = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  int16_t ay = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  angle = atan2((float)ax, (float)ay) * 180.0f / PI + accel_offset;
  gyroZfilt = 0.0f;
}

// ================== Angle Calculation ================== //
void angle_calc(float dt) {
  while (i2cRead(0x3B, i2cData, 14));
  AcX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  AcY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  GyZ_raw = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  float accel_angle = atan2((float)AcX, (float)AcY) * 180.0f / PI + accel_offset;
  if (accel_angle > 180.0f) accel_angle -= 360.0f;
  if (accel_angle < -180.0f) accel_angle += 360.0f;

  gyro_rate = ((float)GyZ_raw - gyro_offset) / GYRO_SCALE;
  gyroZfilt = gyro_lpf_alpha * gyro_rate + (1.0f - gyro_lpf_alpha) * gyroZfilt;
  angle = K_comp * (angle + gyro_rate * dt) + (1.0f - K_comp) * accel_angle;
}

// ================== Motor Control ================== //
void Motor_control(int pwm) {
  int abs_pwm = abs(pwm);
  if (REVERSE_MOTOR) pwm = -pwm;

  if (pwm >= 0) {
    analogWrite(LPWM_PIN, 0);
    analogWrite(RPWM_PIN, abs_pwm);
  } else {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, abs_pwm);
  }
}

// ================== Serial Tuning ================== //
void checkSerialForTuning() {
  static String buffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer.trim();
      if (buffer == "p+") Kp += 1.0;
      else if (buffer == "p-") Kp -= 1.0;
      else if (buffer == "i+") Ki += 0.01;
      else if (buffer == "i-") Ki -= 0.01;
      else if (buffer == "d+") Kd += 0.25;
      else if (buffer == "d-") Kd -= 0.25;

      if (Kp < 0) Kp = 0;
      if (Ki < 0) Ki = 0;
      if (Kd < 0) Kd = 0;

      Serial.print("Gains | Kp: "); Serial.print(Kp, 3);
      Serial.print(" Ki: "); Serial.print(Ki, 4);
      Serial.print(" Kd: "); Serial.println(Kd, 3);
      buffer = "";
    } else buffer += c;
  }
}

void setup() {
  Serial.begin(115200);

  // Setup motor driver pins
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);

  // Initialize MPU6050 and perform calibration
  angle_setup();
  Serial.println("Setup complete.");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0.0f) dt = 1e-3f;
  if (dt * 1e6f >= (float)loop_time_us) {
    lastMicros = now;

    // Update angle estimate
    angle_calc(dt);

    // PID Control
    float integral_candidate = integral + angle * dt;
    float u_test = -(Kp * angle + Kd * gyroZfilt + Ki * integral_candidate);
    if (abs(u_test) <= 255.0f) integral = integral_candidate;
    integral = constrain(integral, -2000.0f, 2000.0f);

    raw_u = -(Kp * angle + Kd * gyroZfilt + Ki * integral);
    pwm_s = (int)constrain(raw_u, -255.0f, 255.0f);

    if (abs(angle) < DEADBAND) pwm_s = 0;

    // Drive motor
    digitalWrite(R_EN_PIN, HIGH);
    digitalWrite(L_EN_PIN, HIGH);
    Motor_control(pwm_s);

    // Telemetry
    if (now - lastPrintMicros > 50000UL) { // every 50 ms
      Serial.print("ang(deg): "); Serial.print(angle, 3);
      Serial.print(" | gyro(deg/s): "); Serial.print(gyro_rate, 3);
      Serial.print(" | gyro_filt: "); Serial.print(gyroZfilt, 3);
      Serial.print(" | integral: "); Serial.print(integral, 3);
      Serial.print(" | raw_u: "); Serial.print(raw_u, 3);
      Serial.print(" | pwm: "); Serial.print(pwm_s);
      Serial.print(" | dt(s): "); Serial.println(dt, 6);
      lastPrintMicros = now;
    }

    // Handle tuning
    checkSerialForTuning();
  }

  while (micros() - now < loop_time_us) {}
}