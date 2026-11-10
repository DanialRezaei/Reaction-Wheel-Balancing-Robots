#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void IRAM_ATTR ENC1_READ();
void IRAM_ATTR ENC2_READ();
void IRAM_ATTR ENC3_READ();

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Cube");
  EEPROM.begin(EEPROM_SIZE);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  // Initialize LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  Wire.begin(21, 22, 400000);

  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  ledcAttach(PWM1, BASE_FREQ, TIMER_BIT);
  Motor1_control(0);
  attachInterrupt(digitalPinToInterrupt(ENC1_1), ENC1_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_2), ENC1_READ, CHANGE);
  
  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  ledcAttach(PWM2, BASE_FREQ, TIMER_BIT);
  Motor2_control(0);
  attachInterrupt(digitalPinToInterrupt(ENC2_1), ENC2_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_2), ENC2_READ, CHANGE);
  
  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  ledcAttach(PWM3, BASE_FREQ, TIMER_BIT);
  Motor3_control(0);
  attachInterrupt(digitalPinToInterrupt(ENC3_1), ENC3_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_2), ENC3_READ, CHANGE);

  EEPROM.get(0, offsets);
  if (offsets.ID == 96) 
    calibrated = true;

  delay(200);
  angle_setup();
}

void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning();
    angle_calc();

    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;
    
    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;
    
    if (vertical_vertex && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      
      gyroX = GyX / 131.0;
      gyroY = GyY / 131.0;
      gyroZ = GyZ / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      
      int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
      int pwm_Z = constrain(zK2 * gyroZ + zK3 * motors_speed_Z, -255, 255);

      motors_speed_X += speed_X / 5; 
      motors_speed_Y += speed_Y / 5;
      XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
    } else if (vertical_edge && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      
      gyroX = GyX / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      
      int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);
      
      motors_speed_X += motor3_speed / 5;
      Motor3_control(pwm_X);
    } else {
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0;
    }
    previousT_1 = currentT;
  }
  
  if (currentT - previousT_2 >= 5000) {    
    battVoltage();
    if (!calibrated && !calibrating) {
      SerialBT.println("first you need to calibrate the balancing points...");
      Serial.println("first you need to calibrate the balancing points (over bluetooth)...");
    }
    previousT_2 = currentT;
  }  
}