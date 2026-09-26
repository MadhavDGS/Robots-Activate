/*
Hybrid FlySky ESP32 Soccer Bot Code
Combines the best of both worlds:
- Uses currentinsideesp.ino pin connections (no rewiring needed!)
- FlySky RC receiver control (no Bluetooth)
- Exponential curve for smooth control
- Dual stick support (CH2 forward/back + CH4 left/right)
- 30kHz PWM for quieter motor operation

PIN CONNECTIONS (Same as currentinsideesp.ino):
----------------------------------------------
FlySky Receiver → ESP32:
  CH2 (Forward/Back) → GPIO 34
  CH4 (Left/Right)   → GPIO 32 (NEW - add this wire!)
  CH5 (Speed Switch) → GPIO 35

BTS7960 Left Motor → ESP32:
  RPWM (Forward) → GPIO 19
  LPWM (Reverse) → GPIO 21

BTS7960 Right Motor → ESP32:
  RPWM (Forward) → GPIO 22
  LPWM (Reverse) → GPIO 23

ESP32 Board version: 2.0.11
*/

// ========== Include Libraries ==========
#include <WiFi.h>
#include "esp_bt.h"

// ========== FlySky Receiver Input Pins ==========
#define CH2_PIN 34  // Forward/Backward (left stick Y-axis)
#define CH4_PIN 32  // Left/Right turns (right stick X-axis) - NEW CONNECTION!
#define CH5_PIN 35  // Speed Mode toggle Switch

// ========== BTS7960 Motor Driver Pins (from currentinsideesp) ==========
// Left Motor
#define ML_RPWM 19  // Left motor forward PWM
#define ML_LPWM 21  // Left motor reverse PWM

// Right Motor
#define MR_RPWM 22  // Right motor forward PWM
#define MR_LPWM 23  // Right motor reverse PWM

// ========== PWM Channels ==========
#define ML_RPWM_CH 0  // Left motor forward channel
#define ML_LPWM_CH 1  // Left motor reverse channel
#define MR_RPWM_CH 2  // Right motor forward channel
#define MR_LPWM_CH 3  // Right motor reverse channel

// ========== PWM Settings (30kHz for quieter operation) ==========
const int PWM_FREQ = 30000;      // 30kHz - much quieter than 1kHz
const int PWM_RESOLUTION = 8;    // 8-bit (0-255)

// ========== RC Signal Limits ==========
const int RC_MAX = 2000;   // Maximum pulse width (µs)
const int RC_MID = 1500;   // Midpoint pulse width (µs)
const int RC_MIN = 1000;   // Minimum pulse width (µs)

// ========== Control Settings ==========
const int DEADBAND_THRESHOLD = 30;        // Ignore small joystick movements
const float EXPO_CURVE = 1.8;             // Smoothness (1.0=linear, 2.0+=smoother)
const int MIN_MOTOR_SPEED = 0;            // Minimum PWM for movement (0=no minimum)

float speedMultiplier = 1.0;  // Speed scaling (adjustable via CH5)

// ========== Function: Apply Exponential Curve ==========
// Makes control smoother at low inputs, more responsive at high inputs
int applyExponentialCurve(int value, int maxValue) {
  if (value == 0) return 0;
  
  // Normalize to 0.0 to 1.0
  float normalized = (float)abs(value) / (float)maxValue;
  
  // Apply exponential curve
  float curved = pow(normalized, EXPO_CURVE);
  
  // Convert back to original range and restore sign
  int result = (int)(curved * maxValue);
  return (value > 0) ? result : -result;
}

// ========== Function: Apply Deadband ==========
int applyDeadband(int value, int threshold = DEADBAND_THRESHOLD) {
  return (abs(value) < threshold) ? 0 : value;
}

// ========== Function: Drive Motors ==========
void driveMotors(int leftSpeed, int rightSpeed) {
  // Left Motor Control
  if (leftSpeed > 0) {
    ledcWrite(ML_LPWM_CH, 0);          // No reverse
    ledcWrite(ML_RPWM_CH, leftSpeed);   // Forward
  } else if (leftSpeed < 0) {
    ledcWrite(ML_RPWM_CH, 0);          // No forward
    ledcWrite(ML_LPWM_CH, abs(leftSpeed)); // Reverse
  } else {
    ledcWrite(ML_RPWM_CH, 0);
    ledcWrite(ML_LPWM_CH, 0);
  }
  
  // Right Motor Control
  if (rightSpeed > 0) {
    ledcWrite(MR_LPWM_CH, 0);          // No reverse
    ledcWrite(MR_RPWM_CH, rightSpeed);  // Forward
  } else if (rightSpeed < 0) {
    ledcWrite(MR_RPWM_CH, 0);          // No forward
    ledcWrite(MR_LPWM_CH, abs(rightSpeed)); // Reverse
  } else {
    ledcWrite(MR_RPWM_CH, 0);
    ledcWrite(MR_LPWM_CH, 0);
  }
}

// ========== Function: Stop Motors ==========
void stopMotors() {
  ledcWrite(ML_RPWM_CH, 0);
  ledcWrite(ML_LPWM_CH, 0);
  ledcWrite(MR_RPWM_CH, 0);
  ledcWrite(MR_LPWM_CH, 0);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  
  // Turn OFF WiFi and Bluetooth to reduce heat
  WiFi.mode(WIFI_OFF);
  btStop();
  
  Serial.println("\n=== Hybrid FlySky ESP32 Soccer Bot ===");
  
  // Configure receiver pins as inputs
  pinMode(CH2_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  
  // Configure motor driver pins as outputs
  pinMode(ML_RPWM, OUTPUT);
  pinMode(ML_LPWM, OUTPUT);
  pinMode(MR_RPWM, OUTPUT);
  pinMode(MR_LPWM, OUTPUT);
  
  // Setup PWM channels (30kHz for quieter operation)
  ledcSetup(ML_RPWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ML_RPWM, ML_RPWM_CH);
  
  ledcSetup(ML_LPWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ML_LPWM, ML_LPWM_CH);
  
  ledcSetup(MR_RPWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MR_RPWM, MR_RPWM_CH);
  
  ledcSetup(MR_LPWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MR_LPWM, MR_LPWM_CH);
  
  stopMotors();
  
  Serial.println("\nHardware Configuration:");
  Serial.println("  Left Motor:  D19(RPWM), D21(LPWM)");
  Serial.println("  Right Motor: D22(RPWM), D23(LPWM)");
  Serial.println("\nFlySky Receiver Channels:");
  Serial.println("  CH2 (D34) = Forward/Backward");
  Serial.println("  CH4 (D32) = Left/Right Turns");
  Serial.println("  CH5 (D35) = Speed Mode Switch");
  Serial.println("\nFeatures:");
  Serial.println("  - 30kHz PWM (quieter motors)");
  Serial.println("  - Exponential curve for smooth control");
  Serial.println("  - Dual-stick control (independent F/B and L/R)");
  Serial.println("  - 3-speed modes via CH5 switch");
  Serial.println("\nReady! Waiting for FlySky signal...\n");
}

// ========== MAIN LOOP ==========
void loop() {
  // Read RC channels (25ms timeout)
  int ch2 = pulseIn(CH2_PIN, HIGH, 25000);  // Forward/Backward
  int ch4 = pulseIn(CH4_PIN, HIGH, 25000);  // Left/Right
  int ch5 = pulseIn(CH5_PIN, HIGH, 25000);  // Speed mode
  
  // Debug output - ALWAYS ENABLED to verify operation
  Serial.printf("CH2: %4d  CH4: %4d  CH5: %4d | ", ch2, ch4, ch5);
  
  // Fail-safe: Stop if signal lost or out of range
  if (ch2 < 500 || ch2 > 2200 || ch4 < 500 || ch4 > 2200) {
    Serial.println("SIGNAL LOST - MOTORS STOPPED!");
    stopMotors();
    return;
  }
  
  // ========== Speed Mode Selection (CH5 - SWC Switch) ==========
  // SWC Position 1 (DOWN ~1000µs) = FULL 100%
  // SWC Position 2 (MIDDLE ~1500µs) = MEDIUM 70%
  // SWC Position 3 (UP ~2000µs) = LOW 40%
  String speedMode;
  if (ch5 < 1300) {                 // Position 1 DOWN: ~1000µs
    speedMultiplier = 1.0;           // 100% FULL POWER
    speedMode = "FULL-100%";
  } else if (ch5 > 1800) {          // Position 3 UP: ~2000µs
    speedMultiplier = 0.4;           // 40% LOW/TRAINING
    speedMode = "LOW-40%";
  } else {                           // Position 2 MIDDLE: ~1500µs
    speedMultiplier = 0.7;           // 70% MEDIUM/SPORT
    speedMode = "MED-70%";
  }
  Serial.printf("Speed: %s | ", speedMode.c_str());
  
  // ========== Map RC Values to Motor Range (-255 to +255) ==========
  int forwardBackward = map(ch2, RC_MIN, RC_MAX, -255, 255);  // CH2 controls forward/back
  int leftRight = map(ch4, RC_MIN, RC_MAX, -255, 255);        // CH4 controls turns
  
  // Apply exponential curve for smooth control
  forwardBackward = applyExponentialCurve(forwardBackward, 255);
  leftRight = applyExponentialCurve(leftRight, 255);
  
  // Apply speed multiplier
  forwardBackward = forwardBackward * speedMultiplier;
  leftRight = leftRight * speedMultiplier;
  
  // Apply deadband to filter out joystick noise
  forwardBackward = applyDeadband(forwardBackward);
  leftRight = applyDeadband(leftRight);
  
  // ========== Arcade Drive Mixing ==========
  // This is the best mixing algorithm from currentinsideesp
  int leftMotorSpeed = forwardBackward + leftRight;
  int rightMotorSpeed = forwardBackward - leftRight;
  
  // Constrain to valid PWM range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
  
  // Apply minimum speed threshold if needed
  if (MIN_MOTOR_SPEED > 0) {
    if (leftMotorSpeed > 0 && leftMotorSpeed < MIN_MOTOR_SPEED) {
      leftMotorSpeed = MIN_MOTOR_SPEED;
    } else if (leftMotorSpeed < 0 && leftMotorSpeed > -MIN_MOTOR_SPEED) {
      leftMotorSpeed = -MIN_MOTOR_SPEED;
    }
    
    if (rightMotorSpeed > 0 && rightMotorSpeed < MIN_MOTOR_SPEED) {
      rightMotorSpeed = MIN_MOTOR_SPEED;
    } else if (rightMotorSpeed < 0 && rightMotorSpeed > -MIN_MOTOR_SPEED) {
      rightMotorSpeed = -MIN_MOTOR_SPEED;
    }
  }
  
  // Debug output - Show motor commands
  Serial.printf("L: %4d  R: %4d\n", leftMotorSpeed, rightMotorSpeed);
  
  // Drive motors
  if (forwardBackward != 0 || leftRight != 0) {
    driveMotors(leftMotorSpeed, rightMotorSpeed);
  } else {
    stopMotors();
  }
  
  // Small delay for stability
  delay(50);  // Increased to reduce serial spam
}
