/*
FlySky Arduino Uno Soccer Bot Code
Exact same FlySky controls as ESP32 version
Adapted for Arduino Uno with BTS7960 motor drivers

PIN CONNECTIONS (Arduino Uno) - SIMPLIFIED:
----------------------------------------------
FlySky Receiver → Arduino Uno (Input Pins):
  CH2 (Forward/Back) → Pin 4 (MOVED)
  CH4 (Left/Right)   → Pin 2 (CONFIRMED WORKING - PHYSICALLY CONNECTED HERE)
  CH5 (Speed Switch) → Pin 8

BTS7960 Left Motor → Arduino Uno (PWM Pins):
  RPWM (Forward) → Pin 9  (CONFIRMED WORKING)
  LPWM (Reverse) → Pin 10 (CONFIRMED WORKING)

BTS7960 Right Motor → Arduino Uno (PWM Pins):
  RPWM (Forward) → Pin 6
  LPWM (Reverse) → Pin 3

Arduino Uno PWM Pins: 3, 5, 6, 9, 10, 11
Arduino Uno Frequency: ~490Hz (default - can't change easily)
*/

// ========== FlySky Receiver Input Pins ==========
#define CH2_PIN 4   // Forward/Backward (left stick Y-axis) - MOVED TO PIN 4
#define CH4_PIN 2   // Left/Right turns (right stick X-axis) - KEPT ON PIN 2 (WORKING)
#define CH5_PIN 8   // Speed Mode toggle Switch

// ========== BTS7960 Motor Driver Pins ==========
// Left Motor - Using CONFIRMED WORKING PWM pins 9,10
#define ML_RPWM 9   // Left motor forward PWM
#define ML_LPWM 10  // Left motor reverse PWM

// Right Motor - Using PWM pins 3,6
#define MR_RPWM 6   // Right motor forward PWM
#define MR_LPWM 3   // Right motor reverse PWM

// ========== RC Signal Limits ==========
const int RC_MAX = 2000;   // Maximum pulse width (µs)
const int RC_MID = 1500;   // Midpoint pulse width (µs)
const int RC_MIN = 1000;   // Minimum pulse width (µs)

// ========== Control Settings ==========
const int DEADBAND_THRESHOLD = 30;        // Ignore small joystick movements
const float EXPO_CURVE = 1.0;             // 1.0 linear for reliable tuning/debug
const int MIN_MOTOR_SPEED = 70;           // Minimum PWM for movement (helps overcome motor stiction)
const bool ENABLE_PIVOT_TURN = true;      // True: left/right with near-zero throttle does spot turn

// ========== Control Direction/Mixing Options ==========
const bool USE_ROTATED_MIX_DECODE = true; // FlySky elevon-style mix decode from CH2+CH4
const bool SWAP_INPUT_CHANNELS = true;    // true = swap CH2/CH4 inputs before decode
const bool INVERT_CH2 = false;            // Set true if forward stick gives backward
const bool INVERT_CH4 = false;            // Set true if left/right is reversed
const bool INVERT_DECODED_THROTTLE = false; // Flip decoded forward/back axis
const bool INVERT_DECODED_STEERING = false; // Flip decoded left/right axis
const bool INVERT_LEFT_MOTOR = true;      // Left side inversion for mirrored drivetrain wiring
const bool INVERT_RIGHT_MOTOR = false;    // Set true if right motor direction is opposite

float speedMultiplier = 1.0;  // Speed scaling (adjustable via CH5)

// ========== Interrupt-Based RC Capture (Fixes pulseIn channel conflicts) ==========
volatile uint16_t ch2PulseUs = 1500;
volatile uint16_t ch4PulseUs = 1500;
volatile uint16_t ch5PulseUs = 1500;

volatile unsigned long ch2RiseUs = 0;
volatile unsigned long ch4RiseUs = 0;
volatile unsigned long ch5RiseUs = 0;

volatile unsigned long ch2LastUpdateUs = 0;
volatile unsigned long ch4LastUpdateUs = 0;
volatile unsigned long ch5LastUpdateUs = 0;

volatile uint8_t lastPortDState = 0;  // For D4 (CH2)
volatile uint8_t lastPortBState = 0;  // For D8 (CH5)

void isrCh4Change() {
  unsigned long now = micros();
  if (digitalRead(CH4_PIN) == HIGH) {
    ch4RiseUs = now;
  } else {
    unsigned long width = now - ch4RiseUs;
    if (width >= 800 && width <= 2200) {
      ch4PulseUs = (uint16_t)width;
      ch4LastUpdateUs = now;
    }
  }
}

ISR(PCINT2_vect) {
  // D0-D7 changed; we only care about D4 (PD4)
  uint8_t current = PIND;
  uint8_t changed = current ^ lastPortDState;
  unsigned long now = micros();

  if (changed & _BV(PD4)) {
    if (current & _BV(PD4)) {
      ch2RiseUs = now;
    } else {
      unsigned long width = now - ch2RiseUs;
      if (width >= 800 && width <= 2200) {
        ch2PulseUs = (uint16_t)width;
        ch2LastUpdateUs = now;
      }
    }
  }

  lastPortDState = current;
}

ISR(PCINT0_vect) {
  // D8-D13 changed; we only care about D8 (PB0)
  uint8_t current = PINB;
  uint8_t changed = current ^ lastPortBState;
  unsigned long now = micros();

  if (changed & _BV(PB0)) {
    if (current & _BV(PB0)) {
      ch5RiseUs = now;
    } else {
      unsigned long width = now - ch5RiseUs;
      if (width >= 800 && width <= 2200) {
        ch5PulseUs = (uint16_t)width;
        ch5LastUpdateUs = now;
      }
    }
  }

  lastPortBState = current;
}

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
  // Constrain to valid PWM range (0-255)
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Left Motor Control (now on pins 9,10)
  if (leftSpeed > 0) {
    analogWrite(ML_LPWM, 0);           // No reverse
    analogWrite(ML_RPWM, leftSpeed);    // Forward
  } else if (leftSpeed < 0) {
    analogWrite(ML_RPWM, 0);           // No forward
    analogWrite(ML_LPWM, abs(leftSpeed)); // Reverse
  } else {
    analogWrite(ML_RPWM, 0);
    analogWrite(ML_LPWM, 0);
  }
  
  // Right Motor Control (pins 6,3)
  if (rightSpeed > 0) {
    analogWrite(MR_LPWM, 0);           // No reverse
    analogWrite(MR_RPWM, rightSpeed);   // Forward
  } else if (rightSpeed < 0) {
    analogWrite(MR_RPWM, 0);           // No forward
    analogWrite(MR_LPWM, abs(rightSpeed)); // Reverse
  } else {
    analogWrite(MR_RPWM, 0);
    analogWrite(MR_LPWM, 0);
  }
}

// ========== Function: Stop Motors ==========
void stopMotors() {
  analogWrite(ML_RPWM, 0);
  analogWrite(ML_LPWM, 0);
  analogWrite(MR_RPWM, 0);
  analogWrite(MR_LPWM, 0);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(9600);  // Arduino Uno typically uses 9600 baud
  
  Serial.println("\n=== FlySky Arduino Uno Soccer Bot ===");
  
  // Configure receiver pins as inputs
  pinMode(CH2_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);

  // Initialize port snapshots before enabling pin-change interrupts
  lastPortDState = PIND;
  lastPortBState = PINB;

  // CH4 on D2: external interrupt
  attachInterrupt(digitalPinToInterrupt(CH4_PIN), isrCh4Change, CHANGE);

  // CH2 on D4: pin-change interrupt (PCINT20 on PORTD)
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT20);

  // CH5 on D8: pin-change interrupt (PCINT0 on PORTB)
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT0);
  
  // Configure motor driver pins as outputs
  pinMode(ML_RPWM, OUTPUT);
  pinMode(ML_LPWM, OUTPUT);
  pinMode(MR_RPWM, OUTPUT);
  pinMode(MR_LPWM, OUTPUT);
  
  stopMotors();
  
  Serial.println("\nHardware Configuration:");
  Serial.println("  Left Motor:  D9(RPWM), D10(LPWM) [CONFIRMED WORKING]");
  Serial.println("  Right Motor: D6(RPWM), D3(LPWM)");
  Serial.println("\nFlySky Receiver Channels:");
  Serial.println("  CH2 (D4) = Forward/Backward");
  Serial.println("  CH4 (D2) = Left/Right Turns [PIN 2 PHYSICALLY CONFIRMED]");
  Serial.println("  CH5 (D8) = Speed Mode Switch");
  Serial.println("\nFeatures:");
  Serial.println("  - Exponential curve for smooth control");
  Serial.println("  - CH2 throttle + CH4 steering mixed drive");
  Serial.println("  - 3-speed modes via CH5 switch");
  Serial.println("  - Interrupt-based RC capture (no pulseIn blocking)");
  Serial.println("  - BTS7960 High-Power Motor Drivers");
  Serial.println("\nNote: Arduino Uno PWM ~490Hz (fixed)");
  Serial.println("Ready! Waiting for FlySky signal...\n");
}

// ========== MAIN LOOP ==========
void loop() {
  // Non-blocking channel read from ISR-captured pulse widths
  int ch2, ch4, ch5;
  unsigned long ch2AgeUs, ch4AgeUs, ch5AgeUs;
  unsigned long nowUs = micros();

  noInterrupts();
  ch2 = ch2PulseUs;
  ch4 = ch4PulseUs;
  ch5 = ch5PulseUs;
  ch2AgeUs = nowUs - ch2LastUpdateUs;
  ch4AgeUs = nowUs - ch4LastUpdateUs;
  ch5AgeUs = nowUs - ch5LastUpdateUs;
  interrupts();

  // If no fresh pulse in ~50ms, mark as lost
  if (ch2AgeUs > 50000UL) ch2 = 0;
  if (ch4AgeUs > 50000UL) ch4 = 0;
  if (ch5AgeUs > 50000UL) ch5 = 0;
  
  // DEBUG: Check which channels are stale/missing
  if (ch2 == 0) Serial.print("[CH2 TIMEOUT] ");
  if (ch4 == 0) Serial.print("[CH4 TIMEOUT] ");
  if (ch5 == 0) Serial.print("[CH5 TIMEOUT] ");
  
  // Debug output - ALWAYS ENABLED to verify operation
  Serial.print("CH2: ");
  Serial.print(ch2);
  Serial.print("  CH4: ");
  Serial.print(ch4);
  Serial.print("  CH5: ");
  Serial.print(ch5);
  Serial.print(" | ");
  
  // Fail-safe: Stop only if BOTH CH2 and CH4 are missing (both read as 0 or out of range)
  // Allow individual channel timeouts (0 values) as long as at least one control channel is valid
  bool ch2_valid = (ch2 > 500 && ch2 < 2200);
  bool ch4_valid = (ch4 > 500 && ch4 < 2200);
  
  // Only stop if BOTH control channels (CH2 and CH4) are lost
  if (!ch2_valid && !ch4_valid) {
    Serial.println("SIGNAL LOST - MOTORS STOPPED!");
    stopMotors();
    return;
  }
  
  // If only one channel is valid, use its value and assume other is centered (0)
  if (!ch2_valid) ch2 = 1500;  // Default to center
  if (!ch4_valid) ch4 = 1500;  // Default to center
  if (ch5 < 500 || ch5 > 2200) ch5 = 1500;  // Default speed to middle
  
  // ========== Speed Mode Selection (CH5 - SWC Switch) ==========
  // SWC Position 1 (DOWN ~1000µs) = LOW 40%
  // SWC Position 2 (MIDDLE ~1500µs) = MEDIUM 70%
  // SWC Position 3 (UP ~2000µs) = FULL 100%
  String speedMode;
  if (ch5 > 1800) {                 // Position 3 UP: ~2000µs
    speedMultiplier = 1.0;           // 100% FULL POWER
    speedMode = "FULL-100%";
  } else if (ch5 < 1300) {          // Position 1 DOWN: ~1000µs
    speedMultiplier = 0.4;           // 40% LOW/TRAINING
    speedMode = "LOW-40%";
  } else {                           // Position 2 MIDDLE: ~1500µs
    speedMultiplier = 0.7;           // 70% MEDIUM/SPORT
    speedMode = "MED-70%";
  }
  Serial.print("Speed: ");
  Serial.print(speedMode);
  Serial.print(" | ");
  
  // ========== Map RC Values to Control Axes (-255 to +255) ==========
  int ch2ForControl = ch2;
  int ch4ForControl = ch4;
  if (SWAP_INPUT_CHANNELS) {
    int tempCh = ch2ForControl;
    ch2ForControl = ch4ForControl;
    ch4ForControl = tempCh;
  }

  int rawCh2 = map(ch2ForControl, RC_MIN, RC_MAX, -255, 255);
  int rawCh4 = map(ch4ForControl, RC_MIN, RC_MAX, -255, 255);

  if (INVERT_CH2) rawCh2 = -rawCh2;
  if (INVERT_CH4) rawCh4 = -rawCh4;

  int throttle;
  int steering;
  if (USE_ROTATED_MIX_DECODE) {
    // Decode FlySky elevon-style rotated mix:
    // rawCh2 ≈ (steering + throttle), rawCh4 ≈ (steering - throttle)
    throttle = (rawCh2 - rawCh4) / 2;
    steering = (rawCh2 + rawCh4) / 2;
  } else {
    // Direct mapping mode (for normal, non-rotated channel assignments)
    throttle = rawCh2;
    steering = rawCh4;
  }

  if (INVERT_DECODED_THROTTLE) throttle = -throttle;
  if (INVERT_DECODED_STEERING) steering = -steering;

  // Apply exponential curve for smooth control
  throttle = applyExponentialCurve(throttle, 255);
  steering = applyExponentialCurve(steering, 255);

  // Apply speed multiplier
  throttle = (int)(throttle * speedMultiplier);
  steering = (int)(steering * speedMultiplier);

  // Apply deadband to filter out joystick noise
  throttle = applyDeadband(throttle);
  steering = applyDeadband(steering);

  // ========== Arcade/Tank Mixing ==========
  // Forward/back drives both sides together, left/right creates differential turn
  int leftMotorSpeed;
  int rightMotorSpeed;
  if (ENABLE_PIVOT_TURN && abs(throttle) < 25 && abs(steering) > 25) {
    // Spot turn in place for cleaner left/right behavior
    leftMotorSpeed = steering;
    rightMotorSpeed = -steering;
  } else {
    leftMotorSpeed = throttle + steering;
    rightMotorSpeed = throttle - steering;
  }

  // Normalize so combined steering+throttle does not clip asymmetrically
  int maxMag = max(abs(leftMotorSpeed), abs(rightMotorSpeed));
  if (maxMag > 255) {
    leftMotorSpeed = (leftMotorSpeed * 255) / maxMag;
    rightMotorSpeed = (rightMotorSpeed * 255) / maxMag;
  }

  if (INVERT_LEFT_MOTOR) leftMotorSpeed = -leftMotorSpeed;
  if (INVERT_RIGHT_MOTOR) rightMotorSpeed = -rightMotorSpeed;
  
  // Constrain to valid range
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
  
  // Debug output - Show decoded controls and motor commands
  Serial.print("T: ");
  Serial.print(throttle);
  Serial.print("  S: ");
  Serial.print(steering);
  Serial.print(" | ");
  Serial.print("L: ");
  Serial.print(leftMotorSpeed);
  Serial.print("  R: ");
  Serial.println(rightMotorSpeed);
  
  // Drive motors
  if (throttle != 0 || steering != 0) {
    driveMotors(leftMotorSpeed, rightMotorSpeed);
  } else {
    stopMotors();
  }
  
  // Small delay for stability
  delay(50);  // 50ms = 20Hz loop rate (Arduino Uno is slower)
}

/*
TUNING NOTES FOR ARDUINO UNO:
=============================

1. PWM FREQUENCY:
   - Arduino Uno default: ~490Hz on pins 5,6 and ~980Hz on pins 3,9,10
   - BTS7960 works fine with this frequency
   - If you need to change PWM frequency, use Timer manipulation (advanced)

2. CONTROL RESPONSIVENESS:
   - Increase EXPO_CURVE (2.0-3.0) if control feels jerky at low speeds
   - Decrease EXPO_CURVE (1.0-1.5) if control feels too sensitive

3. DEADBAND:
   - Increase DEADBAND_THRESHOLD (50-100) if robot drifts when sticks centered
   - Decrease if you want more responsive low-speed control

4. LOOP DELAY:
   - Current: 50ms = 20Hz update rate
  - Arduino Uno is slower than ESP32, so 50ms is reasonable
  - Interrupt capture is non-blocking, so channel reads stay stable

5. SERIAL BAUD RATE:
   - Arduino Uno: 9600 baud (kept simple for stability)
   - Can increase to 115200 if your USB-Serial adapter supports it

6. MOTOR SPEED LIMITS:
   - MIN_MOTOR_SPEED: Set to 100+ if motors don't start at low PWM values
   - MAX_PWM: Currently set to 255 (full speed)

TROUBLESHOOTING:
================

If motors don't respond:
- Check CH2/CH4/CH5 debug values are in range 1000-2000
- Verify receiver is powered and transmitter is on
- Check BTS7960 enable pins are connected to +5V (constant)

If only one motor works:
- Check PWM pin connections (5, 6, 9, 10)
- Verify motor driver pin assignments
- Check battery voltage (12V recommended)

If control is jerky:
- Increase EXPO_CURVE value
- Increase DEADBAND_THRESHOLD
- Check if loop delay is too short

If motors spin wrong direction:
- Swap M+ and M- connections on that motor
- Or swap RPWM/LPWM pins for that motor
*/
