// FlySky Receiver Input Pins
#define CH1_PIN 35  // Forward/Backward
#define CH2_PIN 34  // Left/Right
#define CH5_PIN 39  // Speed Mode toggle Switch

// BTS7960 Right Motor Pins
#define RPWM 14    // Right Motor RPWM
#define RLPWM 27   // Right Motor LPWM

// BTS7960 Left Motor Pins
#define LPWM 32    // Left Motor RPWM
#define LLPWM 33   // Left Motor LPWM

// PWM Channels
#define LPWM_CH 0
#define LLPWM_CH 1
#define RPWM_CH 2
#define RLPWM_CH 3

float speedMultiplier = 1.0;  // Initially Full Speed

int applyDeadband(int value, int threshold = 10) {
  return (abs(value) < threshold) ? 0 : value;
}

void setup() {
  Serial.begin(115200);

  // Set receiver pins
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);

  // Set BTS7960 motor pins as OUTPUT
  pinMode(LPWM, OUTPUT);
  pinMode(LLPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(RLPWM, OUTPUT);

  // Setup PWM channels
  ledcSetup(LPWM_CH, 1000, 8);
  ledcAttachPin(LPWM, LPWM_CH);

  ledcSetup(LLPWM_CH, 1000, 8);
  ledcAttachPin(LLPWM, LLPWM_CH);

  ledcSetup(RPWM_CH, 1000, 8);
  ledcAttachPin(RPWM, RPWM_CH);

  ledcSetup(RLPWM_CH, 1000, 8);
  ledcAttachPin(RLPWM, RLPWM_CH);

  stopMotors();
}

void loop() {
  int ch1 = pulseIn(CH1_PIN, HIGH, 25000);  // Forward/Backward
  int ch2 = pulseIn(CH2_PIN, HIGH, 25000);  // Left/Right
  int ch5 = pulseIn(CH5_PIN, HIGH, 25000);  // Speed Mode Switch

  if (ch1 < 500 || ch1 > 2200 || ch2 < 500 || ch2 > 2200) {
    stopMotors();
    return;
  }

  speedMultiplier = (ch5 < 1500) ? 1.0 : 0.5; //0.5 = 50% Speed, 0.7 = 70%, 0.8 = 80%

  int x = map(ch1, 1000, 2000, -255, 255) * speedMultiplier;
  int y = map(ch2, 1000, 2000, -255, 255) * speedMultiplier;

  x = applyDeadband(x);
  y = applyDeadband(y);

  int leftSpeed, rightSpeed;
  if (y <= -150) {
    leftSpeed = y - x;
    rightSpeed = y + x;
  } else {
    leftSpeed = y + x;
    rightSpeed = y - x;
  }

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotor(LPWM_CH, LLPWM_CH, leftSpeed);
  setMotor(RPWM_CH, RLPWM_CH, rightSpeed);
}

void setMotor(uint8_t fwd_ch, uint8_t rev_ch, int speed) {
  if (speed > 0) {
    ledcWrite(fwd_ch, speed);
    ledcWrite(rev_ch, 0);
  } else if (speed < 0) {
    ledcWrite(fwd_ch, 0);
    ledcWrite(rev_ch, -speed);
  } else {
    ledcWrite(fwd_ch, 0);
    ledcWrite(rev_ch, 0);
  }
}

void stopMotors() {
  setMotor(LPWM_CH, LLPWM_CH, 0);
  setMotor(RPWM_CH, RLPWM_CH, 0);
}

