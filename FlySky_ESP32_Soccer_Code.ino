/*
Flysky+ESP32 Soccer Bot Code by RoboTech Innovator BD:

𝐂𝐎𝐍𝐓𝐀𝐂𝐓:
---------------------------------------------
𝑬-𝒎𝒂𝒊𝒍: aslamhshakil20@gmail.com
𝐈𝐧𝐬𝐭𝐚𝐠𝐫𝐚𝐦: https://www.instagram.com/robotech_innovator?igsh=dHUzNTBtajcyNnJv
𝑭𝒂𝒄𝒆𝒃𝒐𝒐𝒌: https://www.facebook.com/RoboTechInnovator/
𝑾𝒉𝒂𝒕'𝒔 𝑨𝒑𝒑: https://wa.me/c/8801647467658
𝑻𝒆𝒍𝒆𝒈𝒓𝒂𝒎: https://t.me/+NmZDetFcTvA0MGU1

ESP32 Board Link: https://dl.espressif.com/dl/package_esp32_index.json
Goto File > Preferences > Additional Board Manager URLs: (Paste the Link).
Then goto board manager and search for "esp32". then install the esp32 board version of 2.0.11
otherwise the code will not be compiled.
*/

// FlySky Receiver Input Pins
#define CH1_PIN 39  // Forward/Backward control signal input pin from FlySky receiver
#define CH2_PIN 34  // Left/Right control signal input pin from FlySky receiver
#define CH5_PIN 35  // Speed Mode toggle Switch signal input pin from FlySky receiver

// BTS7960 Right Motor Pins
#define RPWM 26   // Right Motor Forward pin (RPWM of BTS7960)
#define RLPWM 25  // Right Motor Reverse pin (LPWM of BTS7960)

// BTS7960 Left Motor Pins
#define LPWM 14   // Left Motor Forward pin (RPWM of BTS7960)
#define LLPWM 27  // Left Motor Reverse pin (LPWM of BTS7960)

// PWM Channels (ESP32 hardware PWM channels, 0-15 available)
#define LPWM_CH 0   // PWM channel for left motor forward
#define LLPWM_CH 1  // PWM channel for left motor reverse
#define RPWM_CH 2   // PWM channel for right motor forward
#define RLPWM_CH 3  // PWM channel for right motor reverse

// RC signal limits
int max_value = 2000;  // Maximum expected pulse width (µs) from receiver
int mid_value = 1500;  // Midpoint pulse width (µs) for neutral stick
int min_value = 1000;  // Minimum expected pulse width (µs) from receiver

float speedMultiplier = 1.0;  // Speed scaling factor, initially full speed (100%)

// Function to apply deadband to joystick values
// If the absolute value is below threshold, return 0 (ignore small noise)
int applyDeadband(int value, int threshold = 30) {
  return (abs(value) < threshold) ? 0 : value;
}

void setup() {
  Serial.begin(115200);  // Initialize serial communication for debugging

  // Configure receiver pins as inputs
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);

  // Configure BTS7960 motor driver pins as outputs
  pinMode(LPWM, OUTPUT);
  pinMode(LLPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(RLPWM, OUTPUT);

  // Setup PWM for left motor forward channel
  ledcSetup(LPWM_CH, 1000, 8);   // 1 kHz frequency, 8-bit resolution
  ledcAttachPin(LPWM, LPWM_CH);  // Attach pin to PWM channel
  // Setup PWM for left motor reverse channel
  ledcSetup(LLPWM_CH, 1000, 8);
  ledcAttachPin(LLPWM, LLPWM_CH);

  // Setup PWM for right motor forward channel
  ledcSetup(RPWM_CH, 1000, 8);
  ledcAttachPin(RPWM, RPWM_CH);
  // Setup PWM for right motor reverse channel
  ledcSetup(RLPWM_CH, 1000, 8);
  ledcAttachPin(RLPWM, RLPWM_CH);

  stopMotors();  // Ensure motors are stopped at startup
}

void loop() {
  /* Read RC channel values using pulseIn (timeout 25 ms)
  1. pin → The GPIO you are measuring.
  Example: CH1_PIN = 35 (receiver output wire for channel 1).
  2. state → Whether to measure the pulse width of HIGH or LOW.
  It's always use HIGH with RC receivers.
  That means: wait until the signal goes HIGH, start counting, then stop when it goes LOW.
  3. timeout → Maximum time to wait (in microseconds).

  If no pulse is received within this time, the function returns 0.
  In my case: 25000 µs = 25 ms.
  Since the RC signal repeats every ~20 ms, this is enough to catch one frame.
*/
  int ch1 = pulseIn(CH1_PIN, HIGH, 25000);  // Forward/Backward stick input
  int ch2 = pulseIn(CH2_PIN, HIGH, 25000);  // Left/Right stick input
  int ch5 = pulseIn(CH5_PIN, HIGH, 25000);  // Speed Mode switch input

  //Serial.print("CH1: " + String(ch1) + "  CH2: " + String(ch2) + "  CH5: " + String(ch5));
  //Serial.println();

  // Fail-safe: if values are out of expected range, stop motors
  if (ch1 < 500 || ch1 > 2200 || ch2 < 500 || ch2 > 2200) {
    stopMotors();
    return;
  }

  // Speed mode selection based on CH5 switch position
  if (ch5 < min_value + 200) {         // If less than ~1200 µs
    speedMultiplier = 0.5;             // Limit speed to 50%
  } else if (ch5 > mid_value + 200) {  // If greater than ~1700 µs
    speedMultiplier = 0.8;             // Limit speed to 80%
  } else {
    speedMultiplier = 1.0;  // Default: 100% speed at middle position
  }

  // Map RC values (1000–2000 µs) to motor speed range (-255 to +255)
  int x = map(ch1, min_value, max_value, -255, 255) * speedMultiplier;  // Forward/Backward
  int y = map(ch2, min_value, max_value, -255, 255) * speedMultiplier;  // Left/Right

  // Apply deadband filtering to avoid small jitter
  x = applyDeadband(x);
  y = applyDeadband(y);

  // Mixing algorithm to calculate left and right motor speeds
  int leftSpeed, rightSpeed;
  if (y <= -150) {       // If turning strongly left
    leftSpeed = y - x;   // Reduce left motor speed
    rightSpeed = y + x;  // Increase right motor speed
  } else {               // Normal forward/backward + turning mix
    leftSpeed = y + x;
    rightSpeed = y - x;
  }

  // Constrain speeds to valid PWM range (-255 to +255)
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  //Serial.print("Left PWM: " + String(leftSpeed) + "   Right PWM: " + String(rightSpeed));
  //Serial.println();

  // Apply motor speeds to left and right motors
  motorDrive(LPWM_CH, LLPWM_CH, leftSpeed);
  motorDrive(RPWM_CH, RLPWM_CH, rightSpeed);
}

// Function to control a motor with forward & reverse PWM channels
void motorDrive(uint8_t fwd_ch, uint8_t rev_ch, int speed) {
  if (speed > 0) {              // Forward direction
    ledcWrite(fwd_ch, speed);   // Apply PWM to forward channel
    ledcWrite(rev_ch, 0);       // Reverse channel off
  } else if (speed < 0) {       // Reverse direction
    ledcWrite(fwd_ch, 0);       // Forward channel off
    ledcWrite(rev_ch, -speed);  // Apply PWM to reverse channel
  } else {                      // Stop motor
    ledcWrite(fwd_ch, 0);
    ledcWrite(rev_ch, 0);
  }
}

// Function to stop both motors
void stopMotors() {
  motorDrive(LPWM_CH, LLPWM_CH, 0);  // Stop left motor
  motorDrive(RPWM_CH, RLPWM_CH, 0);  // Stop right motor
}
