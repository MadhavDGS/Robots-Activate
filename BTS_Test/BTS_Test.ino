#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ── BTS7960 MOTOR PINS (Right side of 38-Pin NodeMCU) ────────────────────────
const int ML_RPWM = 32; // Left  Motor Forward
const int ML_LPWM = 33; // Left  Motor Reverse
const int MR_RPWM = 25; // Right Motor Forward
const int MR_LPWM = 26; // Right Motor Reverse

const int freq = 20000;
const int resolution = 8;
const int pwmML_RPWM = 0;
const int pwmML_LPWM = 1;
const int pwmMR_RPWM = 2;
const int pwmMR_LPWM = 3;

// ESP32 Arduino Core 2.x and Core 3.x compatibility macros
#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define PWM_SETUP(pin, ch)      ledcAttach(pin, freq, resolution)
  #define PWM_WRITE(pin, ch, val) ledcWrite(pin, val)
#else
  #define PWM_SETUP(pin, ch)      do { ledcSetup(ch, freq, resolution); ledcAttachPin(pin, ch); } while(0)
  #define PWM_WRITE(pin, ch, val) ledcWrite(ch, val)
#endif

void stopAll() {
  PWM_WRITE(ML_RPWM, pwmML_RPWM, 0);
  PWM_WRITE(ML_LPWM, pwmML_LPWM, 0);
  PWM_WRITE(MR_RPWM, pwmMR_RPWM, 0);
  PWM_WRITE(MR_LPWM, pwmMR_LPWM, 0);
}

void setLeftMotor(int speed) {
  if (speed > 0) {
    PWM_WRITE(ML_LPWM, pwmML_LPWM, 0);
    PWM_WRITE(ML_RPWM, pwmML_RPWM, speed);
  } else if (speed < 0) {
    PWM_WRITE(ML_RPWM, pwmML_RPWM, 0);
    PWM_WRITE(ML_LPWM, pwmML_LPWM, abs(speed));
  } else {
    PWM_WRITE(ML_RPWM, pwmML_RPWM, 0);
    PWM_WRITE(ML_LPWM, pwmML_LPWM, 0);
  }
}

void setRightMotor(int speed) {
  if (speed > 0) {
    PWM_WRITE(MR_LPWM, pwmMR_LPWM, 0);
    PWM_WRITE(MR_RPWM, pwmMR_RPWM, speed);
  } else if (speed < 0) {
    PWM_WRITE(MR_RPWM, pwmMR_RPWM, 0);
    PWM_WRITE(MR_LPWM, pwmMR_LPWM, abs(speed));
  } else {
    PWM_WRITE(MR_RPWM, pwmMR_RPWM, 0);
    PWM_WRITE(MR_LPWM, pwmMR_LPWM, 0);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout
  delay(500);

  Serial.begin(115200);
  delay(500);
  Serial.println("\n===========================================");
  Serial.println("    BTS7960 INDEPENDENT MOTOR TEST SCRIPT  ");
  Serial.println("===========================================");
  Serial.println("Pins used:");
  Serial.println("  Left Motor  RPWM (FWD) = GPIO 32");
  Serial.println("  Left Motor  LPWM (REV) = GPIO 33");
  Serial.println("  Right Motor RPWM (FWD) = GPIO 25");
  Serial.println("  Right Motor LPWM (REV) = GPIO 26");
  Serial.println("Requirements for BTS7960 to spin:");
  Serial.println("  1. R_EN and L_EN must be connected to 5V!");
  Serial.println("  2. BTS VCC = 5V, BTS GND = ESP32 GND");
  Serial.println("  3. Motor battery connected to B+ and B-");
  Serial.println("===========================================\n");

  PWM_SETUP(ML_RPWM, pwmML_RPWM);
  PWM_SETUP(ML_LPWM, pwmML_LPWM);
  PWM_SETUP(MR_RPWM, pwmMR_RPWM);
  PWM_SETUP(MR_LPWM, pwmMR_LPWM);

  stopAll();
  delay(1000);
}

void loop() {
  // TEST 1: Left Motor Forward
  Serial.println(">>> [1/6] LEFT Motor FORWARD (GPIO 32 = 100%)... Check Left BTS LED!");
  setLeftMotor(255);
  setRightMotor(0);
  delay(2500);
  stopAll();
  delay(1000);

  // TEST 2: Left Motor Reverse
  Serial.println(">>> [2/6] LEFT Motor REVERSE (GPIO 33 = 100%)... Check Left BTS LED!");
  setLeftMotor(-255);
  setRightMotor(0);
  delay(2500);
  stopAll();
  delay(1000);

  // TEST 3: Right Motor Forward
  Serial.println(">>> [3/6] RIGHT Motor FORWARD (GPIO 25 = 100%)... Check Right BTS LED!");
  setLeftMotor(0);
  setRightMotor(255);
  delay(2500);
  stopAll();
  delay(1000);

  // TEST 4: Right Motor Reverse
  Serial.println(">>> [4/6] RIGHT Motor REVERSE (GPIO 26 = 100%)... Check Right BTS LED!");
  setLeftMotor(0);
  setRightMotor(-255);
  delay(2500);
  stopAll();
  delay(1000);

  // TEST 5: Both Motors Forward
  Serial.println(">>> [5/6] BOTH Motors FORWARD (GPIO 32 + 25 = 100%)...");
  setLeftMotor(255);
  setRightMotor(255);
  delay(2500);
  stopAll();
  delay(1000);

  // TEST 6: Both Motors Reverse
  Serial.println(">>> [6/6] BOTH Motors REVERSE (GPIO 33 + 26 = 100%)...");
  setLeftMotor(-255);
  setRightMotor(-255);
  delay(2500);
  stopAll();
  delay(2000);

  Serial.println("\n--- Cycle Complete. Repeating in 2 seconds ---\n");
}
