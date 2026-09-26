#include <Bluepad32.h>
#include "soc/soc.h"          // ESP32 power control
#include "soc/rtc_cntl_reg.h" // ESP32 brownout detector register

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ── ONBOARD STATUS LED ───────────────────────────────────────────────────────
// Most ESP32 boards have an onboard blue LED on GPIO 2.
// Lights up solid when gamepad is connected (great for testing without PC!)
const int LED_STATUS = 2;

// ── MOTOR PINS (All on the RIGHT side of 38-pin ESP32 NodeMCU) ───────────────
// Clustered together on the exact same side as 30D (GPIO 32, 33, 25, 26):
//   GPIO 32 ──► Left  Motor RPWM (Forward)
//   GPIO 33 ──► Left  Motor LPWM (Reverse)
//   GPIO 25 ──► Right Motor RPWM (Forward)
//   GPIO 26 ──► Right Motor LPWM (Reverse)
//   GND     ──► BTS7960 GND (Common ground)
//   (BTS R_EN & L_EN tied to 5V or 3.3V)
const int ML_RPWM = 32; // Left  motor forward PWM (Right side)
const int ML_LPWM = 33; // Left  motor reverse PWM (Right side)
const int MR_RPWM = 25; // Right motor forward PWM (Right side)
const int MR_LPWM = 26; // Right motor reverse PWM (Right side)

// ── PWM CONFIG & ESP32 CORE 2.x / 3.x COMPATIBILITY ──────────────────────────
const int freq = 20000; // 20kHz = silent, high-torque switching for BTS7960
const int resolution = 8;
const int pwmChannelML_RPWM = 0;
const int pwmChannelML_LPWM = 1;
const int pwmChannelMR_RPWM = 2;
const int pwmChannelMR_LPWM = 3;

#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // ESP32 Arduino Core 3.x+ API
  #define PWM_SETUP(pin, ch)      ledcAttach(pin, freq, resolution)
  #define PWM_WRITE(pin, ch, val) ledcWrite(pin, val)
#else
  // ESP32 Arduino Core 2.x legacy API
  #define PWM_SETUP(pin, ch)      do { ledcSetup(ch, freq, resolution); ledcAttachPin(pin, ch); } while(0)
  #define PWM_WRITE(pin, ch, val) ledcWrite(ch, val)
#endif

// ── TUNING & SPEED MODES ─────────────────────────────────────────────────────
const int TRIGGER_THRESHOLD = 80;
const int TRIGGER_MAX = 1020;

// B mode (HIGH POWER - 100%)
const int   JOY_DZ_HIGH  = 35;
const float EXPO_HIGH    = 1.0;   // 1.0 = Perfectly linear, instant FlySky feel
const int   MAX_PWM_HIGH = 255;

// Y mode (MID POWER - ~60%)
const int   JOY_DZ_MID   = 35;
const float EXPO_MID     = 1.0;
const int   MAX_PWM_MID  = 153;

// A mode (LOW POWER - 50%)
const int   JOY_DZ_LOW   = 35;
const float EXPO_LOW     = 1.0;
const int   MAX_PWM_LOW  = 127;

// ── ANTI-BROWNOUT SLEW RATE LIMITER ──────────────────────────────────────────
// Prevents massive 80A+ stall current spikes when slamming forward to reverse,
// which would otherwise cause battery voltage sag and reset the ESP32 / drop Bluetooth.
// 2.5 PWM/ms = ramps 0 to 255 in ~100ms (feels completely instant to fingers, but protects ESP32)
const float SLEW_RATE_PWM_PER_MS = 2.5f;
static float currentLeftSpeed  = 0.0f;
static float currentRightSpeed = 0.0f;
static int   targetLeftSpeed   = 0;
static int   targetRightSpeed  = 0;
static unsigned long lastSlewMs = 0;

// Failsafe: auto-stop if no Bluetooth data arrives for this long
const unsigned long FAILSAFE_MS = 500;

// Set true once to clear stored paired-controller keys, then set back to false
const bool FORGET_PAIRED_KEYS_ON_BOOT = false;

// ─────────────────────────────────────────────────────────────────────────────

static unsigned long lastDataMs = 0; // watchdog timestamp

// ── EXPO CURVE ───────────────────────────────────────────────────────────────
int applyExpo(int value, int maxVal, float curve) {
  if (value == 0)
    return 0;
  float norm = (float)abs(value) / (float)maxVal;
  norm = constrain(norm, 0.0f, 1.0f);
  float curved = pow(norm, curve);
  int result = (int)(curved * maxVal);
  return (value > 0) ? result : -result;
}

// ── LOW-LEVEL MOTOR HARDWARE WRITE ───────────────────────────────────────────
void applyMotorPWM(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    PWM_WRITE(ML_LPWM, pwmChannelML_LPWM, 0);
    PWM_WRITE(ML_RPWM, pwmChannelML_RPWM, leftSpeed);
  } else if (leftSpeed < 0) {
    PWM_WRITE(ML_RPWM, pwmChannelML_RPWM, 0);
    PWM_WRITE(ML_LPWM, pwmChannelML_LPWM, abs(leftSpeed));
  } else {
    // Dynamic Braking: Both low-side MOSFETs conduct to GND when R_EN/L_EN are HIGH
    PWM_WRITE(ML_RPWM, pwmChannelML_RPWM, 0);
    PWM_WRITE(ML_LPWM, pwmChannelML_LPWM, 0);
  }

  // Right motor
  if (rightSpeed > 0) {
    PWM_WRITE(MR_LPWM, pwmChannelMR_LPWM, 0);
    PWM_WRITE(MR_RPWM, pwmChannelMR_RPWM, rightSpeed);
  } else if (rightSpeed < 0) {
    PWM_WRITE(MR_RPWM, pwmChannelMR_RPWM, 0);
    PWM_WRITE(MR_LPWM, pwmChannelMR_LPWM, abs(rightSpeed));
  } else {
    // Dynamic Braking: Both low-side MOSFETs conduct to GND when R_EN/L_EN are HIGH
    PWM_WRITE(MR_RPWM, pwmChannelMR_RPWM, 0);
    PWM_WRITE(MR_LPWM, pwmChannelMR_LPWM, 0);
  }
}

// ── HIGH-LEVEL MOTOR TARGETS & SLEW UPDATE ───────────────────────────────────
void setTargets(int left, int right) {
  targetLeftSpeed  = left;
  targetRightSpeed = right;
}

void Stop() {
  setTargets(0, 0);
}

// Microsecond slew rate update loop — called continuously in loop()
void updateMotorSlew() {
  unsigned long now = millis();
  float dt = (float)(now - lastSlewMs);
  if (dt < 2.0f) return; // Update every 2ms minimum for fine resolution
  lastSlewMs = now;

  float maxStep = SLEW_RATE_PWM_PER_MS * dt;

  // Slew Left Motor
  if (targetLeftSpeed > currentLeftSpeed) {
    currentLeftSpeed = min(currentLeftSpeed + maxStep, (float)targetLeftSpeed);
  } else if (targetLeftSpeed < currentLeftSpeed) {
    currentLeftSpeed = max(currentLeftSpeed - maxStep, (float)targetLeftSpeed);
  }

  // Slew Right Motor
  if (targetRightSpeed > currentRightSpeed) {
    currentRightSpeed = min(currentRightSpeed + maxStep, (float)targetRightSpeed);
  } else if (targetRightSpeed < currentRightSpeed) {
    currentRightSpeed = max(currentRightSpeed - maxStep, (float)targetRightSpeed);
  }

  applyMotorPWM((int)round(currentLeftSpeed), (int)round(currentRightSpeed));
}

// ── BLUETOOTH CALLBACKS ──────────────────────────────────────────────────────
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Controller connected at index=%d\n", i);
      ControllerProperties p = ctl->getProperties();
      Serial.printf("Model: %s, VID=0x%04x, PID=0x%04x\n",
                    ctl->getModelName().c_str(), p.vendor_id, p.product_id);
      myControllers[i] = ctl;
      digitalWrite(LED_STATUS, HIGH); // Solid ON when connected!
      // Welcome rumble (short double buzz)
      ctl->playDualRumble(0 /* delay */, 150 /* duration */, 180 /* weak */, 180 /* strong */);
      return;
    }
  }
  Serial.println("Controller connected but no empty slot found");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      digitalWrite(LED_STATUS, LOW);
      Stop();
      return;
    }
  }
  Serial.println("Controller disconnected but not found");
}

// ── GAMEPAD PROCESSING ───────────────────────────────────────────────────────
void processGamepad(ControllerPtr ctl) {
  int16_t axisX  = ctl->axisX();    // Left Stick X (-512 Left to +511 Right)
  int16_t axisRY = ctl->axisRY();   // Right Stick Y (-512 Forward to +511 Backward)
  int throttle   = ctl->throttle(); // R2 Trigger (0 to 1020)
  int brake      = ctl->brake();    // L2 Trigger (0 to 1020)
  bool bBtn      = ctl->b();
  bool aBtn      = ctl->a();
  bool yBtn      = ctl->y();
  bool xBtn      = ctl->x();

  // Update failsafe watchdog
  lastDataMs = millis();

  // ── MODE SELECT (rising edge with Haptic Rumble) ──────────────────────
  // 2 = HIGH (B, 100%) | 1 = MID (Y, 60%) | 0 = LOW (A, 50%)
  static int mode = 2; // default = HIGH POWER
  static bool bWasDown = false;
  static bool aWasDown = false;
  static bool yWasDown = false;
  static bool xWasDown = false;

  if (bBtn && !bWasDown) {
    mode = 2;
    Serial.println("HIGH POWER 100% (B)");
    ctl->playDualRumble(0, 180, 200, 255); // Strong buzz
  }
  if (yBtn && !yWasDown) {
    mode = 1;
    Serial.println("MID  POWER   60% (Y)");
    ctl->playDualRumble(0, 130, 140, 0);   // Medium buzz
  }
  if (aBtn && !aWasDown) {
    mode = 0;
    Serial.println("LOW  POWER   50% (A)");
    ctl->playDualRumble(0, 90, 80, 0);     // Gentle tap
  }
  bWasDown = bBtn;
  yWasDown = yBtn;
  aWasDown = aBtn;

  // ── DIRECTION FLIP toggle (X button with Haptic Rumble) ───────────────
  static bool flipped = false;
  if (xBtn && !xWasDown) {
    flipped = !flipped;
    Serial.printf("DIRECTION: %s (X)\n", flipped ? "FLIPPED" : "NORMAL");
    ctl->playDualRumble(0, 220, 255, 120); // Distinct pattern
  }
  xWasDown = xBtn;

  // Pick parameters for current mode
  const int MAX_PWM = (mode == 2)   ? MAX_PWM_HIGH
                      : (mode == 1) ? MAX_PWM_MID
                                    : MAX_PWM_LOW;
  const float EXPO = (mode == 2)   ? EXPO_HIGH
                     : (mode == 1) ? EXPO_MID
                                   : EXPO_LOW;
  const int JOY_DZ = (mode == 2) ? JOY_DZ_HIGH : JOY_DZ_LOW;

  int baseSpeed = 0, steeringValue = 0;

  // ── THROTTLE / REVERSE ────────────────────────────────────────────────
  // Controls front/back using either Right Joystick Y or Triggers (R2/L2):
  if (abs(axisRY) > JOY_DZ) {
    // Right Joystick: push forward (negative) = Forward, pull back = Reverse
    int s = map(abs(axisRY), JOY_DZ, 512, 0, 255);
    s = constrain(s, 0, 255);
    s = applyExpo(s, 255, EXPO);
    s = constrain(s, 0, MAX_PWM);
    baseSpeed = (axisRY < 0) ? s : -s;
  } else if (throttle > TRIGGER_THRESHOLD) {
    // R2 Trigger = Forward
    int s = map(throttle, TRIGGER_THRESHOLD, TRIGGER_MAX, 0, 255);
    s = constrain(s, 0, 255);
    s = applyExpo(s, 255, EXPO);
    baseSpeed = constrain(s, 0, MAX_PWM);
  } else if (brake > TRIGGER_THRESHOLD) {
    // L2 Trigger = Reverse
    int s = map(brake, TRIGGER_THRESHOLD, TRIGGER_MAX, 0, 255);
    s = constrain(s, 0, 255);
    s = applyExpo(s, 255, EXPO);
    baseSpeed = -constrain(s, 0, MAX_PWM);
  }

  // ── STEERING (Left Joystick X) ────────────────────────────────────────
  if (abs(axisX) > JOY_DZ) {
    int mapped = map(abs(axisX), JOY_DZ, 512, 0, 255);
    mapped = constrain(mapped, 0, 255);
    mapped = applyExpo(mapped, 255, EXPO);
    mapped = constrain(mapped, 0, MAX_PWM);
    steeringValue = (axisX > 0) ? mapped : -mapped; // Right = positive, Left = negative
  }

  // Apply direction flip (X toggle)
  if (flipped) {
    baseSpeed = -baseSpeed;
    steeringValue = -steeringValue;
  }

  // ── ARCADE DRIVE MIX (Tank / 4-Wheel Skid Steer) ───────────────────────
  int leftMotorSpeed  = constrain(baseSpeed + steeringValue, -MAX_PWM, MAX_PWM);
  int rightMotorSpeed = constrain(baseSpeed - steeringValue, -MAX_PWM, MAX_PWM);

  if (baseSpeed != 0 || steeringValue != 0)
    setTargets(leftMotorSpeed, rightMotorSpeed);
  else
    Stop();

  // ── LIVE DEBUG PRINT (every 100ms) ───────────────────────────────────
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.printf("[RAW] axisX=%5d  axisRY=%5d  R2=%4d  L2=%4d  "
                  "A=%d B=%d X=%d Y=%d\n"
                  "[OUT] mode=%-4s  baseSpeed=%4d  steerVal=%4d  "
                  "T_L=%4d  T_R=%4d  C_L=%4.0f  C_R=%4.0f\n\n",
                  (int)axisX, (int)axisRY, throttle, brake,
                  ctl->a(), ctl->b(), ctl->x(), ctl->y(),
                  (mode == 2)   ? "HIGH"
                  : (mode == 1) ? "MID"
                                : "LOW",
                  baseSpeed, steeringValue,
                  leftMotorSpeed, rightMotorSpeed,
                  currentLeftSpeed, currentRightSpeed);
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() &&
        myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  // 1. Disable hardware brownout detector (prevents resets on 5V battery/buck converter transients)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // 2. Allow 5V power rail to fully charge decoupling caps and stabilize
  delay(800);

  // 3. Status LED setup (GPIO 2)
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  // Configure and attach PWM channels (Supports both Core 2.x and Core 3.x)
  PWM_SETUP(ML_RPWM, pwmChannelML_RPWM);
  PWM_SETUP(ML_LPWM, pwmChannelML_LPWM);
  PWM_SETUP(MR_RPWM, pwmChannelMR_RPWM);
  PWM_SETUP(MR_LPWM, pwmChannelMR_LPWM);

  Stop();

  BP32.setup(&onConnectedController, &onDisconnectedController);

  if (FORGET_PAIRED_KEYS_ON_BOOT) {
    Serial.println("Clearing saved controller keys...");
    BP32.forgetBluetoothKeys();
    Serial.println("Done. Pair your controller again.");
  }

  Serial.println("\n=== Robo Soccer Ready (BTS7960 Driver) ===");
  Serial.println("  Pins (All on RIGHT side of ESP32):");
  Serial.println("    GPIO 32 ──► Left  Motor RPWM (Forward)");
  Serial.println("    GPIO 33 ──► Left  Motor LPWM (Reverse)");
  Serial.println("    GPIO 25 ──► Right Motor RPWM (Forward)");
  Serial.println("    GPIO 26 ──► Right Motor LPWM (Reverse)");
  Serial.println("    GND     ──► BTS7960 GND (Common ground)");
  Serial.println("  Controls:");
  Serial.println("    Left Joystick (X)             = Steer Left / Right");
  Serial.println("    Right Joystick (Y) or R2/L2   = Forward / Reverse");
  Serial.println("    B = HIGH (100%) | Y = MID (60%) | A = LOW (50%)");
  Serial.println("    X = Flip Direction");
  Serial.println("  Features: Dynamic Braking + Slew-Rate Limiter + Haptic Rumble");
  Serial.printf("  Failsafe stop after %lums of no signal\n", FAILSAFE_MS);
  Serial.println("\nPairing mode: press PS/Home on controller...");
}

// ── LOOP ─────────────────────────────────────────────────────────────────────
void loop() {
  // Failsafe: stop motors if no Bluetooth data for FAILSAFE_MS
  if (lastDataMs != 0 && (millis() - lastDataMs > FAILSAFE_MS)) {
    Stop();
  }

  if (BP32.update()) {
    processControllers();
  }

  // Visual Status LED: Blinking = waiting for gamepad, Solid ON = Gamepad connected
  static unsigned long lastBlink = 0;
  bool anyConnected = false;
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected()) anyConnected = true;
  }
  if (!anyConnected) {
    if (millis() - lastBlink >= 400) {
      lastBlink = millis();
      digitalWrite(LED_STATUS, !digitalRead(LED_STATUS)); // Blink while searching
    }
  } else {
    digitalWrite(LED_STATUS, HIGH); // Solid ON when paired!
  }

  // Continuous micro-slew rate limiter (protects ESP32 from brownouts)
  updateMotorSlew();
}


