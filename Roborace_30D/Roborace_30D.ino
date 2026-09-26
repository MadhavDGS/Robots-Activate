#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ── MOTOR PINS (Smartelex 30D — RC / Servo Signal Mode) ──────────────────────
// Wire: GPIO 32 → RC1 Signal (white)   GPIO 33 → RC2 Signal (white)
//       GND → RC1/RC2 GND (black)      RC1 5V (red) → ESP32 VIN
// DIP:  Sw1=OFF  Sw2=OFF  Sw3=OFF  Sw4=ON  (RC Linear Independent)
// Both pins: RIGHT side of 38-pin NodeMCU, adjacent to GPIO 25/26
// GPIO 32 & 33 are 100% safe — not strapping pins, not flash pins
const int RC_LEFT = 32;  // Left  motor → RC1 signal
const int RC_RIGHT = 33; // Right motor → RC2 signal

// ── SERVO PWM CONFIG ────────────────────────────────────────────────────
// 30D ONLY supports standard 50Hz RC signals stably.
const int SERVO_FREQ = 50;       // Hz
const int SERVO_RESOLUTION = 16; // 16-bit = 65536 counts per period

const int CHANNEL_LEFT = 0;
const int CHANNEL_RIGHT = 1;

// Pulse widths in LEDC counts (at 16-bit / 50Hz, period = 20000µs)
// count = (pulse_us / 20000) * 65536
const int SERVO_MIN = 3277;    // 1.0ms → full reverse
const int SERVO_CENTER = 4915; // 1.5ms → stopped
const int SERVO_MAX = 6554;    // 2.0ms → full forward

// ── TUNING ───────────────────────────────────────────────────────────────────
const int TRIGGER_THRESHOLD = 100; // Increased to prevent resting trigger drift
const int TRIGGER_MAX = 1020;

// B mode (HIGH POWER)
const int JOY_DZ_HIGH = 35; // Increased to prevent analog stick drift
const float EXPO_HIGH = 1.0; // 1.0 = PERFECTLY LINEAR (like FlySky)
const int MAX_PWM_HIGH = 255;

// Y mode (MID POWER)
const int JOY_DZ_MID = 35;
const float EXPO_MID = 1.0;
const int MAX_PWM_MID = 153; // ~60%

// A mode (LOW POWER)
const int JOY_DZ_LOW = 35;
const float EXPO_LOW = 1.0;
const int MAX_PWM_LOW = 127; // 50%

// Failsafe
const unsigned long FAILSAFE_MS = 500;

// Set true once to clear paired keys, then set back to false
const bool FORGET_PAIRED_KEYS_ON_BOOT = true;

// ─────────────────────────────────────────────────────────────────────────────

static unsigned long lastDataMs = 0;

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

// ── MAP SPEED TO SERVO PULSE ──────────────────────────────────────────────────
// speed: -255 (full reverse) … 0 (stopped) … +255 (full forward)
int speedToServo(int speed) {
  if (speed == 0)
    return SERVO_CENTER;
  if (speed > 0)
    return map(speed, 1, 255, SERVO_CENTER + 1, SERVO_MAX);
  else
    return map(-speed, 1, 255, SERVO_CENTER - 1, SERVO_MIN);
}

// ── MOTOR DRIVER (RC Servo Signal) ───────────────────────────────────────────
void writeMotors(int leftSpeed, int rightSpeed) {
  ledcWrite(CHANNEL_LEFT, speedToServo(leftSpeed));
  ledcWrite(CHANNEL_RIGHT, speedToServo(rightSpeed));
}

void Stop() {
  writeMotors(0, 0);
}

// Write motor targets instantly
void setTargets(int left, int right) {
  writeMotors(left, right);
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
      Stop();
      return;
    }
  }
  Serial.println("Controller disconnected but not found");
}

// ── GAMEPAD PROCESSING ───────────────────────────────────────────────────────
void processGamepad(ControllerPtr ctl) {
  int16_t axisX = ctl->axisX();
  int throttle = ctl->throttle();
  int brake = ctl->brake();
  bool bBtn = ctl->b();
  bool aBtn = ctl->a();
  bool yBtn = ctl->y();
  bool xBtn = ctl->x();

  lastDataMs = millis();

  // ── MODE SELECT (rising edge) ─────────────────────────────────────────
  static int mode = 2;
  static bool bWasDown = false;
  static bool aWasDown = false;
  static bool yWasDown = false;
  static bool xWasDown = false;
  if (bBtn && !bWasDown) {
    mode = 2;
    Serial.println("HIGH POWER 100% (B)");
  }
  if (yBtn && !yWasDown) {
    mode = 1;
    Serial.println("MID  POWER  60% (Y)");
  }
  if (aBtn && !aWasDown) {
    mode = 0;
    Serial.println("LOW  POWER  50% (A)");
  }
  bWasDown = bBtn;
  yWasDown = yBtn;
  aWasDown = aBtn;
  xWasDown = xBtn;

  // ── DIRECTION FLIP toggle (X button) ─────────────────────────────────
  static bool flipped = false;
  static bool xWasDown2 = false;
  if (xBtn && !xWasDown2) {
    flipped = !flipped;
    Serial.printf("DIRECTION: %s (X)\n", flipped ? "FLIPPED" : "NORMAL");
  }
  xWasDown2 = xBtn;

  // Pick parameters for current mode
  const int MAX_PWM = (mode == 2)   ? MAX_PWM_HIGH
                      : (mode == 1) ? MAX_PWM_MID
                                    : MAX_PWM_LOW;
  const float EXPO = (mode == 2)   ? EXPO_HIGH
                     : (mode == 1) ? EXPO_MID
                                   : EXPO_LOW;
  const int JOY_DZ = (mode == 2) ? JOY_DZ_HIGH : JOY_DZ_LOW;

  int baseSpeed = 0, steeringValue = 0;

  // ── TRIGGERS → FORWARD (R2) / REVERSE (L2) ───────────────────────────
  if (brake > TRIGGER_THRESHOLD) {
    int s = map(brake, TRIGGER_THRESHOLD, TRIGGER_MAX, 0, 255);
    s = constrain(s, 0, 255);
    s = applyExpo(s, 255, EXPO);
    baseSpeed = constrain(s, 0, MAX_PWM); // REVERSED: L2 now goes Forward

  } else if (throttle > TRIGGER_THRESHOLD) {
    int s = map(throttle, TRIGGER_THRESHOLD, TRIGGER_MAX, 0, 255);
    s = constrain(s, 0, 255);
    s = applyExpo(s, 255, EXPO);
    baseSpeed = -constrain(s, 0, MAX_PWM); // REVERSED: R2 now goes Reverse
  }

  // ── JOYSTICK X → LEFT / RIGHT ─────────────────────────────────────────
  if (abs(axisX) > JOY_DZ) {
    int mapped = map(abs(axisX), JOY_DZ, 512, 0, 255);
    mapped = constrain(mapped, 0, 255);
    mapped = applyExpo(mapped, 255, EXPO);
    mapped = constrain(mapped, 0, MAX_PWM);
    steeringValue = (axisX > 0) ? -mapped : mapped; // REVERSED: JoyX directions flipped
  }

  if (flipped)
    baseSpeed = -baseSpeed;

  // ── ARCADE DRIVE MIX ──────────────────────────────────────────────────
  int leftMotorSpeed = constrain(baseSpeed + steeringValue, -MAX_PWM, MAX_PWM);
  int rightMotorSpeed = constrain(baseSpeed - steeringValue, -MAX_PWM, MAX_PWM);

  if (baseSpeed != 0 || steeringValue != 0)
    setTargets(leftMotorSpeed, rightMotorSpeed);
  else
    setTargets(0, 0);

  // ── LIVE DEBUG PRINT (every 100ms) ───────────────────────────────────
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.printf("[RAW] axisX=%5d  throttle=%4d  brake=%4d  "
                  "btnA=%d btnB=%d btnX=%d btnY=%d\n"
                  "[OUT] mode=%-4s  baseSpeed=%4d  steerVal=%4d  "
                  "L=%4d  R=%4d  servoL=%5d  servoR=%5d\n\n",
                  (int)ctl->axisX(), (int)ctl->throttle(), (int)ctl->brake(),
                  ctl->a(), ctl->b(), ctl->x(), ctl->y(),
                  (mode == 2)   ? "HIGH"
                  : (mode == 1) ? "MID"
                                : "LOW",
                  baseSpeed, steeringValue,
                  leftMotorSpeed, rightMotorSpeed, speedToServo(leftMotorSpeed),
                  speedToServo(rightMotorSpeed));
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
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  // Setup servo PWM channels — outputs 1.5ms center pulse immediately
  ledcSetup(CHANNEL_LEFT, SERVO_FREQ, SERVO_RESOLUTION);
  ledcSetup(CHANNEL_RIGHT, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(RC_LEFT, CHANNEL_LEFT);
  ledcAttachPin(RC_RIGHT, CHANNEL_RIGHT);

  Stop(); // output 1.5ms center pulse → 30D sees valid RC at stop

  BP32.setup(&onConnectedController, &onDisconnectedController);

  if (FORGET_PAIRED_KEYS_ON_BOOT) {
    Serial.println("Clearing saved controller keys...");
    BP32.forgetBluetoothKeys();
    Serial.println("Done. Pair your controller again.");
  }

  Serial.println("\n=== RC Car Ready (Smartelex 30D — RC SERVO MODE) ===");
  Serial.println(
      "  DIP: Sw1=OFF  Sw2=OFF  Sw3=OFF  Sw4=ON  (RC Linear Independent)");
  Serial.println("  Wire: GPIO32 → RC1 Signal   GPIO33 → RC2 Signal");
  Serial.println("        GND → RC GND           RC1 5V (red) → ESP32 VIN");
  Serial.println(
      "  Both pins: RIGHT side of 38-pin NodeMCU (same side as GPIO25/26)");
  Serial.println("  B = HIGH POWER | Y = MID POWER | A = LOW POWER");
  Serial.println("  R2 = Forward | L2 = Reverse | Joystick X = Turn");
  Serial.println("  X = Flip direction");
  Serial.printf("  Failsafe stop after %lums\n", FAILSAFE_MS);
  Serial.println("\nPairing mode: press PS/Home on controller...");
}

// ── LOOP ─────────────────────────────────────────────────────────────────────
void loop() {
  if (lastDataMs != 0 && (millis() - lastDataMs > FAILSAFE_MS)) {
    Stop();
  }

  if (BP32.update()) {
    processControllers();
  }
}
