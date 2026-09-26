// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  ESP32 #1 — CONTROLLER (TX)                                             ║
// ║  • Connects to PS5/gamepad via Bluepad32 (Bluetooth) on Core 1          ║
// ║  • Sends motor commands via ESP-NOW on Core 0 (WiFi core) at 200 Hz     ║
// ║  • Dual-core FreeRTOS split = minimum latency                           ║
// ╚══════════════════════════════════════════════════════════════════════════╝

#include <Bluepad32.h>
#include <esp_now.h>
#include <esp_wifi.h>   // needed for esp_wifi_set_ps()
#include <WiFi.h>

// ── TARGET MAC ADDRESS ────────────────────────────────────────────────────────
uint8_t robotMAC[] = {0x78, 0x21, 0x84, 0x8C, 0x8F, 0xF8};  // Robot ESP32 MAC: 78:21:84:8C:8F:F8

// ── ESP-NOW PACKET ────────────────────────────────────────────────────────────
typedef struct {
    int16_t  leftSpeed;   // –255 … 255
    int16_t  rightSpeed;  // –255 … 255
    uint8_t  stop;        // 1 = emergency / failsafe stop
} MotorPacket;

// ── SHARED CACHED VALUES (volatile = safe to share between Core 0 & Core 1) ──
volatile int16_t cachedLeft  = 0;
volatile int16_t cachedRight = 0;
volatile uint8_t cachedStop  = 1;  // start in STOP until controller connects

// ── BLUEPAD32 ─────────────────────────────────────────────────────────────────
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ── TUNING ────────────────────────────────────────────────────────────────────
const int   TRIGGER_THRESHOLD = 150;
const int   TRIGGER_MAX       = 1020;

// B mode (HIGH POWER) settings
const int   JOY_DZ_HIGH  = 50;
const float EXPO_HIGH     = 2.8;
const int   MAX_PWM_HIGH  = 255;

// A mode (LOW POWER) settings
const int   JOY_DZ_LOW  = 50;
const float EXPO_LOW     = 1.8;
const int   MAX_PWM_LOW  = 127;   // 50% cap

// Failsafe: send STOP if no Bluetooth data arrives for this long
const unsigned long FAILSAFE_MS = 500;

// Set true once to clear stored paired-controller keys, then set back to false
const bool FORGET_PAIRED_KEYS_ON_BOOT = true;

// ─────────────────────────────────────────────────────────────────────────────
static unsigned long lastDataMs = 0;

// ── EXPO CURVE ────────────────────────────────────────────────────────────────
int applyExpo(int value, int maxVal, float curve) {
    if (value == 0) return 0;
    float norm   = (float)abs(value) / (float)maxVal;
    norm = constrain(norm, 0.0f, 1.0f);
    float curved = pow(norm, curve);
    int   result = (int)(curved * maxVal);
    return (value > 0) ? result : -result;
}

// ── ESP-NOW SEND CALLBACK ─────────────────────────────────────────────────────
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    // Uncomment for debug:
    // Serial.printf("ESP-NOW: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ── BLUETOOTH CALLBACKS ───────────────────────────────────────────────────────
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller connected at index=%d\n", i);
            ControllerProperties p = ctl->getProperties();
            Serial.printf("Model: %s, VID=0x%04x, PID=0x%04x\n",
                          ctl->getModelName().c_str(), p.vendor_id, p.product_id);
            myControllers[i] = ctl;
            cachedStop = 0;   // clear stop flag — controller is live
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
            cachedLeft  = 0;
            cachedRight = 0;
            cachedStop  = 1;   // espNowTask will keep sending STOP
            return;
        }
    }
    Serial.println("Controller disconnected but not found");
}

// ── GAMEPAD PROCESSING ────────────────────────────────────────────────────────
void processGamepad(ControllerPtr ctl) {
    int16_t axisX    = ctl->axisX();
    int     throttle = ctl->throttle();
    int     brake    = ctl->brake();
    bool    bBtn     = ctl->b();
    bool    aBtn     = ctl->a();
    bool    yBtn     = ctl->y();
    bool    xBtn     = ctl->x();

    // Update failsafe watchdog
    lastDataMs = millis();

    // ── MODE SELECT (rising edge) ─────────────────────────────────────────
    // 2 = HIGH (B, 100%) | 1 = MID (Y, 60%) | 0 = LOW (A, 50%)
    static int  mode     = 2;
    static bool bWasDown = false;
    static bool aWasDown = false;
    static bool yWasDown = false;
    static bool xWasDown = false;
    if (bBtn && !bWasDown) { mode = 2; Serial.println("HIGH POWER 100% (B)"); }
    if (yBtn && !yWasDown) { mode = 1; Serial.println("MID  POWER  60% (Y)"); }
    if (aBtn && !aWasDown) { mode = 0; Serial.println("LOW  POWER  50% (A)"); }
    bWasDown = bBtn;
    yWasDown = yBtn;
    aWasDown = aBtn;
    xWasDown = xBtn;

    // ── DIRECTION FLIP toggle (X button) ─────────────────────────────────
    static bool flipped   = false;
    static bool xWasDown2 = false;
    if (xBtn && !xWasDown2) {
        flipped = !flipped;
        Serial.printf("DIRECTION: %s (X)\n", flipped ? "FLIPPED" : "NORMAL");
    }
    xWasDown2 = xBtn;

    // Pick parameters for current mode
    const int   MAX_PWM = (mode == 2) ? MAX_PWM_HIGH : (mode == 1) ? 153 : MAX_PWM_LOW;
    const float EXPO    = (mode == 2) ? EXPO_HIGH    : EXPO_LOW;
    const int   JOY_DZ  = (mode == 2) ? JOY_DZ_HIGH  : JOY_DZ_LOW;

    int baseSpeed = 0, steeringValue = 0;

    // ── TRIGGERS → FORWARD (throttle=R) / REVERSE (brake=L) ─────────────
    if (brake > TRIGGER_THRESHOLD) {
        int s = map(brake, TRIGGER_THRESHOLD, TRIGGER_MAX, 0, 255);
        s = constrain(s, 0, 255);
        s = applyExpo(s, 255, EXPO);
        steeringValue = -constrain(s, 0, MAX_PWM);   // brake = REVERSE
    } else if (throttle > TRIGGER_THRESHOLD) {
        int s = map(throttle, TRIGGER_THRESHOLD, TRIGGER_MAX, 0, 255);
        s = constrain(s, 0, 255);
        s = applyExpo(s, 255, EXPO);
        steeringValue = constrain(s, 0, MAX_PWM);     // throttle = FORWARD
    }

    // ── JOYSTICK X → LEFT / RIGHT ─────────────────────────────────────────
    if (abs(axisX) > JOY_DZ) {
        int mapped = map(abs(axisX), JOY_DZ, 512, 0, 255);
        mapped = constrain(mapped, 0, 255);
        mapped = applyExpo(mapped, 255, EXPO);
        mapped = constrain(mapped, 0, MAX_PWM);
        baseSpeed = (axisX > 0) ? mapped : -mapped;
    }

    // Apply direction flip
    if (flipped) baseSpeed = -baseSpeed;

    // ── ARCADE DRIVE MIX ──────────────────────────────────────────────────
    int leftMotorSpeed  = constrain(baseSpeed + steeringValue, -MAX_PWM, MAX_PWM);
    int rightMotorSpeed = constrain(baseSpeed - steeringValue, -MAX_PWM, MAX_PWM);

    // ── UPDATE SHARED CACHE (espNowTask on Core 0 will send these) ────────
    cachedLeft  = (int16_t)leftMotorSpeed;
    cachedRight = (int16_t)rightMotorSpeed;
    cachedStop  = 0;

    // ── LIVE DEBUG PRINT (every 200 ms) ───────────────────────────────────
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        Serial.printf(
            "[RAW] axisX=%5d  axisY=%5d  axisRX=%5d  axisRY=%5d  "
            "throttle=%4d  brake=%4d  "
            "btnA=%d btnB=%d btnX=%d btnY=%d  "
            "L1=%d R1=%d L2=%d R2=%d\n"
            "[OUT] mode=%-5s  baseSpeed=%4d  steeringVal=%4d  "
            "L_PWM=%4d  R_PWM=%4d\n\n",
            (int)ctl->axisX(),  (int)ctl->axisY(),
            (int)ctl->axisRX(), (int)ctl->axisRY(),
            (int)ctl->throttle(), (int)ctl->brake(),
            ctl->a(), ctl->b(), ctl->x(), ctl->y(),
            ctl->l1(), ctl->r1(), ctl->l2(), ctl->r2(),
            (mode == 2) ? "HIGH" : (mode == 1) ? "MID" : "LOW",
            baseSpeed, steeringValue,
            leftMotorSpeed, rightMotorSpeed
        );
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  ESP-NOW TASK — runs on Core 0 (the WiFi/protocol core)                 ║
// ║  Sends cached motor values at 200 Hz (every 5 ms).                      ║
// ║  Pinning to Core 0 means ESP-NOW packets are handed directly to the     ║
// ║  WiFi driver without any cross-core scheduling delay.                   ║
// ╚══════════════════════════════════════════════════════════════════════════╝
void espNowTask(void* param) {
    MotorPacket pkt;
    TickType_t  xLastWake = xTaskGetTickCount();
    const TickType_t xInterval = pdMS_TO_TICKS(5);  // 5 ms = 200 Hz

    for (;;) {
        // Failsafe: if Core 1 (BT) has gone silent, force STOP
        if (lastDataMs != 0 && (millis() - lastDataMs > FAILSAFE_MS)) {
            cachedLeft  = 0;
            cachedRight = 0;
            cachedStop  = 1;
        }

        pkt.leftSpeed  = cachedLeft;
        pkt.rightSpeed = cachedRight;
        pkt.stop       = cachedStop;
        esp_now_send(robotMAC, (uint8_t*)&pkt, sizeof(pkt));

        // Precise 5 ms sleep — does NOT drift like millis()-based timers
        vTaskDelayUntil(&xLastWake, xInterval);
    }
}

// ── SETUP ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // ── WiFi / ESP-NOW INIT ───────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // MIN modem sleep = required for BT+WiFi coexistence, lowest-latency option

    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: esp_now_init() failed!");
        while (true) delay(1000);
    }
    esp_now_register_send_cb(onDataSent);

    // Register peer (Robot ESP32)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, robotMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("ERROR: esp_now_add_peer() failed! Check robotMAC.");
    } else {
        Serial.println("ESP-NOW peer added.");
    }

    // ── BLUEPAD32 INIT ────────────────────────────────────────────────────
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);

    if (FORGET_PAIRED_KEYS_ON_BOOT) {
        Serial.println("Clearing saved controller keys...");
        BP32.forgetBluetoothKeys();
        Serial.println("Done. Pair your controller again.");
    }

    // ── LAUNCH ESP-NOW TASK ON CORE 0 ─────────────────────────────────────
    // Core 0 = WiFi/BT protocol core. Pinning here lets ESP-NOW talk to the
    // WiFi driver with zero scheduling overhead.
    xTaskCreatePinnedToCore(
        espNowTask,    // task function
        "espnow_tx",  // name (for debugging)
        2048,         // stack size (bytes)
        NULL,         // parameter
        2,            // priority (higher than loop's priority of 1)
        NULL,         // task handle (not needed)
        0             // ← CORE 0 (WiFi core)
    );

    Serial.println("\n=== Controller ESP32 Ready (dual-core) ===");
    Serial.println("  Core 1: Bluepad32 (BT)  |  Core 0: ESP-NOW @ 200 Hz");
    Serial.println("  B = HIGH POWER | Y = MID POWER | A = LOW POWER");
    Serial.println("  Left trigger = Forward | Right trigger = Reverse");
    Serial.println("  Joystick X   = Left / Right | X = Flip direction");
    Serial.println("\nPairing mode: press PS/Home on controller...");
}

// ── LOOP (runs on Core 1 — app core) ─────────────────────────────────────────
// Just polls Bluepad32 as fast as possible. ESP-NOW sending is handled by
// espNowTask on Core 0, so there is zero interference between the two.
void loop() {
    if (BP32.update()) {
        processControllers();
    }
    // No delay — let Bluepad32 run at full BT stack speed
}
