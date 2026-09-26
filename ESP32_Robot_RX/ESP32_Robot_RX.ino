// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  ESP32 #2 — ROBOT (RX)                                                  ║
// ║  • Receives motor commands from the Controller ESP32 via ESP-NOW        ║
// ║  • Drives BTS7960 H-bridge on the SAME PINS as your original code       ║
// ║  • Prints its own MAC address on boot (copy it into the TX sketch)      ║
// ╚══════════════════════════════════════════════════════════════════════════╝

#include <esp_now.h>
#include <WiFi.h>

// ── MOTOR PINS  (same as original Roborace.ino) ───────────────────────────────
const int ML_RPWM = 19;   // Left  motor forward PWM
const int ML_LPWM = 21;   // Left  motor reverse  PWM
const int MR_RPWM = 22;   // Right motor forward PWM
const int MR_LPWM = 23;   // Right motor reverse  PWM

// ── PWM CHANNELS ─────────────────────────────────────────────────────────────
const int freq              = 30000;
const int resolution        = 8;
const int pwmChannelML_RPWM = 0;
const int pwmChannelML_LPWM = 1;
const int pwmChannelMR_RPWM = 2;
const int pwmChannelMR_LPWM = 3;

// ── FAILSAFE ──────────────────────────────────────────────────────────────────
// If no ESP-NOW packet arrives within this window, stop the motors.
const unsigned long FAILSAFE_MS = 500;   // same as original Roborace.ino

static unsigned long lastPktMs = 0;

// ── ESP-NOW PACKET (must match Controller sketch exactly) ─────────────────────
typedef struct {
    int16_t  leftSpeed;
    int16_t  rightSpeed;
    uint8_t  stop;
} MotorPacket;

// ── MOTOR HELPERS ─────────────────────────────────────────────────────────────
void Stop() {
    ledcWrite(pwmChannelML_RPWM, 0);
    ledcWrite(pwmChannelML_LPWM, 0);
    ledcWrite(pwmChannelMR_RPWM, 0);
    ledcWrite(pwmChannelMR_LPWM, 0);
}

void driveMotors(int leftSpeed, int rightSpeed) {
    // Left motor
    if (leftSpeed > 0) {
        ledcWrite(pwmChannelML_LPWM, 0);
        ledcWrite(pwmChannelML_RPWM, leftSpeed);
    } else if (leftSpeed < 0) {
        ledcWrite(pwmChannelML_RPWM, 0);
        ledcWrite(pwmChannelML_LPWM, abs(leftSpeed));
    } else {
        ledcWrite(pwmChannelML_RPWM, 0);
        ledcWrite(pwmChannelML_LPWM, 0);
    }
    // Right motor
    if (rightSpeed > 0) {
        ledcWrite(pwmChannelMR_LPWM, 0);
        ledcWrite(pwmChannelMR_RPWM, rightSpeed);
    } else if (rightSpeed < 0) {
        ledcWrite(pwmChannelMR_RPWM, 0);
        ledcWrite(pwmChannelMR_LPWM, abs(rightSpeed));
    } else {
        ledcWrite(pwmChannelMR_RPWM, 0);
        ledcWrite(pwmChannelMR_LPWM, 0);
    }
}

// ── ESP-NOW RECEIVE CALLBACK ──────────────────────────────────────────────────
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(MotorPacket)) {
        Serial.printf("ESP-NOW: unexpected packet size %d\n", len);
        return;
    }

    MotorPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    lastPktMs = millis();   // reset failsafe watchdog

    if (pkt.stop) {
        Stop();
        Serial.println("[FAILSAFE] STOP received");
        return;
    }

    driveMotors((int)pkt.leftSpeed, (int)pkt.rightSpeed);

    // Debug print
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.printf("[RX] L=%4d  R=%4d\n", (int)pkt.leftSpeed, (int)pkt.rightSpeed);
    }
}

// ── SETUP ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // ── PWM SETUP ─────────────────────────────────────────────────────────
    ledcSetup(pwmChannelML_RPWM, freq, resolution);
    ledcSetup(pwmChannelML_LPWM, freq, resolution);
    ledcSetup(pwmChannelMR_RPWM, freq, resolution);
    ledcSetup(pwmChannelMR_LPWM, freq, resolution);
    ledcAttachPin(ML_RPWM, pwmChannelML_RPWM);
    ledcAttachPin(ML_LPWM, pwmChannelML_LPWM);
    ledcAttachPin(MR_RPWM, pwmChannelMR_RPWM);
    ledcAttachPin(MR_LPWM, pwmChannelMR_LPWM);
    Stop();

    // ── WiFi / ESP-NOW INIT ───────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Print this board's MAC — copy it into robotMAC[] in the TX sketch!
    Serial.println("\n=== Robot ESP32 (RX) ===");
    Serial.print("My MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("↑↑  Copy this into robotMAC[] in the Controller sketch!  ↑↑\n");

    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: esp_now_init() failed!");
        while (true) delay(1000);
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("Waiting for ESP-NOW commands from Controller ESP32...");
}

// ── LOOP ──────────────────────────────────────────────────────────────────────
void loop() {
    // Local failsafe: stop if no packet arrives within FAILSAFE_MS
    if (lastPktMs != 0 && (millis() - lastPktMs > FAILSAFE_MS)) {
        Stop();
        // Only print once per failsafe event
        static unsigned long lastWarn = 0;
        if (millis() - lastWarn > 1000) {
            lastWarn = millis();
            Serial.println("[FAILSAFE] No signal — motors STOPPED");
        }
    }
    // No delay — all motor work is done in the ESP-NOW receive callback
}
