#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// BTS7960 Motor pins - Left Motor
const int ML_RPWM = 19;  // Left motor forward PWM
const int ML_LPWM = 21;  // Left motor reverse PWM

// BTS7960 Motor pins - Right Motor
const int MR_RPWM = 22;  // Right motor forward PWM
const int MR_LPWM = 23;  // Right motor reverse PWM

// PWM properties
const int freq = 30000;
const int resolution = 8;
const int pwmChannelML_RPWM = 0;
const int pwmChannelML_LPWM = 1;
const int pwmChannelMR_RPWM = 2;
const int pwmChannelMR_LPWM = 3;

// SENSITIVITY SETTINGS - Adjust these to change responsiveness
const int TRIGGER_THRESHOLD = 150;      // Dead zone (higher = less twitchy)
const int JOYSTICK_THRESHOLD = 200;     // Dead zone (higher = less twitchy)
const float TRIGGER_SENSITIVITY = 1.0;  // 1.0 = full speed available
const float STEERING_SENSITIVITY = 1.0; // 1.0 = full steering available
const float EXPO_CURVE = 1.8;           // Smoothness curve (higher = smoother at low inputs)

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller connected at index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Model: %s, VID=0x%04x, PID=0x%04x\n", 
                         ctl->getModelName().c_str(), 
                         properties.vendor_id,
                         properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("Controller connected but no empty slot found");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            Stop();
            break;
        }
    }
    if (!foundController) {
        Serial.println("Controller disconnected but not found");
    }
}

void Stop() {
    // Stop all motors by setting all PWM outputs to 0
    ledcWrite(pwmChannelML_RPWM, 0);
    ledcWrite(pwmChannelML_LPWM, 0);
    ledcWrite(pwmChannelMR_RPWM, 0);
    ledcWrite(pwmChannelMR_LPWM, 0);
}

// Apply exponential curve to reduce sensitivity at low values
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

void driveMotors(int leftSpeed, int rightSpeed) {
    // Left Motor Control (BTS7960)
    // RPWM = forward, LPWM = reverse
    if (leftSpeed > 0) {
        ledcWrite(pwmChannelML_LPWM, 0);           // No reverse
        ledcWrite(pwmChannelML_RPWM, leftSpeed);    // Forward speed
    } else if (leftSpeed < 0) {
        ledcWrite(pwmChannelML_RPWM, 0);           // No forward
        ledcWrite(pwmChannelML_LPWM, abs(leftSpeed)); // Reverse speed
    } else {
        ledcWrite(pwmChannelML_RPWM, 0);
        ledcWrite(pwmChannelML_LPWM, 0);
    }
    
    // Right Motor Control (BTS7960)
    // RPWM = forward, LPWM = reverse
    if (rightSpeed > 0) {
        ledcWrite(pwmChannelMR_LPWM, 0);           // No reverse
        ledcWrite(pwmChannelMR_RPWM, rightSpeed);   // Forward speed
    } else if (rightSpeed < 0) {
        ledcWrite(pwmChannelMR_RPWM, 0);           // No forward
        ledcWrite(pwmChannelMR_LPWM, abs(rightSpeed)); // Reverse speed
    } else {
        ledcWrite(pwmChannelMR_RPWM, 0);
        ledcWrite(pwmChannelMR_LPWM, 0);
    }
}

void processGamepad(ControllerPtr ctl) {
    // Get trigger values (0-1023)
    int throttleValue = ctl->throttle();
    int brakeValue = ctl->brake();
    
    // Get left joystick X-axis for steering (-511 to 512)
    int16_t joyX = ctl->axisX();
    
    // Calculate base speed with dead zone
    int baseSpeed = 0;
    
    if (throttleValue > TRIGGER_THRESHOLD) {
        // Map from threshold to full range
        int mappedSpeed = map(throttleValue, TRIGGER_THRESHOLD, 1023, 0, 255);
        
        // Apply exponential curve for smooth control at low inputs
        mappedSpeed = applyExponentialCurve(mappedSpeed, 255);
        
        // Apply sensitivity multiplier
        baseSpeed = (int)(mappedSpeed * TRIGGER_SENSITIVITY);
        baseSpeed = constrain(baseSpeed, 100, 255);  // Min speed for torque
        
    } else if (brakeValue > TRIGGER_THRESHOLD) {
        // Same mapping for reverse
        int mappedSpeed = map(brakeValue, TRIGGER_THRESHOLD, 1023, 0, 255);
        
        // Apply exponential curve
        mappedSpeed = applyExponentialCurve(mappedSpeed, 255);
        
        // Apply sensitivity multiplier (negative for reverse)
        baseSpeed = -(int)(mappedSpeed * TRIGGER_SENSITIVITY);
        baseSpeed = constrain(baseSpeed, -255, -100);
    }
    
    // Calculate steering with dead zone and exponential curve
    int steeringValue = 0;
    if (abs(joyX) > JOYSTICK_THRESHOLD) {
        // Map joystick with dead zone removed
        int mappedSteering = map(abs(joyX), JOYSTICK_THRESHOLD, 512, 0, 255);
        
        // Apply exponential curve for smooth steering
        mappedSteering = applyExponentialCurve(mappedSteering, 255);
        
        // Apply sensitivity multiplier
        mappedSteering = (int)(mappedSteering * STEERING_SENSITIVITY);
        
        // Restore sign
        steeringValue = (joyX > 0) ? mappedSteering : -mappedSteering;
        steeringValue = constrain(steeringValue, -255, 255);
    }
    
    // ARCADE DRIVE MIXING
    int leftMotorSpeed = baseSpeed + steeringValue;
    int rightMotorSpeed = baseSpeed - steeringValue;
    
    // Constrain to valid PWM range
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
    
    // Apply to motors
    if (baseSpeed != 0 || steeringValue != 0) {
        driveMotors(leftMotorSpeed, rightMotorSpeed);
    } else {
        Stop();
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

void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", 
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    
    // Configure PWM channels for BTS7960
    ledcSetup(pwmChannelML_RPWM, freq, resolution);
    ledcSetup(pwmChannelML_LPWM, freq, resolution);
    ledcSetup(pwmChannelMR_RPWM, freq, resolution);
    ledcSetup(pwmChannelMR_LPWM, freq, resolution);
    
    // Attach pins to PWM channels
    ledcAttachPin(ML_RPWM, pwmChannelML_RPWM);
    ledcAttachPin(ML_LPWM, pwmChannelML_LPWM);
    ledcAttachPin(MR_RPWM, pwmChannelMR_RPWM);
    ledcAttachPin(MR_LPWM, pwmChannelMR_LPWM);
    
    Stop();
    
    // Setup Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    
    Serial.println("\n=== RC Car Ready - BTS7960 DRIVER - SMOOTH CONTROL MODE ===");
    Serial.println("Hardware Configuration:");
    Serial.println("  Left Motor:  D19(RPWM), D21(LPWM)");
    Serial.println("  Right Motor: D22(RPWM), D23(LPWM)");
    Serial.println("\nFeatures:");
    Serial.println("  - BTS7960 High-Power Driver (43A capable)");
    Serial.println("  - Dead zones prevent accidental movement");
    Serial.println("  - Exponential curve for smooth low-speed control");
    Serial.println("  - Full 255 PWM speed at maximum trigger press");
    Serial.println("\nAdjust these values in code:");
    Serial.printf("  TRIGGER_THRESHOLD: %d (dead zone size)\n", TRIGGER_THRESHOLD);
    Serial.printf("  JOYSTICK_THRESHOLD: %d (steering dead zone)\n", JOYSTICK_THRESHOLD);
    Serial.printf("  EXPO_CURVE: %.1f (smoothness: 1.0=linear, 2.0+=smooth)\n", EXPO_CURVE);
    Serial.println("\nPut your controller in pairing mode now...");
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }
    delay(10);
}
