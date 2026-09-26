#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// --- Pin Definitions ---
// Motor A: Steering Motor (Left/Right) - Connected to OUT1 & OUT2
const int motorSteerIn1 = 26; 
const int motorSteerIn2 = 27; 

// Motor B: Drive Motor (Front/Back) - Connected to OUT3 & OUT4
const int motorDriveIn3 = 14; 
const int motorDriveIn4 = 32; // Changed from 12 to 32 to fix ESP32 boot crash

// Set true once to clear stored paired-controller keys, then set back to false
const bool FORGET_PAIRED_KEYS_ON_BOOT = true;

// --- PWM Compatibility Macros ---
// Ensures PWM works on both older (2.x) and newer (3.x) ESP32 Arduino Cores
#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define SETUP_PWM() 
  #define DRIVE_PWM_F(val) analogWrite(motorDriveIn3, val)
  #define DRIVE_PWM_B(val) analogWrite(motorDriveIn4, val)
#else
  #define SETUP_PWM() do { \
    ledcSetup(0, 5000, 8); \
    ledcSetup(1, 5000, 8); \
    ledcAttachPin(motorDriveIn3, 0); \
    ledcAttachPin(motorDriveIn4, 1); \
  } while(0)
  #define DRIVE_PWM_F(val) ledcWrite(0, val)
  #define DRIVE_PWM_B(val) ledcWrite(1, val)
#endif


// --- Motor Control Functions ---

void goForward(int speed) {
  DRIVE_PWM_F(speed);
  DRIVE_PWM_B(0);
}

void goBackward(int speed) {
  DRIVE_PWM_F(0);
  DRIVE_PWM_B(speed);
}

void stopDrive() {
  DRIVE_PWM_F(0);
  DRIVE_PWM_B(0);
}

void steerLeft() {
  digitalWrite(motorSteerIn1, HIGH);
  digitalWrite(motorSteerIn2, LOW);
}

void steerRight() {
  digitalWrite(motorSteerIn1, LOW);
  digitalWrite(motorSteerIn2, HIGH);
}

void centerSteering() {
  digitalWrite(motorSteerIn1, LOW);
  digitalWrite(motorSteerIn2, LOW);
}

void stopCar() {
  stopDrive();
  centerSteering();
}


// --- BLUETOOTH CALLBACKS ---

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller connected at index=%d\n", i);
            myControllers[i] = ctl;
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            stopCar();
            return;
        }
    }
}


// --- GAMEPAD PROCESSING ---

void processGamepad(ControllerPtr ctl) {
    int16_t axisX    = ctl->axisX();       // -512 to 511
    int16_t axisY    = ctl->axisY();       // -512 to 511
    int     throttle = ctl->throttle();    // 0 to 1020
    int     brake    = ctl->brake();       // 0 to 1020

    // 1. Steering Control (Left Joystick X)
    // Cheap RC cars usually just have an ON/OFF steering motor 
    const int STEER_DZ = 150;
    if (axisX < -STEER_DZ) {
        steerLeft();
    } else if (axisX > STEER_DZ) {
        steerRight();
    } else {
        centerSteering();
    }

    // 2. Drive Control (Triggers ONLY)
    const int TRIGGER_THRESHOLD = 50;
    
    if (throttle > TRIGGER_THRESHOLD) {
        int speed = map(throttle, 0, 1020, 0, 255);
        goForward(constrain(speed, 0, 255));
    } 
    else if (brake > TRIGGER_THRESHOLD) {
        int speed = map(brake, 0, 1020, 0, 255);
        goBackward(constrain(speed, 0, 255));
    } 
    else {
        stopDrive();
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->isGamepad()) {
            processGamepad(myController);
        }
    }
}


// --- SETUP ---

void setup() {
  Serial.begin(115200);
  
  pinMode(motorSteerIn1, OUTPUT);
  pinMode(motorSteerIn2, OUTPUT);
  pinMode(motorDriveIn3, OUTPUT);
  pinMode(motorDriveIn4, OUTPUT);

  SETUP_PWM();
  stopCar();

  // Initialize Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  
  if (FORGET_PAIRED_KEYS_ON_BOOT) {
      Serial.println("Clearing saved controller keys...");
      BP32.forgetBluetoothKeys();
      Serial.println("Done. Set FORGET_PAIRED_KEYS_ON_BOOT to false after your first successful pair.");
  }

  Serial.println("\n=== Bluepad32 RC Car Ready ===");
  Serial.println("Pairing mode: press PS/Home on your gamepad controller...");
}

// --- LOOP ---

void loop() {
  // Older versions of Bluepad32 have BP32.update() return void
  BP32.update();
  processControllers();
  delay(15); // Small delay to prevent loop from running too fast
}
