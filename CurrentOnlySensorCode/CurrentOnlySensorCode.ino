/*
  MazeSolver_Blind_Fixed.ino
  ---------------------------------------------------------
  Fixes applied vs the previous version:
    1. Wire.begin() was missing entirely -> I2C never actually
       started, so every sensor.begin() call spun for the full
       3-second timeout on every boot (up to ~9s wasted).
    2. Startup is now staggered: Serial -> short settle delay ->
       UART/motor stop -> I2C bus recovery + sensors (one at a
       time, with pauses) -> Bluetooth LAST. This spreads out
       the current spikes at power-on instead of firing them
       all in the same instant, which is the classic cause of
       an ESP32 brownout-reset loop on a marginal supply.
    3. Added an immediate boot print + reset-reason print, so
       you can watch USB serial and see directly whether the
       chip is brownout-resetting in a loop before it manages
       to finish booting.

  REMINDER (hardware, not code):
    - The 5V logic rail (ESP32+ATmega32+sensors) and the 7.4V
      motor rail MUST share a common ground, or you'll get
      exactly this kind of flaky, hard-to-reproduce behavior.
    - A bulk capacitor (470-1000uF) across 5V/GND right at the
      ESP32's VIN pin will smooth out the startup current spike
      and is a very common fix if brownout resets persist even
      with this staggered boot.
*/

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "BluetoothSerial.h"
#include "esp_system.h"

BluetoothSerial SerialBT;

// Sensor objects
Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X();

// XSHUT Pins
#define FRONT_XSHUT_PIN 19
#define LEFT_XSHUT_PIN 18
#define RIGHT_XSHUT_PIN 4

bool front_ok = false;
bool left_ok  = false;
bool right_ok = false;

// Motor speeds
int BASE_LEFT = 180;
int BASE_RIGHT = 175; // slightly lower to compensate for drift
int TURN_POWER = 170;

// Only true once SerialBT.begin() has actually run - avoids touching
// SerialBT before it's initialized.
bool bt_ready = false;

// ---------------- LOGGING ----------------
void LOG(const String &msg) {
    Serial.println(msg);
    if (bt_ready && SerialBT.hasClient()) {
        SerialBT.println(msg);
    }
}

// ---------------- MOTOR COMMANDS ----------------
void sendMotorCmd(char cmd, int left, int right) {
    Serial2.print(cmd);
    Serial2.write((uint8_t)left);
    Serial2.write((uint8_t)right);
}

// ---------------- BOOT DIAGNOSTICS ----------------
// If "BOOT" + this reason keep repeating on the USB serial monitor
// without ever reaching "GO!", the chip is brownout-resetting in a
// loop - that's a power problem, not a code problem.
void printResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.print("Reset reason: ");
    switch (reason) {
        case ESP_RST_POWERON:   Serial.println("Power-on reset"); break;
        case ESP_RST_BROWNOUT:  Serial.println("BROWNOUT (power sag at boot!)"); break;
        case ESP_RST_SW:        Serial.println("Software reset"); break;
        case ESP_RST_PANIC:     Serial.println("Software panic/exception"); break;
        case ESP_RST_INT_WDT:   Serial.println("Interrupt watchdog"); break;
        case ESP_RST_TASK_WDT:  Serial.println("Task watchdog"); break;
        case ESP_RST_WDT:       Serial.println("Other watchdog"); break;
        case ESP_RST_DEEPSLEEP: Serial.println("Deep sleep wake"); break;
        case ESP_RST_EXT:       Serial.println("External pin reset"); break;
        default:                Serial.println("Unknown (" + String((int)reason) + ")"); break;
    }
}

// ---------------- I2C BUS RECOVERY ----------------
void i2cBusRecover() {
    LOG("I2C: Running bus recovery...");
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    digitalWrite(21, HIGH); // SDA high
    for (int i = 0; i < 9; i++) {
        digitalWrite(22, LOW);  delayMicroseconds(10);
        digitalWrite(22, HIGH); delayMicroseconds(10);
    }
    // STOP condition: SDA rises while SCL is high
    digitalWrite(21, LOW);  delayMicroseconds(10);
    digitalWrite(21, HIGH); delayMicroseconds(10);

    Wire.begin();          // <-- THE MISSING LINE. I2C was never started before.
    Wire.setClock(100000); // Standard 100kHz - more tolerant of long/noisy wiring than 400kHz
    delay(50);
    LOG("I2C: Bus recovery complete, Wire started.");
}

// ---------------- SENSOR INIT ----------------
bool initSensor(Adafruit_VL53L0X &sensor, uint8_t addr, const char* name) {
    unsigned long start = millis();
    while (millis() - start < 3000) {
        if (sensor.begin(addr)) {
            LOG("  [OK] Sensor " + String(name) + " ready at 0x" + String(addr, HEX));
            return true;
        }
        delay(200);
    }
    LOG("  [FAIL] Sensor " + String(name) + " NOT FOUND! Skipping...");
    return false;
}

void setup() {
    // --- STAGE 0: USB serial up first, before anything else draws current ---
    Serial.begin(115200);
    delay(50);
    Serial.println("\n>>> BOOT <<<"); // If this repeats in a loop, it's a brownout reset loop
    printResetReason();

    // --- STAGE 1: Let the power rail settle before asking for any current ---
    // Right at power-on, regulators/caps are still charging. Give it a moment
    // instead of firing up Serial2 + sensors + Bluetooth all in the same instant.
    delay(300);

    // --- STAGE 2: UART to ATmega32 + force motors stopped ---
    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    delay(50);
    sendMotorCmd('S', 0, 0); // Force stop in case the ATmega was mid-command on a reset

    // --- STAGE 3: I2C bus + sensors (relatively low current) ---
    // Do this BEFORE Bluetooth, since sensor init draws far less current
    // than bringing up the BT radio does.
    pinMode(FRONT_XSHUT_PIN, OUTPUT);
    pinMode(LEFT_XSHUT_PIN,  OUTPUT);
    pinMode(RIGHT_XSHUT_PIN, OUTPUT);
    digitalWrite(FRONT_XSHUT_PIN, LOW);
    digitalWrite(LEFT_XSHUT_PIN,  LOW);
    digitalWrite(RIGHT_XSHUT_PIN, LOW);
    delay(50); // hold all three sensors in reset briefly

    i2cBusRecover(); // also calls Wire.begin() now

    // Bring sensors up ONE AT A TIME with a pause between each, instead of
    // back-to-back, so their I2C init current spikes don't stack on top of
    // each other.
    digitalWrite(FRONT_XSHUT_PIN, HIGH);
    delay(100);
    front_ok = initSensor(sensorFront, 0x30, "FRONT");
    delay(100);

    digitalWrite(LEFT_XSHUT_PIN, HIGH);
    delay(100);
    left_ok = initSensor(sensorLeft, 0x31, "LEFT");
    delay(100);

    digitalWrite(RIGHT_XSHUT_PIN, HIGH);
    delay(100);
    right_ok = initSensor(sensorRight, 0x32, "RIGHT");
    delay(100);

    Serial.println("Sensors done. Bringing up Bluetooth...");

    // --- STAGE 4: Bluetooth LAST ---
    // Bringing up the classic BT stack is the single biggest current spike
    // at boot (bursts of 200-300mA aren't unusual). Doing it last, after
    // everything else is already stable, means it isn't competing with the
    // sensor I2C init for the same current budget.
    delay(200);
    SerialBT.begin("MazeSolver_Blind");
    bt_ready = true;
    delay(200);

    LOG("\nBooting Sensor-Only Maze Solver...");
    LOG("Sensors initialized. Starting in 3 seconds...");
    delay(3000);
    LOG(">>> GO! <<<");
}

// ── FIXED-TIME TURNS (CALIBRATE THESE!) ──
void turn90(char dir) {
    LOG("Turning " + String(dir) + " for 90 degrees...");
    sendMotorCmd(dir, TURN_POWER, TURN_POWER);
    // TUNE THIS: adjust milliseconds until the car turns exactly 90 degrees
    delay(600);
    sendMotorCmd('S', 0, 0);
    delay(200); // pause to stabilize
}

void turn180() {
    LOG("U-Turn 180 degrees...");
    sendMotorCmd('L', TURN_POWER, TURN_POWER); // Spin left for a U-Turn
    // TUNE THIS: adjust milliseconds until the car turns exactly 180 degrees
    delay(1200);
    sendMotorCmd('S', 0, 0);
    delay(200);
}

void loop() {
    VL53L0X_RangingMeasurementData_t measureFront, measureLeft, measureRight;

    // 1. READ SENSORS (only if they initialized successfully)
    if (front_ok) sensorFront.rangingTest(&measureFront, false);
    if (left_ok)  sensorLeft.rangingTest(&measureLeft, false);
    if (right_ok) sensorRight.rangingTest(&measureRight, false);

    int front_dist = (!front_ok || measureFront.RangeStatus != 0) ? 8190 : measureFront.RangeMilliMeter;
    int left_dist  = (!left_ok  || measureLeft.RangeStatus  != 0) ? 8190 : measureLeft.RangeMilliMeter;
    int right_dist = (!right_ok || measureRight.RangeStatus != 0) ? 8190 : measureRight.RangeMilliMeter;

    // Optional: Log distance. Comment out if it spams too much.
    // LOG("F:" + String(front_dist) + " L:" + String(left_dist) + " R:" + String(right_dist));

    // 2. MAKE DECISION
    int THRESHOLD = 200; // Stop if a wall is closer than 200mm

    if (front_dist > THRESHOLD) {
        // Path is clear ahead, go straight!
        sendMotorCmd('F', BASE_LEFT, BASE_RIGHT);
        delay(100);
    } else {
        // Wall ahead! Stop immediately.
        sendMotorCmd('S', 0, 0);
        LOG("Wall Ahead! Deciding where to turn...");

        delay(500);

        // Right-Hand Preference Logic
        if (right_dist > THRESHOLD) {
            turn90('R');
        } else if (left_dist > THRESHOLD) {
            turn90('L');
        } else {
            turn180();
        }
    }
}
