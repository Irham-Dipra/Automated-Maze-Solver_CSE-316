#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define FRONT_XSHUT_PIN 19
#define LEFT_XSHUT_PIN 18
#define RIGHT_XSHUT_PIN 4

Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X();

// --- HARDWARE CALIBRATION VARIABLES ---
int BASE_LEFT = 128;  // Default requested by user
int BASE_RIGHT = 145; // Default requested by user
int DELAY_90_RIGHT = 390;
int DELAY_90_LEFT = 390;
int DELAY_180_U = 780;
int FRONT_BRAKE_DIST = 260;

// Function to print current state
void printState() {
    SerialBT.println("\nCurrent: BL=" + String(BASE_LEFT) + " | BR=" + String(BASE_RIGHT) + 
                     " | TR=" + String(DELAY_90_RIGHT) + " | TL=" + String(DELAY_90_LEFT) + 
                     " | TU=" + String(DELAY_180_U) + " | FD=" + String(FRONT_BRAKE_DIST));
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 16, 17); // UART to ATmega
    Serial2.print('S'); // Stop motors on boot
    SerialBT.begin("Maze_Calibrator");
    Wire.begin();
    
    // Boot Sensors
    pinMode(FRONT_XSHUT_PIN, OUTPUT); pinMode(LEFT_XSHUT_PIN, OUTPUT); pinMode(RIGHT_XSHUT_PIN, OUTPUT);
    digitalWrite(FRONT_XSHUT_PIN, LOW); digitalWrite(LEFT_XSHUT_PIN, LOW); digitalWrite(RIGHT_XSHUT_PIN, LOW);
    delay(10); 
    digitalWrite(FRONT_XSHUT_PIN, HIGH); delay(10); sensorFront.begin(0x30);
    digitalWrite(LEFT_XSHUT_PIN, HIGH); delay(10); sensorLeft.begin(0x31);
    digitalWrite(RIGHT_XSHUT_PIN, HIGH); delay(10); sensorRight.begin(0x32);
    
    SerialBT.println("Waiting for Bluetooth...");
    while (!SerialBT.hasClient()) delay(500);
    delay(1000);
    
    // Print instructions ONLY once!
    SerialBT.println("\n=== HARDWARE CALIBRATION MENU ===");
    SerialBT.println("Commands:");
    SerialBT.println("  BL:150  -> Base Left Speed");
    SerialBT.println("  BR:150  -> Base Right Speed");
    SerialBT.println("  TR:400  -> Turn Right Delay");
    SerialBT.println("  TL:400  -> Turn Left Delay");
    SerialBT.println("  TU:800  -> U-Turn Delay");
    SerialBT.println("  FD:260  -> Front Brake Distance");
    SerialBT.println("  START   -> Begin driving");
    SerialBT.println("  STOP    -> Halt the car");
    SerialBT.println("=================================");
    
    printState();
    
    bool is_running = false;
    
    while (!is_running) {
        if (SerialBT.available()) {
            String msg = SerialBT.readStringUntil('\n');
            msg.trim();
            if (msg.equalsIgnoreCase("START")) {
                SerialBT.println("\n>>> STARTING BLIND CALIBRATION DRIVE <<<");
                is_running = true;
            }
            else if (msg.startsWith("BL:")) { BASE_LEFT = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("BR:")) { BASE_RIGHT = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("TR:")) { DELAY_90_RIGHT = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("TL:")) { DELAY_90_LEFT = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("TU:")) { DELAY_180_U = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("FD:")) { FRONT_BRAKE_DIST = msg.substring(3).toInt(); printState(); }
        }
        delay(50);
    }
}

bool system_halted = false;

// Strict glitch filtering for walls!
bool is_blocked(VL53L0X_RangingMeasurementData_t measure, int dist, int threshold) {
  if (measure.RangeStatus != 0) return false;
  if (dist < threshold) return true;
  return false;
}

void loop() {
    static bool was_stopped = true;
    
    // Check for Bluetooth Commands on the fly
    if (SerialBT.available()) {
        String msg = SerialBT.readStringUntil('\n');
        msg.trim();
        
        if (msg.equalsIgnoreCase("STOP") && !system_halted) {
            SerialBT.println("\n>>> SYSTEM HALTED! You may now recalibrate. Send 'START' to resume. <<<");
            system_halted = true;
            Serial2.print('S');
        }
        else if (msg.equalsIgnoreCase("START") && system_halted) {
            SerialBT.println("\n>>> RESUMING DRIVE WITH NEW CALIBRATION! <<<");
            system_halted = false;
        }
        else if (system_halted) {
            // Allow recalibration while stopped!
            if (msg.startsWith("BL:")) { BASE_LEFT = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("BR:")) { BASE_RIGHT = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("TR:")) { DELAY_90_RIGHT = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("TL:")) { DELAY_90_LEFT = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("TU:")) { DELAY_180_U = msg.substring(3).toInt(); printState(); }
            else if (msg.startsWith("FD:")) { FRONT_BRAKE_DIST = msg.substring(3).toInt(); printState(); }
        }
    }

    if (system_halted) {
        delay(100);
        return; // Skip driving logic entirely
    }

    VL53L0X_RangingMeasurementData_t measureFront, measureLeft, measureRight;
    sensorFront.rangingTest(&measureFront, false);
    sensorLeft.rangingTest(&measureLeft, false);
    sensorRight.rangingTest(&measureRight, false);
    
    int front_dist = (measureFront.RangeStatus != 0) ? 8190 : measureFront.RangeMilliMeter;
    int left_dist = (measureLeft.RangeStatus != 0) ? 8190 : measureLeft.RangeMilliMeter;
    int right_dist = (measureRight.RangeStatus != 0) ? 8190 : measureRight.RangeMilliMeter;
    
    // Front Brakes
    if (is_blocked(measureFront, front_dist, FRONT_BRAKE_DIST)) {
        Serial2.print('S');
        SerialBT.println("\nCORNER DETECTED! [F:" + String(front_dist) + " L:" + String(left_dist) + " R:" + String(right_dist) + "]");
        delay(2000); // 2 second pause for you to observe!
        
        bool left_blocked = is_blocked(measureLeft, left_dist, 250);
        bool right_blocked = is_blocked(measureRight, right_dist, 250);
        
        if (!right_blocked) {
            SerialBT.println("Blind Turning Right...");
            Serial2.print('R');
            delay(DELAY_90_RIGHT);
        } else if (!left_blocked) {
            SerialBT.println("Blind Turning Left...");
            Serial2.print('L');
            delay(DELAY_90_LEFT);
        } else {
            SerialBT.println("Blind U-Turn...");
            Serial2.print('R');
            delay(DELAY_180_U);
        }
        
        Serial2.print('S');
        SerialBT.println("Turn complete! Check the physical angle!");
        delay(2000); // Pause to check turn accuracy
        was_stopped = true; // Trigger soft start on next loop!
    } 
    else {
        // DRIVE BLINDLY STRAIGHT
        int l_speed = BASE_LEFT; if (l_speed < 0) l_speed = 0; if (l_speed > 255) l_speed = 255;
        int r_speed = BASE_RIGHT; if (r_speed < 0) r_speed = 0; if (r_speed > 255) r_speed = 255;
        
        if (was_stopped) {
            SerialBT.println("SOFT START: Ramping up power to prevent Brown-Out Resets...");
            for (int i = 0; i <= 100; i += 20) { // Ramp up over 5 steps
                Serial2.print('P');
                Serial2.write((uint8_t)((l_speed * i) / 100));
                Serial2.write((uint8_t)((r_speed * i) / 100));
                delay(40);
            }
            was_stopped = false;
        }

        Serial2.print('P');
        Serial2.write((uint8_t)l_speed);
        Serial2.write((uint8_t)r_speed);
        
        // Live print exactly what the car is seeing
        SerialBT.println("BLIND DRIVE: [F:" + String(front_dist) + " L:" + String(left_dist) + " R:" + String(right_dist) + "]");
    }
    
    delay(50);
}
