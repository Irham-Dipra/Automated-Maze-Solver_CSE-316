#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int BASE_LEFT_SPEED = 145; 
const int BASE_RIGHT_SPEED = 145; // Lower this from 200 to match the left side!


#define FRONT_XSHUT_PIN 19
#define LEFT_XSHUT_PIN 18
#define RIGHT_XSHUT_PIN 4

Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X();

// --- PID CONTROL VARIABLES ---
float Kp = 0.2;  // Proportional Gain
float Kd = 0.1;  // Derivative Gain
float Ki = 0.0;  // Integral Gain
int previous_error = 0;
int total_error = 0;
const int TARGET_DISTANCE = 128; // Perfect center of your 360mm maze! (360 - 105) / 2

// --- DYNAMIC CALIBRATION VARIABLES ---
int BASE_LEFT = 200; 
int BASE_RIGHT = 30;
int TURN_L = 180;   // Power needed to overcome sideways friction during turns!
int TURN_R = 150;   // Power needed to overcome sideways friction during turns!
int DELAY_90_RIGHT = 390;
int DELAY_90_LEFT = 390;
int DELAY_180_U = 780;
int FRONT_BRAKE_DIST = 260;

bool system_halted = false;

// Function to print current state
void printState() {
    SerialBT.println("\nCurrent: BL=" + String(BASE_LEFT) + " | BR=" + String(BASE_RIGHT) + 
                     " | XL=" + String(TURN_L) + " | XR=" + String(TURN_R) +
                     " | TR=" + String(DELAY_90_RIGHT) + " | TL=" + String(DELAY_90_LEFT) + 
                     " | TU=" + String(DELAY_180_U) + " | FD=" + String(FRONT_BRAKE_DIST) +
                     " | P=" + String(Kp) + " | D=" + String(Kd) + " | I=" + String(Ki));
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // UART to ATmega32
  Serial2.print('S'); // FORCE STOP ATMEGA: In case ESP32 was reset while driving!
  SerialBT.begin("Maze_Robot_Dipra"); // Bluetooth Name!
  Wire.begin();
  
  Serial.println("\nBooting Phase 3: The Brain...");
  SerialBT.println("\nBluetooth Connected! Booting Brain...");

  // Boot Sensors
  pinMode(FRONT_XSHUT_PIN, OUTPUT);
  pinMode(LEFT_XSHUT_PIN, OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);

  digitalWrite(FRONT_XSHUT_PIN, LOW);
  digitalWrite(LEFT_XSHUT_PIN, LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);
  delay(10); 

  digitalWrite(FRONT_XSHUT_PIN, HIGH);
  delay(10);
  sensorFront.begin(0x30);
 
  digitalWrite(LEFT_XSHUT_PIN, HIGH);
  delay(10);
  sensorLeft.begin(0x31);

  digitalWrite(RIGHT_XSHUT_PIN, HIGH);
  delay(10);
  sensorRight.begin(0x32);
  
  // --- INTERACTIVE STARTUP PROMPT ---
  SerialBT.println("Waiting for Bluetooth connection...");
  while (!SerialBT.hasClient()) delay(500);
  delay(1000);
  
  SerialBT.println("\n=== PHASE 3 (PID) CALIBRATION MENU ===");
  SerialBT.println("  BL:180  -> Base Left Speed (Straight)");
  SerialBT.println("  BR:10   -> Base Right Speed (Straight)");
  SerialBT.println("  XL:180  -> Turn Left Power (Pivot)");
  SerialBT.println("  XR:150  -> Turn Right Power (Pivot)");
  SerialBT.println("  FD:260  -> Front Brake Distance");
  SerialBT.println("  P:0.3   -> PID P-Gain");
  SerialBT.println("  D:0.2   -> PID D-Gain");
  SerialBT.println("  START   -> Begin driving");
  SerialBT.println("  STOP    -> Halt the car (on the fly)");
  SerialBT.println("======================================");
  printState();
  
  bool is_running = false;
  while (!is_running) {
      if (SerialBT.available()) {
          String msg = SerialBT.readStringUntil('\n');
          msg.trim();
          
          if (msg.equalsIgnoreCase("START")) {
              SerialBT.println("\n>>> STARTING PID MAZE SOLVER! <<<");
              previous_error = 0;
              total_error = 0;
              is_running = true;
          }
          else if (msg.startsWith("BL:")) { BASE_LEFT = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("BR:")) { BASE_RIGHT = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("XL:")) { TURN_L = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("XR:")) { TURN_R = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("TR:")) { DELAY_90_RIGHT = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("TL:")) { DELAY_90_LEFT = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("TU:")) { DELAY_180_U = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("FD:")) { FRONT_BRAKE_DIST = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("P:")) { Kp = msg.substring(2).toFloat(); printState(); }
          else if (msg.startsWith("D:")) { Kd = msg.substring(2).toFloat(); printState(); }
          else if (msg.startsWith("I:")) { Ki = msg.substring(2).toFloat(); printState(); }
      }
      delay(50);
  }
}

bool is_blocked(VL53L0X_RangingMeasurementData_t measure, int dist, int threshold) {
  if (measure.RangeStatus != 0) return false; // Ignore glitches (1, 2, 3) AND Out of Range (4)!
  if (dist < threshold) return true;
  return false;
}

void loop() {
  static int power_ramp = 30; // Start at 30% power to prevent BOR, ramp up dynamically!
  
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
          previous_error = 0;
          total_error = 0;
          power_ramp = 30; // Reset soft start!
          system_halted = false;
      }
      else if (system_halted) {
          // Allow recalibration while stopped!
          if (msg.startsWith("BL:")) { BASE_LEFT = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("BR:")) { BASE_RIGHT = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("XL:")) { TURN_L = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("XR:")) { TURN_R = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("TR:")) { DELAY_90_RIGHT = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("TL:")) { DELAY_90_LEFT = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("TU:")) { DELAY_180_U = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("FD:")) { FRONT_BRAKE_DIST = msg.substring(3).toInt(); printState(); }
          else if (msg.startsWith("P:")) { Kp = msg.substring(2).toFloat(); printState(); }
          else if (msg.startsWith("D:")) { Kd = msg.substring(2).toFloat(); printState(); }
          else if (msg.startsWith("I:")) { Ki = msg.substring(2).toFloat(); printState(); }
      }
  }

  if (system_halted) {
      delay(100);
      return; // Skip driving logic entirely
  }

  VL53L0X_RangingMeasurementData_t measureFront, measureLeft, measureRight;
  
  // Read Front Sensor
  sensorFront.rangingTest(&measureFront, false);
  int front_dist = measureFront.RangeMilliMeter;
  
  // FRONT THRESHOLD
  bool front_blocked = is_blocked(measureFront, front_dist, FRONT_BRAKE_DIST);

  if (front_blocked) {
      // 1. We hit a wall!
      Serial2.print('S');
      
      // Read side sensors to see the exact state of the intersection
      sensorLeft.rangingTest(&measureLeft, false);
      sensorRight.rangingTest(&measureRight, false);
      int left_dist = (measureLeft.RangeStatus != 0) ? 8190 : measureLeft.RangeMilliMeter;
      int right_dist = (measureRight.RangeStatus != 0) ? 8190 : measureRight.RangeMilliMeter;
      
      String s1 = "MAZE: Wall Ahead! STOPPING... [F:" + String(front_dist) + " L:" + String(left_dist) + " R:" + String(right_dist) + "]";
      Serial.println(s1);
      SerialBT.println(s1);
      delay(2000); // DEBUG WAIT: Stop for 2 seconds before deciding so the user can watch!
      
      // SIDE THRESHOLD
      bool left_blocked = is_blocked(measureLeft, left_dist, 250);
      bool right_blocked = is_blocked(measureRight, right_dist, 250);

      // 3. The Right-Hand Rule Maze Algorithm
      if (!right_blocked) {
          Serial2.print('R');
          Serial2.write((uint8_t)TURN_L);
          Serial2.write((uint8_t)TURN_R);
          String s6 = "MAZE: Right is open! Turn RIGHT. [L:" + String(left_dist) + " R:" + String(right_dist) + "]";
          Serial.println(s6);
          SerialBT.println(s6);
          delay(DELAY_90_RIGHT);
      }
      else if (!left_blocked) {
          Serial2.print('L');
          Serial2.write((uint8_t)TURN_L);
          Serial2.write((uint8_t)TURN_R);
          String s7 = "MAZE: Left is open! Turn LEFT. [L:" + String(left_dist) + " R:" + String(right_dist) + "]";
          Serial.println(s7);
          SerialBT.println(s7);
          delay(DELAY_90_LEFT);
      }
      else {
          Serial2.print('R');
          Serial2.write((uint8_t)TURN_L);
          Serial2.write((uint8_t)TURN_R);
          String s8 = "MAZE: Dead End! 180 U-TURN. [L:" + String(left_dist) + " R:" + String(right_dist) + "]";
          Serial.println(s8);
          SerialBT.println(s8);
          delay(DELAY_180_U);
      }
      
      // 4. Stop motors after the primary turn
      Serial2.print('S');
      delay(300); // Let momentum settle
      
      // --- POST-TURN AUTO ALIGNMENT (Absolute Peak Radar Sweep) ---
      SerialBT.println("ALIGN: Radar Sweep searching for absolute peak...");
      int sweep_count = 0;
      char search_dir = 'L'; // Arbitrarily guess left first
      bool reversed_once = false;
      
      sensorFront.rangingTest(&measureFront, false);
      int last_f = (measureFront.RangeStatus != 0) ? 8190 : measureFront.RangeMilliMeter;
      
      while (sweep_count < 20) { // Increased safety limit
          // Twitch in search direction
          Serial2.print(search_dir);
          Serial2.write((uint8_t)TURN_L);
          Serial2.write((uint8_t)TURN_R); 
          delay(60); // INCREASED twitch time so the heavy car actually moves!
          Serial2.print('S'); 
          delay(150); // wait for chassis to stop rocking
          
          sensorFront.rangingTest(&measureFront, false);
          int new_f = (measureFront.RangeStatus != 0) ? 8190 : measureFront.RangeMilliMeter;
          
          SerialBT.println("  [Sweep " + String(sweep_count+1) + "] Twitched " + String(search_dir) + " -> New F:" + String(new_f));
          
          if (new_f < last_f) {
              // The distance got shorter! We are turning INTO the wall.
              if (sweep_count == 0 && !reversed_once) {
                  // It was our very first guess, and we guessed wrong. 
                  search_dir = (search_dir == 'L') ? 'R' : 'L';
                  reversed_once = true;
                  SerialBT.println("  ALIGN: Bad first guess. Reversing sweep to " + String(search_dir));
                  
                  // Undo the bad twitch to return to the starting position
                  Serial2.print(search_dir); Serial2.write((uint8_t)TURN_L); Serial2.write((uint8_t)TURN_R);
                  delay(60); Serial2.print('S'); delay(150);
                  // Don't update last_f, let it try the new direction on the next loop
              } 
              else {
                  // We were getting better, but now we got worse! We passed the true center!
                  char undo_dir = (search_dir == 'L') ? 'R' : 'L';
                  
                  // Undo the overshoot twitch to return to the absolute peak!
                  Serial2.print(undo_dir); Serial2.write((uint8_t)TURN_L); Serial2.write((uint8_t)TURN_R);
                  delay(60); Serial2.print('S'); delay(150);
                  SerialBT.println("ALIGN: Peak found & Locked! [F:" + String(last_f) + "]");
                  break;
              }
          }
          else if (new_f == last_f && new_f == 8190) {
              // We hit the physical limit of the sensor twice. We can't optimize any further.
              SerialBT.println("ALIGN: Max range limit reached! Locked at [F:8190]");
              break;
          }
          else {
              // We got better! Keep going this way!
              last_f = new_f;
          }
          
          sweep_count++;
      }
      
      if (sweep_count >= 20) {
          SerialBT.println("ALIGN: Sweep limit reached. Resuming.");
      }

      // RESET PID MEMORY SO IT DOESN'T SWERVE AFTER THE TURN!
      previous_error = 0;
      total_error = 0;
      power_ramp = 30; // Trigger non-blocking soft start

      delay(2000); // DEBUG WAIT: Stop for 2 seconds so the user can admire the perfectly aligned car!
  } 
  else {
      // Path is clear! Actively monitor sides while driving forward.
      sensorLeft.rangingTest(&measureLeft, false);
      sensorRight.rangingTest(&measureRight, false);
      
      // --- FULL PID CONTROL LOOP ---
      int left_dist = (measureLeft.RangeStatus != 0) ? 8190 : measureLeft.RangeMilliMeter;
      int right_dist = (measureRight.RangeStatus != 0) ? 8190 : measureRight.RangeMilliMeter;
      
      int error = 0;
      
      // Dynamic Single-Wall Tracking (Handles Open Intersections!)
      if (left_dist < 250 && right_dist < 250) {
          // Dual Wall: standard tracking
          error = (left_dist - right_dist) / 2;
      } 
      else if (left_dist < 250 && right_dist >= 250) {
          // Right wall missing! Hug the left wall
          error = left_dist - TARGET_DISTANCE;
      } 
      else if (right_dist < 250 && left_dist >= 250) {
          // Left wall missing! Hug the right wall
          error = TARGET_DISTANCE - right_dist;
      } 
      else {
          // Both missing (Wide open area): Drive perfectly straight without steering
          error = 0;
      }
      
      // 1. Calculate PID Math
      float P = Kp * error;
      float D = Kd * (error - previous_error);
      total_error += error;
      float I = Ki * total_error;
      
      int adjustment = (int)(P + I + D);
      previous_error = error;
      
      // 2. Apply Adjustment to Calibrated Base Speeds
      int new_left = BASE_LEFT - adjustment;
      int new_right = BASE_RIGHT + adjustment;
      
      // 3. Clamp Speeds (0 to 255) Since ATmega UART timeout handles 0!
      if (new_left > 255) new_left = 255;
      if (new_left < 0) new_left = 0;
      if (new_right > 255) new_right = 255;
      if (new_right < 0) new_right = 0;
      
      // --- DYNAMIC NON-BLOCKING SOFT START ---
      // This allows the PID to aggressively steer the car *WHILE* ramping up power!
      if (power_ramp < 100) {
          power_ramp += 10; // Ramp up over ~7 cycles
      }
      
      int final_left = (new_left * power_ramp) / 100;
      int final_right = (new_right * power_ramp) / 100;
      
      // 4. Send Dynamic PWM Command over UART
      Serial2.print('P');
      Serial2.write((uint8_t)final_left);
      Serial2.write((uint8_t)final_right);
      
      // Determine Steering Direction for Logs
      String steer_dir = "(Straight)";
      if (adjustment < -10) steer_dir = "(Steering Right)";
      else if (adjustment > 10) steer_dir = "(Steering Left)";
      
      // Print PID telemetry for user debugging!
      String s_pid = "PID -> Err:" + String(error) + " Adj:" + String(adjustment) + " " + steer_dir + " [PWM_L:" + String(final_left) + " PWM_R:" + String(final_right) + "]";
      Serial.println(s_pid);
      SerialBT.println(s_pid);
  }

  delay(50); // Reduced delay so the loop checks the walls faster
}