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

// --- PID CONTROL VARIABLES ---
float Kp = 0.2;  // Proportional Gain (Lowered from 0.5 to stop violent wiggling)
float Kd = 0.1;  // Derivative Gain
float Ki = 0.0;  // Integral Gain
int previous_error = 0;
int total_error = 0;
const int TARGET_DISTANCE = 107; // Perfect center of a 320mm maze
const int BASE_LEFT_SPEED = 145; // Calibrated base speeds
const int BASE_RIGHT_SPEED = 200;

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
  Serial.println("Waiting for Bluetooth connection...");
  while (!SerialBT.hasClient()) {
      delay(500); // Wait infinitely until the user's phone connects!
  }
  delay(1000); // Give the phone terminal an extra second to initialize
  
  unsigned long last_menu_time = 0;
  
  while (true) {
      // Reprint the menu every 5 seconds so it never gets lost!
      if (millis() - last_menu_time > 5000) {
          SerialBT.println("\n--- PID TUNING MENU ---");
          SerialBT.println("Current Values: Kp=" + String(Kp) + " | Kd=" + String(Kd) + " | Ki=" + String(Ki));
          SerialBT.println("To change Kp, send: P:0.3");
          SerialBT.println("To change Kd, send: D:0.2");
          SerialBT.println("To start driving, send: START");
          last_menu_time = millis();
      }
      
      if (SerialBT.available()) {
          String msg = SerialBT.readStringUntil('\n');
          msg.trim();
          
          if (msg.equalsIgnoreCase("START")) {
              SerialBT.println("\n>>> STARTING MAZE SOLVER! <<<");
              break;
          }
          else if (msg.startsWith("P:")) {
              Kp = msg.substring(2).toFloat();
              SerialBT.println("\n>>> Kp UPDATED TO: " + String(Kp));
              last_menu_time = 0; // Force menu to reprint immediately to show new values
          }
          else if (msg.startsWith("D:")) {
              Kd = msg.substring(2).toFloat();
              SerialBT.println("\n>>> Kd UPDATED TO: " + String(Kd));
              last_menu_time = 0; 
          }
          else if (msg.startsWith("I:")) {
              Ki = msg.substring(2).toFloat();
              SerialBT.println("\n>>> Ki UPDATED TO: " + String(Ki));
              last_menu_time = 0;
          }
      }
      delay(50);
  }
}

bool is_blocked(VL53L0X_RangingMeasurementData_t measure, int dist, int threshold) {
  if (measure.RangeStatus == 4) return false;
  if (dist < threshold) return true;
  return false;
}

void loop() {
  VL53L0X_RangingMeasurementData_t measureFront, measureLeft, measureRight;
  
  // Read Front Sensor
  sensorFront.rangingTest(&measureFront, false);
  int front_dist = measureFront.RangeMilliMeter;
  
  // FRONT THRESHOLD: Increased to 260mm! The car is heavy and moving fast, it needs room to stop!
  bool front_blocked = is_blocked(measureFront, front_dist, 260);

  if (front_blocked) {
      // 1. We hit a wall!
      Serial2.print('S');
      
      // Read side sensors to see the exact state of the intersection
      sensorLeft.rangingTest(&measureLeft, false);
      sensorRight.rangingTest(&measureRight, false);
      int left_dist = (measureLeft.RangeStatus == 4) ? 8190 : measureLeft.RangeMilliMeter;
      int right_dist = (measureRight.RangeStatus == 4) ? 8190 : measureRight.RangeMilliMeter;
      
      String s1 = "MAZE: Wall Ahead! STOPPING... [F:" + String(front_dist) + " L:" + String(left_dist) + " R:" + String(right_dist) + "]";
      Serial.println(s1);
      SerialBT.println(s1);
      delay(2000); // DEBUG WAIT: Stop for 2 seconds before deciding so the user can watch!
      
      // SIDE THRESHOLD: Increased to 250mm! 
      // If the car is 105mm wide in a 320mm maze, and drifts all the way to the left, 
      // the right wall is ~215mm away. 250mm guarantees we NEVER mistake a wide hallway for an open intersection!
      bool left_blocked = is_blocked(measureLeft, left_dist, 250);
      bool right_blocked = is_blocked(measureRight, right_dist, 250);

      // 3. The Right-Hand Rule Maze Algorithm
      if (!right_blocked) {
          Serial2.print('R');
          String s6 = "MAZE: Right is open! Turn RIGHT. [L:" + String(left_dist) + " R:" + String(right_dist) + "]";
          Serial.println(s6);
          SerialBT.println(s6);
          delay(390); // 90-degree Right Turn (Restored to overcome static friction!)
      }
      else if (!left_blocked) {
          Serial2.print('L');
          String s7 = "MAZE: Left is open! Turn LEFT. [L:" + String(left_dist) + " R:" + String(right_dist) + "]";
          Serial.println(s7);
          SerialBT.println(s7);
          delay(390); // 90-degree Left Turn (Restored)
      }
      else {
          Serial2.print('R');
          String s8 = "MAZE: Dead End! 180 U-TURN. [L:" + String(left_dist) + " R:" + String(right_dist) + "]";
          Serial.println(s8);
          SerialBT.println(s8);
          delay(780); // 180-degree turn (Restored)
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
      int last_f = (measureFront.RangeStatus == 4) ? 8190 : measureFront.RangeMilliMeter;
      
      while (sweep_count < 15) { // increased safety limit to allow full sweeping
          // Twitch in search direction
          Serial2.print(search_dir); 
          delay(40); // tiny micro-twitch
          Serial2.print('S'); 
          delay(150); // wait for chassis to stop rocking
          
          sensorFront.rangingTest(&measureFront, false);
          int new_f = (measureFront.RangeStatus == 4) ? 8190 : measureFront.RangeMilliMeter;
          
          if (new_f < last_f) {
              // The distance got shorter! We are turning INTO the wall.
              if (sweep_count == 0 && !reversed_once) {
                  // It was our very first guess, and we guessed wrong. 
                  search_dir = (search_dir == 'L') ? 'R' : 'L';
                  reversed_once = true;
                  SerialBT.println("ALIGN: Bad first guess. Reversing sweep to " + String(search_dir));
                  
                  // Undo the bad twitch to return to the starting position
                  Serial2.print(search_dir); delay(40); Serial2.print('S'); delay(150);
                  // Don't update last_f, let it try the new direction on the next loop
              } 
              else {
                  // We were getting better, but now we got worse! We passed the true center!
                  char undo_dir = (search_dir == 'L') ? 'R' : 'L';
                  
                  // Undo the overshoot twitch to return to the absolute peak!
                  Serial2.print(undo_dir); delay(40); Serial2.print('S'); delay(150);
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
      
      if (sweep_count >= 15) {
          SerialBT.println("ALIGN: Sweep limit reached. Resuming.");
      }

      delay(2000); // DEBUG WAIT: Stop for 2 seconds so the user can admire the perfectly aligned car!
  } 
  else {
      // Path is clear! Actively monitor sides while driving forward.
      sensorLeft.rangingTest(&measureLeft, false);
      sensorRight.rangingTest(&measureRight, false);
      
      // --- FULL PID CONTROL LOOP ---
      int left_dist = (measureLeft.RangeStatus == 4) ? 8190 : measureLeft.RangeMilliMeter;
      int right_dist = (measureRight.RangeStatus == 4) ? 8190 : measureRight.RangeMilliMeter;
      
      int error = 0;
      
      // Dynamic Single-Wall Tracking (Handles Open Intersections!)
      if (left_dist < 250 && right_dist < 250) {
          // Dual Wall: standard tracking
          error = left_dist - right_dist;
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
      int new_left = BASE_LEFT_SPEED - adjustment;
      int new_right = BASE_RIGHT_SPEED + adjustment;
      
      // 3. Clamp Speeds (0 to 255) to prevent overflow/stalling
      if (new_left > 255) new_left = 255;
      if (new_left < 0) new_left = 0;
      if (new_right > 255) new_right = 255;
      if (new_right < 0) new_right = 0;
      
      // 4. Send Dynamic PWM Command over UART
      Serial2.print('P');
      Serial2.write((uint8_t)new_left);
      Serial2.write((uint8_t)new_right);
      
      // Print PID telemetry for user debugging!
      String s_pid = "PID -> Err:" + String(error) + " Adj:" + String(adjustment) + " [L:" + String(new_left) + " R:" + String(new_right) + "]";
      Serial.println(s_pid);
      SerialBT.println(s_pid);
  }

  delay(50); // Reduced delay so the loop checks the walls faster
}