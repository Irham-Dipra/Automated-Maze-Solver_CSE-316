#include "Adafruit_VL53L0X.h"
#include <Wire.h>

#define FRONT_XSHUT_PIN 19
#define LEFT_XSHUT_PIN 18
#define RIGHT_XSHUT_PIN 4

Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // UART to ATmega32
  Wire.begin();
  
  Serial.println("\nBooting Phase 3: The Brain...");

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
}

bool is_blocked(VL53L0X_RangingMeasurementData_t measure, int dist, int threshold) {
  if (measure.RangeStatus == 4) return false;
  if (dist <= 20) return false; 
  if (dist < threshold) return true;
  return false;
}

void loop() {
  VL53L0X_RangingMeasurementData_t measureFront, measureLeft, measureRight;
  
  // Read Front Sensor
  sensorFront.rangingTest(&measureFront, false);
  int front_dist = measureFront.RangeMilliMeter;
  
  // FRONT THRESHOLD: We use 220mm (22cm) to give the car enough braking distance so it doesn't crash!
  bool front_blocked = is_blocked(measureFront, front_dist, 220);

  if (front_blocked) {
      // 1. We hit a wall!
      Serial2.print('S');
      Serial.println("MAZE: Wall Ahead! STOPPING to decide...");
      delay(200); // Snappy reaction time
      
      // 2. Read side sensors while stopped
      sensorLeft.rangingTest(&measureLeft, false);
      sensorRight.rangingTest(&measureRight, false);
      int left_dist = measureLeft.RangeMilliMeter;
      int right_dist = measureRight.RangeMilliMeter;
      
      // SIDE THRESHOLD: We use 150mm (15cm) to check if the side paths are genuinely blocked
      bool left_blocked = is_blocked(measureLeft, left_dist, 150);
      bool right_blocked = is_blocked(measureRight, right_dist, 150);

      // 3. The Right-Hand Rule Maze Algorithm
      if (!right_blocked) {
          Serial2.print('R');
          Serial.println("MAZE: Right is open! Turning RIGHT 90 deg.");
          delay(257); // 90-degree turn
      }
      else if (!left_blocked) {
          Serial2.print('L');
          Serial.println("MAZE: Right blocked, Left is open! Turning LEFT 90 deg.");
          delay(257); // 90-degree turn
      }
      else {
          Serial2.print('R');
          Serial.println("MAZE: Dead End! Doing a 180 U-TURN.");
          delay(514); // 180 degrees
      }
      
      // 4. Brief stop to prevent slipping before going forward
      Serial2.print('S');
      delay(200); 
  } 
  else {
      // Path is clear! Actively monitor sides while driving forward.
      sensorLeft.rangingTest(&measureLeft, false);
      sensorRight.rangingTest(&measureRight, false);
      
      int left_dist = measureLeft.RangeMilliMeter;
      int right_dist = measureRight.RangeMilliMeter;
      
      // Drift Correction: Bang-Bang Control
      // If drifting too close to LEFT wall (< 80mm)
      if (measureLeft.RangeStatus != 4 && left_dist > 20 && left_dist < 80) {
          Serial2.print('R'); // Twitch Right
          Serial.println("DRIFT: Too close to Left wall! Twitching Right.");
          delay(40); // Restored to 40ms
          Serial2.print('F'); 
          delay(150); // COOLDOWN: Drive forward for 150ms to actually escape the wall before checking again!
      }
      // If drifting too close to RIGHT wall (< 80mm)
      else if (measureRight.RangeStatus != 4 && right_dist > 20 && right_dist < 80) {
          Serial2.print('L'); // Twitch Left
          Serial.println("DRIFT: Too close to Right wall! Twitching Left.");
          delay(40); // Restored to 40ms
          Serial2.print('F'); 
          delay(150); // COOLDOWN: Drive forward for 150ms
      }
      
      // Resume forward driving
      Serial2.print('F');
      Serial.println("MAZE: Path Clear -> FORWARD");
  }

  delay(50); // Reduced delay so the loop checks the walls faster
}