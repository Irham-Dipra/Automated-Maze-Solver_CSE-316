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

// --- MEMORY SYSTEM ---
char path[100];
int pathLength = 0;
bool mazeComplete = false;

void recordTurn(char move) {
  if (pathLength < 100) {
    path[pathLength] = move;
    pathLength++;
    
    Serial.print("MEMORY SAVED: ");
    SerialBT.print("MEMORY SAVED: ");
    for (int i=0; i<pathLength; i++) {
        Serial.print(path[i]);
        SerialBT.print(path[i]);
    }
    Serial.println();
    SerialBT.println();
  }
}

bool checkMazeComplete(int front, int left, int right) {
  // If all 3 sensors read infinite space (>400mm), the car has driven out of the maze into the open room!
  if (front > 400 && left > 400 && right > 400) {
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // UART to ATmega32
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
  
  // --- STARTUP DELAY ---
  // Give the user 10 seconds to place the car in the maze before it starts thinking!
  Serial.println("\nSensors Booted. Please place car in the maze!");
  SerialBT.println("\nSensors Booted. Please place car in the maze!");
  
  for(int i = 10; i > 0; i--) {
      Serial.print("Starting in "); Serial.print(i); Serial.println("...");
      SerialBT.print("Starting in "); SerialBT.print(i); SerialBT.println("...");
      delay(1000);
  }
  
  Serial.println("GO!");
  SerialBT.println("GO!");
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
  if (measureFront.RangeStatus == 4) front_dist = 1000;
  
  // Read side sensors immediately for finish line check
  sensorLeft.rangingTest(&measureLeft, false);
  sensorRight.rangingTest(&measureRight, false);
  int left_dist = measureLeft.RangeMilliMeter;
  int right_dist = measureRight.RangeMilliMeter;
  if (measureLeft.RangeStatus == 4) left_dist = 1000;
  if (measureRight.RangeStatus == 4) right_dist = 1000;

  // --- 1. FINISH LINE CHECK ---
  if (!mazeComplete && checkMazeComplete(front_dist, left_dist, right_dist)) {
      mazeComplete = true;
      Serial2.print('S'); // Stop the car
      
      Serial.println("\n=========================");
      Serial.println("🏆 MAZE COMPLETE! 🏆");
      Serial.print("FINAL PATH: ");
      
      SerialBT.println("\n=========================");
      SerialBT.println("🏆 MAZE COMPLETE! 🏆");
      SerialBT.print("FINAL PATH: ");
      
      for (int i=0; i<pathLength; i++) {
          Serial.print(path[i]);
          SerialBT.print(path[i]);
      }
      
      Serial.println("\n=========================");
      SerialBT.println("\n=========================");
      
      while(1) { delay(1000); } // Infinite halt
  }
  
  // FRONT THRESHOLD: We use 220mm (22cm) to give the car enough braking distance so it doesn't crash!
  bool front_blocked = is_blocked(measureFront, front_dist, 220);

  if (front_blocked) {
      // 1. We hit a wall!
      Serial2.print('S');
      Serial.println("MAZE: Wall Ahead! STOPPING to decide...");
      SerialBT.println("MAZE: Wall Ahead! STOPPING to decide...");
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
          SerialBT.println("MAZE: Right is open! Turning RIGHT 90 deg.");
          recordTurn('R');
          delay(257); // 90-degree turn
      }
      else if (!left_blocked) {
          Serial2.print('L');
          Serial.println("MAZE: Right blocked, Left is open! Turning LEFT 90 deg.");
          SerialBT.println("MAZE: Right blocked, Left is open! Turning LEFT 90 deg.");
          recordTurn('L');
          delay(257); // 90-degree turn
      }
      else {
          Serial2.print('R');
          Serial.println("MAZE: Dead End! Doing a 180 U-TURN.");
          SerialBT.println("MAZE: Dead End! Doing a 180 U-TURN.");
          recordTurn('U');
          delay(463); // Tuned for momentum
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
      if (measureLeft.RangeStatus != 4 && left_dist < 80) {
          Serial2.print('R'); // Twitch Right
          Serial.println("DRIFT: Too close to Left wall! Twitching Right.");
          SerialBT.println("DRIFT: Too close to Left wall! Twitching Right.");
          delay(80); // Increased to 80ms to overcome motor friction!
          Serial2.print('F'); 
          delay(150); // COOLDOWN: Drive forward for 150ms to actually escape the wall before checking again!
      }
      // If drifting too close to RIGHT wall (< 80mm)
      else if (measureRight.RangeStatus != 4 && right_dist < 80) {
          Serial2.print('L'); // Twitch Left
          Serial.println("DRIFT: Too close to Right wall! Twitching Left.");
          SerialBT.println("DRIFT: Too close to Right wall! Twitching Left.");
          delay(80); // Increased to 80ms!
          Serial2.print('F'); 
          delay(150); // COOLDOWN: Drive forward for 150ms
      }
      
      // Resume forward driving
      else {
          Serial2.print('F');
          // Removed constant 'FORWARD' logging here because it spams the Bluetooth screen 20 times a second!
      }
  }

  delay(50); // Reduced delay so the loop checks the walls faster
}