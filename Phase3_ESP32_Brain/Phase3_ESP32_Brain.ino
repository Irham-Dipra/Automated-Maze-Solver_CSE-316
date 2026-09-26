#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "BluetoothSerial.h"
#include "esp_log.h"   // Needed to redirect Wire.cpp errors to BT

BluetoothSerial SerialBT;

// ============================================================
//  ESP SYSTEM LOG → BLUETOOTH MIRROR
//  This hooks into the ESP32 internal logging system so that
//  Wire.cpp / I2C errors (Error 263 etc.) ALSO appear in BT.
// ============================================================
static int esp_log_to_bt(const char *fmt, va_list args) {
    // Print to USB Serial (normal behavior)
    int ret = vprintf(fmt, args);
    // Also forward to BT when connected
    if (SerialBT.hasClient()) {
        char buf[256];
        va_list args2;
        va_copy(args2, args);
        vsnprintf(buf, sizeof(buf), fmt, args2);
        va_end(args2);
        SerialBT.print(buf);
    }
    return ret;
}

#define FRONT_XSHUT_PIN 19
#define LEFT_XSHUT_PIN 18
#define RIGHT_XSHUT_PIN 4

Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X();

// ---------------- MPU6050 Gyroscope Variables ----------------
#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_GYRO_ZOUT_H 0x47

const float GYRO_SENS = 131.0;
float gyroZoffset   = 0;
float yaw           = 0;
unsigned long lastGyroTime = 0;

// ---------------- PID CONTROL VARIABLES ----------------
// Kp: Proportional. Higher = faster correction but oscillates. Start LOW.
// Kd: Damping. Raise when Kp causes left-right swinging.
// Rule of thumb: if car oscillates, raise Kd. If car drifts slowly, raise Kp.
float Kp = 1.7;  // Proportional Gain
float Kd = 3.5;  // Derivative Gain — strong damping
float Ki = 0.0;  // Integral Gain  — leave at 0
float headingIntegral = 0;
float lastHeadingError = 0;

// ---------------- DYNAMIC CALIBRATION VARIABLES ----------------
int BASE_LEFT  = 180;
int BASE_RIGHT = 175; // Slightly lower: right motor is physically stronger, this equalizes drift
int TURN_L = 170;     // Turn power: high enough to spin, low enough not to break I2C
int TURN_R = 170;     // If turns still timeout, raise by 10. If I2C fails, lower by 10.
int FRONT_BRAKE_DIST = 260;

// Cached side sensor distances — updated whenever side sensors are read.
// Shown in PID log for debugging without reading sensors on every straight cycle.
int last_left_dist  = 8190;
int last_right_dist = 8190;

// ============================================================
//  NON-BLOCKING LOG HELPER
//  Sends to USB Serial AND Bluetooth (only when a client is connected).
//  The car NEVER waits for Bluetooth — it is purely a passive observer.
// ============================================================
void LOG(const String &msg) {
    Serial.println(msg);
    if (SerialBT.hasClient()) {
        SerialBT.println(msg);
    }
}

void sendMotorCmd(char cmd, int left, int right) {
    Serial2.print(cmd);
    Serial2.write((uint8_t)left);
    Serial2.write((uint8_t)right);
}

// ============================================================
//  I2C BUS RECOVERY
//  Sends 9 SCL clock pulses to unstick a slave that crashed
//  mid-transaction and is holding SDA low (causes Error 263).
//  Call this BEFORE Wire.begin() and sensor init.
// ============================================================
void i2cBusRecover() {
    LOG("I2C: Running bus recovery (9-pulse SCL reset)...");
    // ESP32 default I2C pins: SDA=21, SCL=22
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    digitalWrite(21, HIGH);  // SDA high
    for (int i = 0; i < 9; i++) {
        digitalWrite(22, LOW);  delayMicroseconds(10);
        digitalWrite(22, HIGH); delayMicroseconds(10);
    }
    // Send STOP condition: SDA goes HIGH while SCL is HIGH
    digitalWrite(21, LOW);  delayMicroseconds(10);
    digitalWrite(21, HIGH); delayMicroseconds(10);
    Wire.begin(); // Re-init Wire after manual bus manipulation
    delay(50);
    LOG("I2C: Bus recovery complete.");
}

// ============================================================
//  SENSOR INIT WITH TIMEOUT
//  Tries to begin() the sensor for up to 3 seconds.
//  Returns true on success, false if the sensor is dead/missing.
// ============================================================
bool initSensor(Adafruit_VL53L0X &sensor, uint8_t addr, const char* name) {
    unsigned long start = millis();
    while (millis() - start < 3000) {
        if (sensor.begin(addr)) {
            LOG("  [OK] Sensor " + String(name) + " ready at 0x" + String(addr, HEX));
            return true;
        }
        delay(200);
    }
    LOG("  [FAIL] Sensor " + String(name) + " NOT FOUND at 0x" + String(addr, HEX) + "! Check wiring.");
    return false;
}



// ---------------- MPU6050 raw register driver ----------------
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);         // wake the chip up
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_CONFIG);
  Wire.write(0x00);         // +/-250 deg/s range
  Wire.endTransmission();

  lastGyroTime = micros();
}

int16_t readGyroZraw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  // Check return value: if 0 bytes received, I2C failed — return 0 (assume no rotation)
  // This prevents yaw corruption if I2C glitches during a spin.
  uint8_t bytesReceived = Wire.requestFrom(MPU_ADDR, 2);
  if (bytesReceived < 2) {
    return 0;  // Safe fallback: treat as zero rotation instead of garbage
  }
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw;
}

void calibrateGyro(int samples = 500) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readGyroZraw();
    delay(3);
  }
  gyroZoffset = sum / (float)samples;
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastGyroTime) / 1000000.0;
  lastGyroTime = now;

  float gz = (readGyroZraw() - gyroZoffset) / GYRO_SENS;
  yaw += gz * dt;
}

int computeHeadingCorrection(float targetYaw) {
  float error = targetYaw - yaw;

  headingIntegral += error;
  headingIntegral = constrain(headingIntegral, -50, 50);

  float derivative = error - lastHeadingError;
  lastHeadingError = error;

  float correction = Kp * error + Ki * headingIntegral + Kd * derivative;

  // HARD CLAMP: Limit maximum correction to ±35 PWM.
  // This prevents one motor from fully saturating (e.g. 255/40 split)
  // which causes violent overshoots and oscillation.
  correction = constrain(correction, -35.0, 35.0);

  return (int)correction;
}

void turnToAngle(float targetDelta, int turnPower) {
  float target   = yaw + targetDelta;
  float error    = target - yaw;
  char  turn_dir = (error > 0) ? 'L' : 'R';

  float    yawAtStart      = yaw;
  float    lastYawSnapshot = yaw;
  int      frozenCycles    = 0;
  bool     gyroWorking     = true;
  int      iter            = 0;

  unsigned long turnTimeout = constrain((unsigned long)(abs(targetDelta) * 67), 6000, 12000);
  unsigned long turnStart   = millis();

  LOG("TURN START: " + String(targetDelta, 0) + "deg dir=" + String(turn_dir) +
      " Yaw=" + String(yaw, 1) + " Target=" + String(target, 1));

  // ── PHASE 1: Gyro-guided closed-loop turn ─────────────────────────────
  // Watch BT: if [Tx] lines show RawZ=0 and Yaw frozen → gyro I2C failing.
  // If Yaw changes each iteration → gyro is working, turn should complete.
  while (abs(error) > 1.0) {
    if (millis() - turnStart > turnTimeout) {
      LOG("TURN: TIMEOUT. Gyro OK=" + String(gyroWorking) +
          " YawMoved=" + String(yaw - yawAtStart, 1) + "deg");
      break;
    }

    updateYaw();
    error = target - yaw;

    // Detect gyro I2C freeze: yaw not changing for 5 consecutive cycles
    if (abs(yaw - lastYawSnapshot) < 0.05) {
      frozenCycles++;
    } else {
      frozenCycles    = 0;
      lastYawSnapshot = yaw;
    }

    if (frozenCycles >= 5) {
      gyroWorking = false;
      LOG("TURN: Gyro I2C DEAD during spin! Yaw frozen at " + String(yaw, 1) +
          ". Switching to TIMED fallback.");
      break;
    }

    // Diagnostic log every 10 iterations so you can track in BT monitor
    if (iter % 10 == 0) {
      int16_t rawZ = readGyroZraw();
      LOG("  [T" + String(iter) + "] Yaw:" + String(yaw, 2) +
          " Err:" + String(error, 1) +
          " RawZ:" + String(rawZ) +
          " ms:" + String(millis() - turnStart));
    }
    iter++;

    int power = constrain(map(abs(error), 0, 90, 40, turnPower), 40, turnPower);
    sendMotorCmd(turn_dir, power, power);
    delay(5);
  }

  sendMotorCmd('S', 0, 0);
  delay(100);

  // ── PHASE 2: Timed open-loop fallback (only if gyro froze in Phase 1) ─
  // ms_per_deg: how many milliseconds per degree of turn at turnPower PWM.
  // DEFAULT = 7ms/deg. YOU MUST CALIBRATE THIS:
  //   1. Set a fixed 90-degree turn
  //   2. Time how long it physically takes at TURN_L/TURN_R = 170 PWM
  //   3. Divide: if 90deg takes 900ms, ms_per_deg = 10.0
  if (!gyroWorking) {
    float         degRemaining = abs(target - yaw);
    const float   ms_per_deg   = 7.0;   // ← CALIBRATE THIS (ms per degree at 170 PWM)
    unsigned long timedMs      = constrain((unsigned long)(degRemaining * ms_per_deg), 200, 5000);

    LOG("TURN TIMED: " + String(degRemaining, 0) + "deg x " +
        String(ms_per_deg) + "ms/deg = " + String(timedMs) + "ms");

    sendMotorCmd(turn_dir, turnPower, turnPower);
    delay(timedMs);
    sendMotorCmd('S', 0, 0);
    delay(100);
    LOG("TURN TIMED: Done.");
  }

  yaw = target;   // Snap to intended heading for next straight
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // UART to ATmega32
  sendMotorCmd('S', 0, 0); // FORCE STOP ATMEGA: In case ESP32 was reset while driving!

  // Start Bluetooth — NON-BLOCKING. Car does NOT wait for a client!
  SerialBT.begin("MazeSolver_ESP32");

  // Hook ESP32 internal logger → also sends Wire/I2C errors to BT
  esp_log_set_vprintf(esp_log_to_bt);

  LOG("\nBooting Phase 4: Gyro Heading-Hold Brain...");
  LOG("Bluetooth: Connect to 'MazeSolver_ESP32' for live logs.");
  delay(1000); // Give hardware a second to stabilize

  LOG("Booting Sensors...");

  // --- Boot ToF Sensors with I2C bus recovery ---
  // Pull ALL XSHUT pins LOW to reset all sensors at once
  pinMode(FRONT_XSHUT_PIN, OUTPUT);
  pinMode(LEFT_XSHUT_PIN,  OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);
  digitalWrite(FRONT_XSHUT_PIN, LOW);
  digitalWrite(LEFT_XSHUT_PIN,  LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);
  delay(50); // Hold in reset

  // Recover I2C bus BEFORE bringing sensors out of reset
  // (clears any glitch-lock from previous power cycle)
  i2cBusRecover();

  // Bring sensors up ONE AT A TIME and assign unique addresses
  digitalWrite(FRONT_XSHUT_PIN, HIGH); delay(10);
  initSensor(sensorFront, 0x30, "FRONT");

  digitalWrite(LEFT_XSHUT_PIN, HIGH); delay(10);
  initSensor(sensorLeft, 0x31, "LEFT");

  digitalWrite(RIGHT_XSHUT_PIN, HIGH); delay(10);
  initSensor(sensorRight, 0x32, "RIGHT");

  delay(1000); // Give sensors a moment to stabilize

  LOG("======================================");
  LOG("Place the car on a flat surface for gyroscope calibration.");
  LOG("Do NOT touch the car during calibration!");
  LOG("Calibration is starting after 3 seconds...");
  LOG("======================================");

  delay(3000); // Give user time to place car on a flat surface

  LOG("======================================");
  LOG("3 second is over.");
  LOG("======================================");
  
  LOG("======================================");
  LOG("CALIBRATING GYROSCOPE...");
  LOG("======================================");

  mpuInit();
  calibrateGyro(500); // Takes ~1.5 seconds
  LOG("\nGyroscope Calibrated!");

  LOG("\n>>> POST-CALIBRATION: STARTING MAZE SOLVER IN 1 SECOND... <<<");
  delay(1000);

  headingIntegral = 0;
  lastHeadingError = 0;
  yaw = 0.0; // Reset heading to exactly straight ahead!
  lastGyroTime = micros(); // Reset gyro timer!

  LOG("\n>>> GO! <<<");
}

bool is_blocked(VL53L0X_RangingMeasurementData_t measure, int dist, int threshold) {
  if (measure.RangeStatus != 0) return false; // Ignore glitches (1, 2, 3) AND Out of Range (4)!
  if (dist < threshold) return true;
  return false;
}

void loop() {
  static int power_ramp = 30; // Start at 30% power to prevent BOR, ramp up dynamically!
  
  // Continuously track heading
  updateYaw();

  VL53L0X_RangingMeasurementData_t measureFront, measureLeft, measureRight;
  
  // ── READ ALL 3 SENSORS CONSTANTLY FOR DEBUGGING ──
  sensorFront.rangingTest(&measureFront, false);
  sensorLeft.rangingTest(&measureLeft, false);
  sensorRight.rangingTest(&measureRight, false);
  
  int front_dist = (measureFront.RangeStatus != 0) ? 8190 : measureFront.RangeMilliMeter;
  int left_dist  = (measureLeft.RangeStatus  != 0) ? 8190 : measureLeft.RangeMilliMeter;
  int right_dist = (measureRight.RangeStatus != 0) ? 8190 : measureRight.RangeMilliMeter;
  
  last_left_dist  = left_dist;
  last_right_dist = right_dist;
  
  // FRONT THRESHOLD
  bool front_blocked = is_blocked(measureFront, front_dist, FRONT_BRAKE_DIST);

  if (front_blocked) {
      // 1. We hit a wall!
      sendMotorCmd('S', 0, 0);
      
      LOG("MAZE: Wall Ahead! STOPPING... [F:" + String(front_dist) + " L:" + String(left_dist) + " R:" + String(right_dist) + "]");
      delay(1000); // Stop for 1 second before deciding so the user can watch!
      
      // Update Yaw while we wait just in case of drift
      updateYaw(); 
      
      // SIDE THRESHOLD
      bool left_blocked = is_blocked(measureLeft, left_dist, 250);
      bool right_blocked = is_blocked(measureRight, right_dist, 250);

      // 3. The Right-Hand Rule Maze Algorithm using Closed-Loop Gyro Turns!
      if (!right_blocked) {
          LOG("MAZE: Right is open! Turn RIGHT.");
          turnToAngle(-90.0, TURN_R);
      }
      else if (!left_blocked) {
          LOG("MAZE: Left is open! Turn LEFT.");
          turnToAngle(90.0, TURN_L);
      }
      else {
          LOG("MAZE: Dead End! 180 U-TURN.");
          turnToAngle(180.0, TURN_R);
      }
      
      // --- POST-TURN AUTO ALIGNMENT (Absolute Peak Radar Sweep) ---
      LOG("ALIGN: Radar Sweep searching for absolute peak...");
      int sweep_count = 0;
      char search_dir = 'L'; // Arbitrarily guess left first
      bool reversed_once = false;
      
      sensorFront.rangingTest(&measureFront, false);
      int last_f = (measureFront.RangeStatus != 0) ? 8190 : measureFront.RangeMilliMeter;
      
      while (sweep_count < 20) { 
          // Twitch in search direction
          sendMotorCmd(search_dir, TURN_L, TURN_R);
          delay(60); 
          sendMotorCmd('S', 0, 0);
          delay(150); // wait for chassis to stop rocking
          
          sensorFront.rangingTest(&measureFront, false);
          int new_f = (measureFront.RangeStatus != 0) ? 8190 : measureFront.RangeMilliMeter;
          
          LOG("  [Sweep " + String(sweep_count+1) + "] Twitched " + String(search_dir) + " -> New F:" + String(new_f));
          
          if (new_f < last_f) {
              if (sweep_count == 0 && !reversed_once) {
                  search_dir = (search_dir == 'L') ? 'R' : 'L';
                  reversed_once = true;
                  LOG("  ALIGN: Bad first guess. Reversing sweep.");
                  sendMotorCmd(search_dir, TURN_L, TURN_R);
                  delay(60); sendMotorCmd('S', 0, 0); delay(150);
              } 
              else {
                  char undo_dir = (search_dir == 'L') ? 'R' : 'L';
                  sendMotorCmd(undo_dir, TURN_L, TURN_R);
                  delay(60); sendMotorCmd('S', 0, 0); delay(150);
                  LOG("ALIGN: Peak found & Locked! [F:" + String(last_f) + "]");
                  break;
              }
          }
          else if (new_f == last_f && new_f == 8190) {
              LOG("ALIGN: Max range limit reached! Locked at [F:8190]");
              break;
          }
          else {
              last_f = new_f;
          }
          sweep_count++;
      }
      
      if (sweep_count >= 20) {
          LOG("ALIGN: Sweep limit reached. Resuming.");
      }

      // RESET GYRO PID MEMORY SO IT DOESN'T SWERVE AFTER THE TURN!
      // The new hallway is our new absolute 0.0 heading!
      yaw = 0.0;
      headingIntegral = 0;
      lastHeadingError = 0;
      lastGyroTime = micros(); // Must reset timer before loop starts again!
      power_ramp = 30; // Trigger non-blocking soft start

      delay(1500); // Stop for 1.5 seconds so the user can admire the perfectly aligned car!
  } 
  else {
      // Path is clear! GYROSCOPE HEADING-HOLD PID
      // We want to target a perfect 0.0 degree heading straight down the hall.
      int correction = computeHeadingCorrection(0.0);
      
      // 2. Apply Correction to Calibrated Base Speeds
      int new_left = BASE_LEFT - correction;
      int new_right = BASE_RIGHT + correction;
      
      // 3. Clamp Speeds (0 to 255) Since ATmega UART timeout handles 0!
      if (new_left > 255) new_left = 255;
      if (new_left < 0) new_left = 0;
      if (new_right > 255) new_right = 255;
      if (new_right < 0) new_right = 0;
      
      // --- DYNAMIC NON-BLOCKING SOFT START ---
      // This allows the PID to aggressively steer the car *WHILE* ramping up power!
      if (power_ramp < 100) {
          power_ramp += 5; // Ramp up over ~14 cycles instead of 7 to prevent I2C voltage crash!
      }
      
      int final_left = (new_left * power_ramp) / 100;
      int final_right = (new_right * power_ramp) / 100;
      
      // 4. Send Dynamic PWM Command over UART
      sendMotorCmd('P', final_left, final_right);
      
      // Determine Steering Direction for Logs
      String steer_dir = "(Straight)";
      if (correction < -5) steer_dir = "(Steering Right)";
      else if (correction > 5) steer_dir = "(Steering Left)";
      
      // Print PID telemetry + all 3 sensor distances for full visibility!
      LOG("GYRO PID -> Yaw:" + String(yaw, 2) +
          " Adj:" + String(correction) + " " + steer_dir +
          " [L:" + String(final_left) + " R:" + String(final_right) + "]"
          " [F:" + String(front_dist) +
          " L:" + String(last_left_dist) +
          " R:" + String(last_right_dist) + "]");
  }

  delay(50); // Increased delay to 50ms to prevent ToF I2C lockup
}