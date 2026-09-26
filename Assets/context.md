# Context: Automated Maze Solving Robot Project

I am building an autonomous maze-solving robot. I need your help with code and hardware logic. Please read the following context carefully to understand the project architecture, equipment, and current challenges.

## 1. Project Goal
The robot must navigate a physical maze (constructed with books/walls, ~360mm hallway width) using the "Right-Hand Rule" algorithm. It must maintain a perfectly straight line down the hallways using a PID controller, detect intersections, and make precise 90-degree or 180-degree turns.

## 2. Hardware & Equipment
*   **Chassis:** Standard 2WD acrylic chassis. The front caster wheel is fixed (tied straight) to prevent erratic swiveling.
*   **Motors:** 2x standard 6V Yellow TT DC Gear Motors.
*   **Motor Driver:** L298N (driven by hardware PWM).
*   **Power:** 2x 3.7V 18650 Lithium-Ion batteries in series (7.4V nominal).
*   **Sensors (Current):** 3x VL53L0X Time-of-Flight (ToF) distance sensors (Front, Left, Right). They share a single I2C bus and are dynamically re-addressed at boot using their XSHUT pins.
*   **Sensors (Planned Upgrade):** MPU6050 Gyroscope (I2C) to implement a Heading-Hold PID.
*   **Master Brain (ESP32):** Handles sensor reading, PID math, maze logic (Right-Hand Rule), and hosts a Bluetooth Serial menu for live tuning. It calculates the required motor speeds and sends them over UART to the Slave.
*   **Slave Motor Controller (ATmega32):** Dedicated to generating clean, non-blocking hardware PWM. It receives UART commands from the ESP32 and directly controls the L298N H-Bridge.

## 3. Software Architecture & Code Structure
The system is divided into two microcontrollers to prevent the ESP32's WiFi/Bluetooth stack from interrupting the hardware PWM signals.

**A. ATmega32 (Slave) Code:**
*   Written in pure C/C++ using AVR registers. 
*   Runs a robust UART state machine with a 10ms timeout-recovery mechanism to prevent packet desynchronization.
*   Expects 3-byte packets from the ESP32: 
    *   `[Command Char] [Left PWM Byte] [Right PWM Byte]`
    *   Commands include: `'P'` (PID Forward), `'L'` (Turn Left), `'R'` (Turn Right), `'S'` (Stop).
*   It applies the PWM bytes to `OCR1A` and `OCR1B` (Timer 1) and sets the H-Bridge direction pins on `PORTB`.

**B. ESP32 (Master) Code:**
*   Written in Arduino C++.
*   `setup()`: Initializes the 3 ToF sensors, re-addresses them to `0x30`, `0x31`, `0x32`, and opens a Bluetooth Serial menu waiting for a `START` command.
*   `loop()`: 
    1. Parses live Bluetooth commands to update calibration variables on the fly: `BL` (Base Left PWM), `BR` (Base Right PWM), `XL` (Turn Left Power), `XR` (Turn Right Power), and `Kp`/`Kd`/`Ki`.
    2. Reads the ToF sensors.
    3. If `Front_Distance < Threshold`, it executes the Maze Logic (stops, reads sides, executes 90-degree Right/Left turn or 180 U-Turn, then does a "Radar Sweep" micro-twitch to perfectly align to the new hallway).
    4. If the path is clear, it calculates Positional PID using `Error = (Left_Distance - Right_Distance) / 2`. 
    5. It dynamically applies a Soft-Start power ramp to prevent voltage drops, calculates the final `final_left` and `final_right` PWM, and sends them via `Serial2` UART to the ATmega32.

## 4. Current Problems & Challenges
1.  **Extreme Motor Variance:** The cheap TT motors are highly unmatched. To drive straight, the Left motor requires a Base PWM of ~200-255, while the Right motor only requires ~10-30. 
2.  **Static Friction Dead-Zones:** Because the Right motor operates around 10-30 PWM, it sits right on the boundary of static friction. A PID adjustment of just `±5` causes it to toggle violently between completely stalled and wildly overpowering the Left motor. This makes Positional PID tuning incredibly difficult.
3.  **Turning vs. Straight Discrepancy:** The PWM required to overcome sideways friction during a 90-degree turn is completely different from driving straight. We solved this by creating separate `XL` and `XR` turn-power variables, which the ESP32 sends to the ATmega specifically for turning.
4.  **Initial Drift:** Because ToF sensors require a fraction of a second to read, and require perfectly parallel walls to calculate a valid Positional Error, the robot often drifts violently upon starting or immediately after exiting a turn before the PID can lock on.

## 5. Next Steps
Due to the extreme hardware variance of the TT motors causing the Positional PID to fail upon startup, we are mounting an **MPU6050 Gyroscope**. 
The goal is to write a **Heading-Hold PID** loop. Instead of relying on the side ToF sensors to drive straight, the ESP32 will read the Z-axis (Yaw) from the MPU6050. The moment the robot drifts from `Yaw = 0`, the Gyro PID will instantly compensate the motor speeds. The ToF sensors will then only be used to detect walls and intersections.

---
*End of Context. Please acknowledge you understand the architecture, the hardware limitations, and the upcoming MPU6050 integration goal.*
