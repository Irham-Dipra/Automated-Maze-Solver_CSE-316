# Automated Maze Solver - Detailed Project Execution Plan

This document serves as the comprehensive, step-by-step guide to building, programming, and testing the autonomous maze-solving robot. It integrates the master-slave architecture, exact wiring instructions using the I2C multiplexer, and an accelerated 5-week schedule to ensure ample time for full system testing.

---

## 1. Project Overview & System Architecture

The robot operates on a **Master-Slave** architecture to distribute the computational load and overcome the memory limitations of the ATmega32.

*   **ATmega32 (Master / Movement):** The primary real-time motion controller. It handles hardware interrupts (wheel encoders), executes continuous PID algorithms for straight driving/turning, and translates high-level commands into motor PWM signals.
*   **ESP32 DevKit V1 (Slave / Logic & AI):** The "brain" of the system. It processes the sensor array via I2C, runs the Flood Fill algorithm to map the 256-cell maze, and sends high-level directional commands (e.g., "Forward", "Turn Left") to the ATmega32 over UART.

### Team Responsibilities
*   **Irham Dipra (2205068):** [Insert specific role, e.g., ESP32 Logic & AI integration]
*   **Priyanjan Das Anabil (2205079):** [Insert specific role, e.g., ATmega32 Hardware & PID Tuning]
*   **Aroni Ananya (2205087):** [Insert specific role, e.g., Maze Construction & System Testing]

---

## 2. Resource Checklist & Procurement Phase

Before starting the build, immediately procure the "Not Acquired" items. This is your **Day 1 priority**.

**Currently Acquired:**
*   [x] ATmega32 Dev Board
*   [x] ESP32 DevKit V1
*   [x] L298N Motor Driver
*   [x] 2WD Robot Chassis Kit & DC Motors
*   [x] 3x VL53L0X ToF Sensors
*   [x] TCA9548A I2C Multiplexer
*   [x] 18650 Li-ion Batteries (x2) & Charger
*   [x] Jumper Wires

**Must Acquire Immediately:**
*   [ ] **Logic Level Shifter (5V↔3.3V):** *CRITICAL* for safe UART communication between ATmega32 (5V) and ESP32 (3.3V).
*   [ ] **IR Slotted Opto-Sensor Modules (x2):** *CRITICAL* for reading the encoder discs for PID control.
*   [ ] **Perfboard:** To mount the logic components permanently (replaces breadboards).
*   [ ] **Spacers / Standoffs (M3):** To mount the ToF sensors to the front of the acrylic chassis.
*   [ ] **Double-Sided Foam Tape:** For securely mounting the L298N, ESP32, and Battery Holder to the chassis.
*   [ ] **Toggle Switch:** To easily cut battery power.
*   [ ] **Soldering Iron, Lead & Rosin:** For permanent connections on the perfboard.

---

## 3. Step-by-Step Hardware Build Guide (The "Foolproof" Wiring)

Follow this sequence exactly to avoid burning out components or facing impossible cable management.

### Phase 1: Mechanics (Bottom Deck)
1.  Mount the two DC motors to the bottom of the acrylic base.
2.  Push the black slotted encoder discs onto the inner shafts of the DC motors.
3.  Mount the front caster wheel.
4.  Mount the 18650 battery holder at the rear (between the motors) using double-sided foam tape.
5.  Mount the L298N motor driver in the middle/front of the bottom deck (behind the caster wheel) using double-sided foam tape.
6.  Connect the Left Motor to `OUT1` and `OUT2` on the L298N.
7.  Connect the Right Motor to `OUT3` and `OUT4` on the L298N.

### Phase 2: Power Distribution & Logic (Top Deck)
1.  Wire the battery positive terminal through the toggle switch.
2.  **L298N Power:** Wire the switch output down to the `12V` terminal of the L298N (on the bottom deck).
4.  **Grounding (CRITICAL):** Wire the battery negative terminal to the `GND` of the L298N. **All components in this robot must share a common ground line.**
5.  **Logic Hub:** Solder female headers to your Perfboard for the ATmega32 and ESP32. Mount the Perfboard to the center of the top deck. (Let the USBasp dangle off the side via jumper wires).

### Phase 3: ATmega32 (Master) Integration
1.  **Motor Direction:** Connect ATmega32 standard GPIO pins to `IN1`, `IN2`, `IN3`, `IN4` on the L298N.
2.  **Motor Speed (PWM):** Connect two ATmega32 PWM pins to `ENA` and `ENB` on the L298N.
3.  **Encoders:** Mount the IR Slotted Opto-Sensors so the encoder discs pass through them. Connect their signal pins to ATmega32 `INT0` and `INT1` (External Interrupt pins).

### Phase 4: ESP32 (Slave) & Sensors Integration
Since we are using the **TCA9548A I2C Multiplexer**, we do not need to wire XSHUT pins.
1.  **Multiplexer Power:** Connect TCA9548A `VIN` to 3.3V and `GND` to common ground.
2.  **ESP32 to Multiplexer:** Connect ESP32 `SDA` to TCA9548A `SDA`, and ESP32 `SCL` to TCA9548A `SCL`.
3.  **Multiplexer to Sensors:** 
    *   Front ToF Sensor: Connect `SDA`/`SCL` to Multiplexer Channel 0 (`SD0`/`SC0`).
    *   Left ToF Sensor: Connect `SDA`/`SCL` to Multiplexer Channel 1 (`SD1`/`SC1`).
    *   Right ToF Sensor: Connect `SDA`/`SCL` to Multiplexer Channel 2 (`SD2`/`SC2`).
4.  **Sensor Power:** Connect all ToF `VIN` to 3.3V and `GND` to common ground.

### Phase 5: The UART Bridge
1.  Connect the **Logic Level Shifter** between the ATmega32 (5V side) and ESP32 (3.3V side).
2.  Connect ATmega32 `TX` $\rightarrow$ Shifter $\rightarrow$ ESP32 `RX`.
3.  Connect ATmega32 `RX` $\rightarrow$ Shifter $\rightarrow$ ESP32 `TX`.

---

## 4. Physical Maze Construction

Do not wait until Week 4 to think about the maze. You need it for testing.
*   **Dimensions:** $16 \times 16$ grid. If a full maze is too large for your home, build an $8 \times 8$ or $4 \times 4$ for testing.
*   **Materials:** Use a large plywood board painted matte black or white.
*   **Walls:** Use high-density foam board or stiff cardboard, cut into uniform strips. Use double-sided tape or hot glue to secure them.
*   **Cell Size:** Ensure the cell width is at least 3-4 cm wider than your robot's total width to allow for slight drift.

---

## 5. The Accelerated Schedule (Target: 3 Weeks Core Dev)

This schedule compresses the build and software logic into 3 weeks to guarantee 2 weeks for full physical testing and edge-case resolution.

### Week 1: Hardware Assembly & Basic Movement
*   **Day 1-2:** Procure all missing components. Cut and paint the base for a mini test maze ($4 \times 4$).
*   **Day 3-5:** Complete Wiring Phases 1, 2, and 3.
*   **Day 6-7:** Write basic ATmega32 code to spin motors forward, backward, left, right. Ensure L298N is functioning correctly.

### Week 2: PID Control, Sensors, & Algorithm Simulation
*   **Day 1-3 (ATmega32):** Read encoder interrupts. Write the PID loop. **Goal:** The robot must drive in a perfectly straight line for 1 meter ($E_{PID} \approx 0$) and execute exact $90^\circ$ turns on the floor.
*   **Day 1-3 (PC):** Write and test the Flood Fill algorithm in standard C++ on a computer terminal. Ensure it solves virtual mazes.
*   **Day 4-7 (ESP32):** Complete Wiring Phase 4. Program the ESP32 to switch multiplexer channels and read accurate distances from all three ToF sensors.

### Week 3: UART Bridge & System Integration (Project Update 1)
*   **Day 1-2:** Complete Wiring Phase 5 (Level Shifter). Establish UART communication. Send dummy commands from ESP32 to ATmega32 (e.g., send "F", robot moves Forward exactly 1 cell).
*   **Day 3-5:** Port the PC Flood Fill C++ code onto the ESP32. Merge the sensor readings into the algorithm to detect physical walls.
*   **Day 6-7:** Run the robot in the mini $4 \times 4$ test maze. **Goal:** It should autonomously solve the small grid.

### Week 4: Full Physical Maze & Extensive Testing
*   **Day 1-2:** Construct the full $16 \times 16$ (or $8 \times 8$ if space-constrained) physical maze.
*   **Day 3-7:** Run continuous full-scale tests. 
    *   *Bug hunting:* Does the robot drift and scrape walls? (Tune PID).
    *   *Bug hunting:* Does it register ghost walls? (Tune ToF sensor threshold distances).

### Week 5: Practice Runs, Stretch Goals, & Polish
*   **Day 1-4:** Focus on **Algorithmic Path Efficiency ($\eta_{path} = 1$)**. Ensure that once the center is found, it calculates the absolute shortest path.
*   *(Optional)* Implement the "Speed Run" (returning to start and executing the optimal path at high speed).
*   **Day 5-7:** Final presentation preparation. Record videos of successful runs in case of live demonstration failures.
