Controls
===================================
1. ATmega32
2. ESP32 DevKit V1 (Coprocessor)
3. L298N Motor Driver
4. Logic Level Shifter (5V↔3.3V) (For ATmega32–ESP32 comms)   

Body & Movement
====================================
5. 2WD Robot Chassis Kit
6. IR Slotted Opto-Sensor Modules (x2) (Reads encoder discs) (Not Acquired)


Senses
====================================
7. VL53L0X ToF Sensors (x3) 
8. TCA9548A I2C Multiplexer (All three of your VL53L0X sensors have the exact same hardware "name" (I2C address 0x29). If you connect them all to the ESP32, they will all scream at the same time and crash. The Multiplexer is like a traffic cop. The ESP32 talks to the Multiplexer, and says "Only let me talk to Sensor #1 right now." Placement: It goes between the ESP32 and the sensors. You can just stick it to the chassis right next to the ESP32)



Power System
====================================
9. 18650 Li-ion Batteries (x2)
10. 18650 Battery Holder & Charger

Wiring & Hardware
====================================
28. 13. Jumper Wires (M-M, M-F, F-F) (Avoid big loose wires; use pre-formed solid core wires or solder direct to perfboard)
29. 14. Perfboard (Crucial: Replaces breadboards to save space and secure the ESP32/ATmega32)
30. 15. Spacers / Standoffs (M3) (For mounting sensors)
31. 16. Toggle Switch
32. 17. Soldering Iron, Lead & Rosin
33. 18. Double-Sided Foam Tape (For mounting L298N, ESP32, Battery Holder. NEVER use gum on metallic pins!)