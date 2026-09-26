# ESP32 DevKit V1 - 30-Pin Physical Reference

Hold your ESP32 so that the **Micro-USB port is pointing DOWN** towards you. 

Here is the exact physical layout of the 30 pins (15 on the left, 15 on the right), going from Top to Bottom.

### LEFT SIDE (Top to Bottom)
1.  **EN** (Enable / Reset Button)
2.  **VP** (GPIO 36) - *[DANGER: Input Only]*
3.  **VN** (GPIO 39) - *[DANGER: Input Only]*
4.  **D34** (GPIO 34) - *[DANGER: Input Only]*
5.  **D35** (GPIO 35) - *[DANGER: Input Only]*
6.  **D32** (GPIO 32)
7.  **D33** (GPIO 33)
8.  **D25** (GPIO 25)
9.  **D26** (GPIO 26)
10. **D27** (GPIO 27)
11. **D14** (GPIO 14)
12. **D12** (GPIO 12) - *[DANGER: Strapping Pin]*
13. **D13** (GPIO 13)
14. **GND** (Ground)
15. **VIN** (5V Power IN)

---

### RIGHT SIDE (Top to Bottom)
16. **D23** (GPIO 23)
17. **D22** (GPIO 22) - **[USE FOR I2C SCL]**
18. **TX0** (GPIO 1) - *[Do not use, reserved for USB]*
19. **RX0** (GPIO 3) - *[Do not use, reserved for USB]*
20. **D21** (GPIO 21) - **[USE FOR I2C SDA]**
21. **D19** (GPIO 19)
22. **D18** (GPIO 18)
23. **D5** (GPIO 5) - *[DANGER: Strapping Pin]*
24. **TX2** (GPIO 17) - **[USE FOR UART TX]**
25. **RX2** (GPIO 16) - **[USE FOR UART RX]**
26. **D4** (GPIO 4)
27. **D2** (GPIO 2) - *[DANGER: Strapping Pin]*
28. **D15** (GPIO 15) - *[DANGER: Strapping Pin]*
29. **GND** (Ground)
30. **3V3** (3.3V Power OUT)

---

### 🔑 Quick Summary for Your Micromouse:
*   **Power:** Feed your battery's 5V to `VIN` (Left Pin 15), and Ground to `GND` (Left Pin 14).
*   **Sensors:** Use `3V3` (Right Pin 30) to power your Multiplexer/Sensors. Connect the Multiplexer to `D21` (Right Pin 20) and `D22` (Right Pin 17).
*   **ATmega32 Comms:** We will eventually use `RX2` (Right Pin 25) and `TX2` (Right Pin 24) to send commands to the ATmega32.
