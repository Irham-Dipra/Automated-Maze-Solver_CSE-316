# Connections

## Powers

*   Battery holder (15V) positive - Motor drive 12V
*   Battery holder (15V) negative - Breadboard ground 2


*   Motor driver GND - Breadboard ground 2
*   Motor Driver 5V - Breadboard power 1

*   Power 1 - Power 2
*   Gnd 1 - Gnd 2

## Motor Driver Connections

*   Out1 - Left Motor top
*   Out2 - Left Motor bottom
*   Out3 - Right Motor top
*   Out4 - Right Motor bottom

*   IN1 - Atmega PB0 (1)
*   IN2 - Atmega PB1 (2)
*   IN3 - Atmega PB2 (3)
*   IN4 - Atmega PB4 (5)

*   ENA - Atmega PD5 (19)
*   ENB - Atmega PD4 (18)


## Atmega connections

*   VCC (Pin 10) - power 1
*   GND (Pin 11) - Gnd 1

*   AVCC (Pin 30) - power 2
*   GND  (Pin 31) - Gnd 2

## ESP

*   VIN - Power 2
*   ESP Gnd 1 - Gnd 1

## Logic Level Shifter (UART Bridge)

*   HV - Power 1 (ATmega 5V)
*   LV - ESP32 3V3 Pin
*   GND (both) - Gnd 1 / Gnd 2
*   LV1 - ESP32 TX2 (Pin 24)
*   HV1 - ATmega RXD (PD0 / Pin 14)
