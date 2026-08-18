# Breadboard Integration Test: System "Hello World"

This guide contains the exact temporary wiring and starter code needed to validate the core Master-Slave communication chain. This test strips away sensors and encoders to isolate the UART and motor driving logic.

## 1. Minimal Temporary Wiring

> [!CAUTION]
> Ensure the power is OFF (battery disconnected or toggle switch off) while making these connections.

### The Bare ATmega32 Setup (No Crystal Required)
Since you don't have a crystal oscillator or capacitors, **you must use the ATmega32's internal 1MHz oscillator.** By factory default, a new ATmega32 chip ships with the 1MHz internal clock enabled, so you don't need to change any fuses! The code provided below is already set to `1000000UL` (1MHz) to match this.

You still need one external component:
*   1x **10kΩ Resistor** (If you don't have this, you can *temporarily* bridge Pin 9 to 5V with a wire, but a resistor is much safer to prevent shorting during programming).

**Breadboard Wiring for the Bare Chip & USBasp:**
1.  **Power & Ground:** Connect ATmega32 Pin 10 (`VCC`) and Pin 30 (`AVCC`) to the breadboard's positive (red) rail. Connect Pin 11 (`GND`) and Pin 31 (`GND`) to the negative (blue) rail.
2.  **Reset Pull-up:** Connect the 10kΩ resistor from Pin 9 (`RESET`) to the 5V rail.
3.  **USBasp Connections (ISP Programming):**
    *   USBasp `VCC` $\rightarrow$ Breadboard 5V rail (This powers the ATmega32 from your laptop!).
    *   USBasp `GND` $\rightarrow$ Breadboard Ground rail.
    *   USBasp `MOSI` $\rightarrow$ ATmega32 Pin 6 (`PB5`).
    *   USBasp `MISO` $\rightarrow$ ATmega32 Pin 7 (`PB6`).
    *   USBasp `SCK` $\rightarrow$ ATmega32 Pin 8 (`PB7`).
    *   USBasp `RESET` $\rightarrow$ ATmega32 Pin 9 (`RESET`).

### Laptop Power vs. Battery Power (CRITICAL WARNING)
> [!WARNING]
> **Can your laptop power the breadboard?**
> **YES** for the ATmega32 and ESP32. The USBasp's `VCC` pin supplies 5V directly from your laptop. This is completely safe and sufficient for flashing code and testing logic.
> **ABSOLUTELY NOT** for the DC Motor. A motor draws massive current spikes that will instantly brown-out your microcontrollers, crash the USBasp, and potentially permanently damage your laptop's USB port.

**To run the full test safely:**
1. Keep the USBasp connected to power the ATmega32/ESP32 logic.
2. **L298N Power:** Connect your 18650 Battery (+) to the L298N `12V` terminal. Connect Battery (-) to the L298N `GND` terminal.
3. **Shared Ground:** Connect the L298N `GND` to your breadboard's Ground rail. This links the laptop's ground with the battery's ground (which is safe and required). 
*(Do NOT connect the L298N's 5V output to the breadboard while the USBasp is also supplying 5V. Let the laptop handle the 5V logic for this test).*

### Motor Control
1. **L298N to Motor:** Connect one yellow DC motor to L298N `OUT1` and `OUT2`.
2. **ATmega32 to L298N Logic:**
   - Connect ATmega32 Pin 18 (`PD4`) to L298N `IN1`.
   - Connect ATmega32 Pin 19 (`PD5`) to L298N `IN2`.
   - Connect ATmega32 Pin 21 (`PD7`, PWM Pin) to L298N `ENA` (remove the jumper cap if present).

### The UART Bridge (Master-Slave Link)
> [!IMPORTANT]
> The ATmega32 uses 5V logic, and the ESP32 uses 3.3V logic. If you do not have a Logic Level Shifter, you **must** build a voltage divider using resistors to step down the 5V from the ATmega32, otherwise you will permanently damage the ESP32's RX pin!

**1. ESP32 to ATmega32 (3.3V to 5V)**
*   Connect ESP32 Pin 17 (`TX2`) directly to ATmega32 Pin 14 (`RXD`). 
*(This is usually safe because the ATmega32 will register 3.3V as a logical HIGH).*

**2. ATmega32 to ESP32 (5V to 3.3V) - The Voltage Divider**
*   Connect a **10kΩ resistor** to ATmega32 Pin 15 (`TXD`).
*   Connect the other end of that 10kΩ resistor to ESP32 Pin 16 (`RX2`).
*   Connect a **20kΩ resistor** (or two 10kΩ resistors in series) from ESP32 Pin 16 (`RX2`) directly to Ground.
*(This resistor network drops the ATmega's 5V signal down to a safe ~3.3V for the ESP32).*

---

## 2. ESP32 Code (The Brain)

Upload this C++ code to the ESP32 using the Arduino IDE. It uses `Serial2` for hardware UART to talk to the ATmega32.

```cpp
// ESP32 Dummy Logic - Sends a move command every 10 seconds

#define RXp2 16
#define TXp2 17

void setup() {
  // Serial for PC Monitor debugging
  Serial.begin(115200);
  
  // Serial2 for talking to ATmega32 over UART
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
  
  Serial.println("ESP32 Brain Initialized. Waiting 10 seconds...");
  delay(10000);
}

void loop() {
  Serial.println("Commanding ATmega32: MOVE FORWARD 5 SECONDS");
  
  // Send the character 'F' to the ATmega32
  Serial2.print("F");
  
  // Wait 15 seconds (5s for movement to finish + 10s cooldown) before sending again
  delay(15000); 
}
```

---

## 3. ATmega32 Code (The Muscle)

Compile and flash this AVR C code to the ATmega32. This code continuously listens to the UART RX buffer. When it receives the 'F' command, it triggers the motor.

```c
#define F_CPU 1000000UL // Adjust based on your ATmega32 clock fuse setting
#include <avr/io.h>
#include <util/delay.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void uart_init() {
    UCSRB |= (1 << RXEN) | (1 << TXEN); // Turn on transmission and reception
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
    UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate
    UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits
}

unsigned char uart_receive() {
    while ((UCSRA & (1 << RXC)) == 0); // Wait until data is received
    return UDR; // Fetch received data from the buffer
}

void init_motor_pins() {
    // Set PD4, PD5, PD7 as output pins for L298N
    DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD7);
}

void motor_forward_5s() {
    // Enable Motor A at full speed (ENA = HIGH)
    PORTD |= (1 << PD7);
    
    // Set Direction (IN1 = HIGH, IN2 = LOW)
    PORTD |= (1 << PD4);
    PORTD &= ~(1 << PD5);
    
    _delay_ms(5000); // Run for 5 seconds
    
    // Stop Motor (IN1 = LOW, IN2 = LOW, ENA = LOW)
    PORTD &= ~(1 << PD4);
    PORTD &= ~(1 << PD5);
    PORTD &= ~(1 << PD7);
}

int main(void) {
    uart_init();
    init_motor_pins();

    while (1) {
        // Wait for command from ESP32
        unsigned char cmd = uart_receive();
        
        if (cmd == 'F') {
            motor_forward_5s();
        }
    }
    return 0;
}
```

## Running the Test
1. Upload the code to both boards.
2. Turn on the main battery power switch.
3. Observe: The motor should remain completely still upon boot. After ~10 seconds, the motor should spin at maximum speed for exactly 5 seconds, then stop. 
4. If this happens, your Master-Slave architecture works perfectly. You are ready to start building the physical chassis!
