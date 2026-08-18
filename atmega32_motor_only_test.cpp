#define F_CPU 1000000UL // Assumes factory default 1MHz internal clock
#include <avr/io.h>
#include <util/delay.h>

void init_motor_pins() {
    // Set PD4 (IN1), PD5 (IN2), and PD7 (ENA) as output pins
    DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD7);
}

int main(void) {
    init_motor_pins();

    while (1) {
        // --- SPIN MOTOR ---
        PORTD |= (1 << PD7);    // ENA = HIGH (Enable motor at max speed)
        PORTD |= (1 << PD4);    // IN1 = HIGH (Direction pin 1)
        PORTD &= ~(1 << PD5);   // IN2 = LOW  (Direction pin 2)
        
        _delay_ms(2000);        // Let it spin for 2 seconds
        
        // --- STOP MOTOR ---
        PORTD &= ~(1 << PD7);   // ENA = LOW (Disable motor)
        PORTD &= ~(1 << PD4);   // IN1 = LOW
        PORTD &= ~(1 << PD5);   // IN2 = LOW
        
        _delay_ms(2000);        // Keep it stopped for 2 seconds
    }
    return 0;
}
