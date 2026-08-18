#define F_CPU 1000000UL 
#include <avr/io.h>
#include <util/delay.h>

#define USART_BAUDRATE 9600
// Use 8UL because we are enabling U2X (Double Speed Mode) for better accuracy at 1MHz
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)

void uart_init() {
    UCSRA |= (1 << U2X); // Double transmission speed (Crucial for 1MHz clock stability)
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
    // Enable Motor at full speed (ENA = HIGH)
    PORTD |= (1 << PD7);
    
    // Set Direction Forward (IN1 = HIGH, IN2 = LOW)
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
        // Wait forever until the ESP32 sends a character
        unsigned char cmd = uart_receive();
        
        // If the ESP32 says 'F' (Forward), run the motor sequence
        if (cmd == 'F') {
            motor_forward_5s();
        }
    }
    return 0;
}
