#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)

// ----- MOTOR SPEED CALIBRATION -----
// Adjust these if the car pulls to one side!
#define SPEED_LEFT 145   // 0 to 255
#define SPEED_RIGHT 200  // 0 to 255

void uart_init() {
    UCSRA |= (1 << U2X); // double speed mode
    UCSRB |= (1 << RXEN) | (1 << TXEN); // enable receiver and transitter
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // talk to ucsrc, not ubrrh. also 8 bit data
    UBRRL = BAUD_PRESCALE; // load lower 8 bits of baud rate
    UBRRH = (BAUD_PRESCALE >> 8); // load higher 8 bits of baud rate
}

uint8_t uart_available() {
    return (UCSRA & (1 << RXC));
}

unsigned char uart_read() {
    return UDR;
}

void init_motor_pins() {
    DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB4);
    DDRD |= (1<<PD5) | (1<<PD4);
    PORTB &= ~((1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB4));
}

void init_pwm() {
    TCCR1A |= (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1); // 8 bit fast pwm mode, prescaler 64
    TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10);
}

void stop_motors() {
    OCR1A = 0; 
    OCR1B = 0;
}

void move_forward() {
    PORTB |= (1<<PB0);  PORTB &= ~(1<<PB1); // Left Forward
    PORTB |= (1<<PB2);  PORTB &= ~(1<<PB4); // Right Forward
    OCR1A = SPEED_LEFT; 
    OCR1B = SPEED_RIGHT;
}

void move_forward_pid(unsigned char left_pwm, unsigned char right_pwm) {
    PORTB |= (1<<PB0);  PORTB &= ~(1<<PB1); // Left Forward
    PORTB |= (1<<PB2);  PORTB &= ~(1<<PB4); // Right Forward
    OCR1A = left_pwm; 
    OCR1B = right_pwm;
}

void move_backward() {
    PORTB &= ~(1<<PB0); PORTB |= (1<<PB1); // Left Backward
    PORTB &= ~(1<<PB2); PORTB |= (1<<PB4); // Right Backward
    OCR1A = SPEED_LEFT; 
    OCR1B = SPEED_RIGHT;
}

void turn_left() {
    PORTB &= ~(1<<PB0); PORTB |= (1<<PB1); // Left Backward
    PORTB |= (1<<PB2);  PORTB &= ~(1<<PB4); // Right Forward
    OCR1A = SPEED_LEFT; 
    OCR1B = SPEED_RIGHT;
}

void turn_right() {
    PORTB |= (1<<PB0);  PORTB &= ~(1<<PB1); // Left Forward
    PORTB &= ~(1<<PB2); PORTB |= (1<<PB4); // Right Backward
    OCR1A = SPEED_LEFT; 
    OCR1B = SPEED_RIGHT;
}

int main(void) {
    uart_init();
    init_motor_pins();
    init_pwm();
    stop_motors();

    unsigned char rx_state = 0;
    unsigned char left_pwm = 0;

    while(1) {
        if (uart_available()) {
            unsigned char byte_in = uart_read();
            
            if (rx_state == 0) {
                if (byte_in == 'P') rx_state = 1;
                else if (byte_in == 'S') stop_motors();
                else if (byte_in == 'L') turn_left();
                else if (byte_in == 'R') turn_right();
                else if (byte_in == 'F') move_forward(); // Fallback
                else if (byte_in == 'B') move_backward(); // Fallback
            }
            else if (rx_state == 1) {
                left_pwm = byte_in;
                rx_state = 2;
            }
            else if (rx_state == 2) {
                unsigned char right_pwm = byte_in;
                move_forward_pid(left_pwm, right_pwm);
                rx_state = 0; // Reset state machine, ready for next command!
            }
        }
    }
    return 0;
}
