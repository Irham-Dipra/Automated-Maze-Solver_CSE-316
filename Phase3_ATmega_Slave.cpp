#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)

// ----- MOTOR SPEED CALIBRATION -----
// Adjust these if the car pulls to one side!
#define SPEED_LEFT 120   // 0 to 255
#define SPEED_RIGHT 150  // 0 to 255

void uart_init() {
    UCSRA |= (1 << U2X); 
    UCSRB |= (1 << RXEN) | (1 << TXEN);
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
    UBRRL = BAUD_PRESCALE;
    UBRRH = (BAUD_PRESCALE >> 8);
}

unsigned char uart_receive() {
    if ((UCSRA & (1 << RXC)) == 0) return 0;
    return UDR;
}

void init_motor_pins() {
    DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB4);
    DDRD |= (1<<PD5) | (1<<PD4);
    PORTB &= ~((1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB4));
}

void init_pwm() {
    TCCR1A |= (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1);
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

    unsigned char last_cmd = 'S';

    while(1) {
        unsigned char cmd = uart_receive();
        
        // Only change state if we receive a new command
        if (cmd != 0 && cmd != last_cmd) {
            if (cmd == 'F') move_forward();
            else if (cmd == 'B') move_backward();
            else if (cmd == 'L') turn_left();
            else if (cmd == 'R') turn_right();
            else if (cmd == 'S') stop_motors();
            
            last_cmd = cmd;
        }
    }
    return 0;
}
