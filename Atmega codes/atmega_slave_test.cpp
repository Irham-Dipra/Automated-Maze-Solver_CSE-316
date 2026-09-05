#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)

void uart_init() {
    UCSRA |= (1 << U2X); // Double speed mode for accuracy at 1MHz
    UCSRB |= (1 << RXEN) | (1 << TXEN);
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
    UBRRL = BAUD_PRESCALE;
    UBRRH = (BAUD_PRESCALE >> 8);
}

unsigned char uart_receive() {
    // If no new data has arrived, return 0
    if ((UCSRA & (1 << RXC)) == 0) {
        return 0;
    }
    return UDR; // Fetch the received data byte
}

void init_motor_pins() {
    // Direction pins: PB0, PB1, PB2, PB4
    DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB4);
    
    // PWM speed pins: PD5 (Left), PD4 (Right)
    DDRD |= (1<<PD5) | (1<<PD4);
    
    // Initially force direction to STOP
    PORTB &= ~((1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB4));
}

void init_pwm() {
    // Setup Timer 1 for 8-bit Fast PWM
    TCCR1A |= (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1);
    TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10);
}

void move_forward() {
    // Left motor forward
    PORTB |= (1<<PB0);
    PORTB &= ~(1<<PB1);

    // Right motor forward
    PORTB |= (1<<PB2);
    PORTB &= ~(1<<PB4);

    // Set Max Speed (PWM)
    OCR1A = 255; 
    OCR1B = 255;
}

void stop_motors() {
    // Set Speed to Zero (PWM)
    OCR1A = 0; 
    OCR1B = 0;
}

int main(void) {
    uart_init();
    init_motor_pins();
    init_pwm();
    
    unsigned char last_cmd = 0;

    while(1) {
        unsigned char cmd = uart_receive();
        
        if (cmd == 'F' && last_cmd != 'F') {
            move_forward();
            last_cmd = 'F';
        } 
        else if (cmd == 'S' && last_cmd != 'S') {
            stop_motors();
            last_cmd = 'S';
        }
    }
    
    return 0;
}
