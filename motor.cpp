#define F_CPU 1000000UL

#include <avr/io.h>


// ==================================================
// MOTOR INITIALIZATION
// ==================================================

void init_motor_pins()
{
    /*
        Direction pins:

        PB0 -> IN1
        PB1 -> IN2
        PB2 -> IN3
        PB4 -> IN4
    */

    DDRB |= (1 << PB0) |
            (1 << PB1) |
            (1 << PB2) |
            (1 << PB4);


    /*
        PWM pins:

        PD5 -> OC1A -> ENA -> Left motor
        PD4 -> OC1B -> ENB -> Right motor
    */

    DDRD |= (1 << PD5) |
            (1 << PD4);


    // Initially stopped

    PORTB &= ~(
        (1 << PB0) |
        (1 << PB1) |
        (1 << PB2) |
        (1 << PB4)
    );
}


// ==================================================
// PWM
// ==================================================

void init_pwm()
{
    /*
        Timer1
        Fast PWM 8-bit

        OC1A -> left motor
        OC1B -> right motor
    */

    TCCR1A =
        (1 << WGM10) |
        (1 << COM1A1) |
        (1 << COM1B1);

    TCCR1B =
        (1 << WGM12) |
        (1 << CS11) |
        (1 << CS10);


    // Motors initially disabled

    OCR1A = 0;
    OCR1B = 0;
}


// ==================================================
// MOVE FORWARD
// ==================================================

void move_forward()
{
    /*
        LEFT MOTOR

        IN1 = HIGH
        IN2 = LOW
    */

    PORTB |=  (1 << PB0);
    PORTB &= ~(1 << PB1);


    /*
        RIGHT MOTOR

        IN3 = HIGH
        IN4 = LOW
    */

    PORTB |=  (1 << PB2);
    PORTB &= ~(1 << PB4);


    /*
        Your previous straight-line
        speed correction
    */

    OCR1A = 210;    // Left
    OCR1B = 255;    // Right
}


// ==================================================
// STOP
// ==================================================

void stop_motor()
{
    // Disable motor direction signals

    PORTB &= ~(
        (1 << PB0) |
        (1 << PB1) |
        (1 << PB2) |
        (1 << PB4)
    );


    // PWM = zero

    OCR1A = 0;
    OCR1B = 0;
}


// ==================================================
// UART INITIALIZATION
// ==================================================

void uart_init()
{
    /*
        ATmega32 clock = 1 MHz
        Baud = 9600

        Use U2X double-speed mode.
    */

    UCSRA = (1 << U2X);


    /*
        Baud rate register

        1 MHz
        9600 baud
        U2X = 1

        UBRR = 12
    */

    UBRRH = 0;
    UBRRL = 12;


    /*
        Enable UART receiver.

        We don't need transmitter yet.
    */

    UCSRB = (1 << RXEN);


    /*
        8 data bits
        1 stop bit
        no parity

        UART format = 8N1
    */

    UCSRC =
        (1 << URSEL) |
        (1 << UCSZ1) |
        (1 << UCSZ0);
}


// ==================================================
// RECEIVE ONE CHARACTER
// ==================================================

char uart_receive()
{
    // Wait until data arrives

    while (!(UCSRA & (1 << RXC)))
    {
    }


    // Return received byte

    return UDR;
}


// ==================================================
// MAIN
// ==================================================

int main(void)
{
    init_motor_pins();

    init_pwm();

    uart_init();


    // Safety:
    // car remains stopped until ESP32 says RUN.

    stop_motor();


    while (1)
    {
        /*
            Wait for one command
            from ESP32
        */

        char command = uart_receive();


        // -------------------------
        // RUN
        // -------------------------

        if (command == 'R')
        {
            move_forward();
        }


        // -------------------------
        // STOP
        // -------------------------

        else if (command == 'S')
        {
            stop_motor();
        }
    }


    return 0;
}