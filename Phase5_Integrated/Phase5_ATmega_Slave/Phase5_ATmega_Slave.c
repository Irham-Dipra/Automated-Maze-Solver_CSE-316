/* ===========================================================================
 *  Phase5_ATmega_Slave.c  --  hardened motor controller for the maze solver
 * ===========================================================================
 *
 *  Replaces Phase3_ATmega_Slave.cpp. Same wiring, same job (take commands
 *  from the ESP32 over UART, drive the L298N), but four concrete faults from
 *  the old firmware are fixed:
 *
 *  1. FRAMING.  The old protocol was a bare 3-byte stream [cmd][L][R] with no
 *     sync byte and no checksum. The command letters are 'P'=80, 'L'=76,
 *     'R'=82, 'S'=83, 'F'=70, 'B'=66 -- which sit right inside the PWM range
 *     the ESP32 actually sends. Lose ONE byte on a noisy 9600 line and the
 *     state machine resyncs onto a PWM byte that happens to equal 'R' or 'L',
 *     and the robot pivots at full power while the ESP32 believes it is
 *     driving straight. That is exactly the "car suddenly spins / violently
 *     steers" signature in bluetooth_logs/. Now every packet is
 *     [0xA5][cmd][L][R][chk] and a bad checksum is dropped, not executed.
 *
 *  2. FAILSAFE.  The old firmware held the last command forever. When the
 *     ESP32 brownout-reset or Bluetooth dropped mid-run (see
 *     serial_20260925_000240.txt: "Connection lost" while driving) the motors
 *     kept running. Now: no valid packet for CMD_TIMEOUT_MS -> motors stop.
 *     The ESP32 sends a heartbeat packet every tick to hold them on.
 *
 *  3. PWM FREQUENCY.  The old code used Fast PWM with prescaler 64. At the
 *     factory-default 1 MHz internal RC that is 1e6/(64*256) = 61 Hz. 61 Hz
 *     into a TT motor is the audible "high-pitched buzzing, motor refuses to
 *     spin" from the progress report: at low duty the motor gets 16 ms pulses
 *     that shake the gearbox instead of turning it. Now the prescaler is
 *     picked from F_CPU so the carrier lands near 250-500 Hz either way.
 *
 *  4. BLOCKING RX.  Bytes were polled in the main loop. A missed byte
 *     desynced the stream. RX is now interrupt-driven into a ring buffer.
 *
 *  ---------------------------------------------------------------------
 *  BUILD
 *    16 MHz crystal (STRONGLY recommended -- see note below):
 *      avr-gcc -mmcu=atmega32 -DF_CPU=16000000UL -Os -o slave.elf Phase5_ATmega_Slave.c
 *      avrdude ... -U lfuse:w:0xFF:m -U hfuse:w:0xC9:m     (ext crystal, BOD 4.0V)
 *    factory default 1 MHz internal RC (works, but read the warning):
 *      avr-gcc -mmcu=atmega32 -DF_CPU=1000000UL  -Os -o slave.elf Phase5_ATmega_Slave.c
 *
 *  WARNING about the 1 MHz internal RC oscillator: it is only factory-trimmed
 *  to +-3% at 5 V / 25 C and drifts further with supply voltage and
 *  temperature. UART needs the two ends within about +-2%. On battery, with
 *  the rail sagging under motor load, this is a real source of the corrupted
 *  packets described in fault 1. The checksum below turns that corruption
 *  into dropped packets (safe) instead of wrong moves (dangerous) -- but if
 *  you see the FAILSAFE stopping you constantly, the fix is the crystal.
 *
 *  WIRING (unchanged from your Assets/connections.md)
 *    IN1 -> PB0   IN2 -> PB1   IN3 -> PB2   IN4 -> PB4
 *    ENA -> PD5 (OC1A, LEFT)   ENB -> PD4 (OC1B, RIGHT)
 *    ESP32 TX2 -> level shifter -> PD0 (RXD)
 * ======================================================================== */

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 *  Protocol
 * ------------------------------------------------------------------------ */
#define SYNC_BYTE        0xA5
#define CHK_SALT         0x5A
#define USART_BAUDRATE   9600UL
#define BAUD_PRESCALE    (((F_CPU / (USART_BAUDRATE * 8UL))) - 1UL)

/* Motors stop if no valid packet arrives inside this window. The ESP32
 * heartbeats every 20 ms, so 300 ms tolerates ~15 dropped packets before
 * giving up -- long enough not to stutter, short enough that a crashed
 * master cannot drive the robot into a wall. */
#define CMD_TIMEOUT_MS   300

/* Commands. 'P' is accepted as an alias for 'F' so old ESP32 sketches that
 * still say "PID forward" keep working. */
#define CMD_FORWARD      'F'
#define CMD_FORWARD_PID  'P'
#define CMD_BACK         'B'
#define CMD_PIVOT_LEFT   'L'
#define CMD_PIVOT_RIGHT  'R'
#define CMD_STOP         'S'
#define CMD_BRAKE        'K'

/* --------------------------------------------------------------------------
 *  Millisecond timebase (Timer0, CTC)
 * ------------------------------------------------------------------------ */
#if   F_CPU == 16000000UL
  #define T0_PRESCALE_BITS ((1 << CS01) | (1 << CS00))   /* /64  -> 250 kHz */
  #define T0_TOP           249
#elif F_CPU == 8000000UL
  #define T0_PRESCALE_BITS ((1 << CS01) | (1 << CS00))   /* /64  -> 125 kHz */
  #define T0_TOP           124
#else  /* 1 MHz */
  #define T0_PRESCALE_BITS (1 << CS01)                   /* /8   -> 125 kHz */
  #define T0_TOP           124
#endif

static volatile uint32_t g_millis = 0;

ISR(TIMER0_COMP_vect) { g_millis++; }

static uint32_t millis_now(void) {
    uint32_t m;
    uint8_t s = SREG;
    cli();
    m = g_millis;
    SREG = s;
    return m;
}

static void timer0_init(void) {
    TCCR0 = (1 << WGM01) | T0_PRESCALE_BITS;   /* CTC */
    OCR0  = T0_TOP;
    TIMSK |= (1 << OCIE0);
}

/* --------------------------------------------------------------------------
 *  Motors
 * ------------------------------------------------------------------------ */
/* Timer1, 8-bit PHASE-CORRECT PWM (mode 1). Phase-correct rather than fast
 * PWM because it halves the carrier for a given prescaler, which lets us land
 * in the 250-500 Hz band the L298N + TT motor combination likes at either
 * clock speed. Prescaler is picked per clock so the carrier is comparable:
 *      16 MHz, /64 -> 16e6/(2*64*255) = 490 Hz
 *       1 MHz, /8  ->  1e6/(2*8*255)  = 245 Hz
 * The old firmware's 61 Hz is what made the motors buzz instead of turn. */
#if F_CPU >= 8000000UL
  #define T1_PRESCALE_BITS ((1 << CS11) | (1 << CS10))   /* /64 */
#else
  #define T1_PRESCALE_BITS (1 << CS11)                   /* /8  */
#endif

static void motors_init(void) {
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB4);
    DDRD |= (1 << PD5) | (1 << PD4);
    PORTB &= (uint8_t)~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB4));

    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = T1_PRESCALE_BITS;
    OCR1A  = 0;   /* PD5 / ENA / LEFT  */
    OCR1B  = 0;   /* PD4 / ENB / RIGHT */
}

static void dir_left_fwd(void)  { PORTB |=  (1 << PB0); PORTB &= (uint8_t)~(1 << PB1); }
static void dir_left_rev(void)  { PORTB &= (uint8_t)~(1 << PB0); PORTB |=  (1 << PB1); }
static void dir_left_off(void)  { PORTB &= (uint8_t)~((1 << PB0) | (1 << PB1)); }
static void dir_left_brk(void)  { PORTB |=  (1 << PB0) | (1 << PB1); }

static void dir_right_fwd(void) { PORTB |=  (1 << PB2); PORTB &= (uint8_t)~(1 << PB4); }
static void dir_right_rev(void) { PORTB &= (uint8_t)~(1 << PB2); PORTB |=  (1 << PB4); }
static void dir_right_off(void) { PORTB &= (uint8_t)~((1 << PB2) | (1 << PB4)); }
static void dir_right_brk(void) { PORTB |=  (1 << PB2) | (1 << PB4); }

static void motors_stop(void) {
    OCR1A = 0;
    OCR1B = 0;
    dir_left_off();
    dir_right_off();
}

/* Active brake: both H-bridge inputs high shorts the motor terminals, so the
 * back-EMF brakes the rotor instead of it coasting. Used at the end of a
 * pivot, where a 200 ms coast is several degrees of overshoot. */
static void motors_brake(void) {
    dir_left_brk();
    dir_right_brk();
    OCR1A = 255;
    OCR1B = 255;
}

static void apply_command(uint8_t cmd, uint8_t l, uint8_t r) {
    switch (cmd) {
        case CMD_FORWARD:
        case CMD_FORWARD_PID:
            dir_left_fwd();  dir_right_fwd();  break;
        case CMD_BACK:
            dir_left_rev();  dir_right_rev();  break;
        case CMD_PIVOT_LEFT:                       /* left wheel back, right fwd */
            dir_left_rev();  dir_right_fwd(); break;
        case CMD_PIVOT_RIGHT:                      /* left fwd, right wheel back */
            dir_left_fwd();  dir_right_rev(); break;
        case CMD_BRAKE:
            motors_brake();  return;
        case CMD_STOP:
        default:
            motors_stop();   return;
    }
    OCR1A = l;   /* LEFT  */
    OCR1B = r;   /* RIGHT */
}

/* --------------------------------------------------------------------------
 *  UART -- interrupt driven receive into a ring buffer
 * ------------------------------------------------------------------------ */
#define RX_BUF_SIZE 32                              /* must be a power of two */
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_head = 0, rx_tail = 0;

ISR(USART_RXC_vect) {
    uint8_t d = UDR;
    uint8_t next = (uint8_t)((rx_head + 1) & (RX_BUF_SIZE - 1));
    if (next != rx_tail) {            /* drop on overflow rather than corrupt */
        rx_buf[rx_head] = d;
        rx_head = next;
    }
}

static uint8_t uart_has_byte(void) { return (rx_head != rx_tail); }

static uint8_t uart_get_byte(void) {
    uint8_t d = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) & (RX_BUF_SIZE - 1));
    return d;
}

static void uart_init(void) {
    UCSRA |= (1 << U2X);
    UCSRB  = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
    UCSRC  = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);   /* 8N1 */
    UBRRH  = (uint8_t)(BAUD_PRESCALE >> 8);
    UBRRL  = (uint8_t)(BAUD_PRESCALE);
}

static void uart_put(uint8_t c) {
    while (!(UCSRA & (1 << UDRE))) { }
    UDR = c;
}

static void uart_puts(const char *s) {
    while (*s) uart_put((uint8_t)*s++);
}

/* --------------------------------------------------------------------------
 *  Packet parser
 *
 *  [0xA5][cmd][left][right][chk],  chk = cmd ^ left ^ right ^ 0x5A
 *
 *  On a checksum failure the whole packet is thrown away and the parser goes
 *  back to hunting for 0xA5. It never half-executes. A 0xA5 appearing inside
 *  a payload is harmless: that packet fails its checksum, is dropped, and the
 *  next good one (20 ms later) resyncs us.
 * ------------------------------------------------------------------------ */
int main(void) {
    uint8_t  state = 0;
    uint8_t  cmd = CMD_STOP, pl = 0, pr = 0;
    uint8_t  failsafe_latched = 1;          /* start stopped, not moving */
    uint32_t last_packet_ms;

    uart_init();
    timer0_init();
    motors_init();
    motors_stop();
    sei();

    last_packet_ms = millis_now();
    uart_puts("\r\nATmega32 slave ready (framed protocol, failsafe armed)\r\n");

    for (;;) {
        while (uart_has_byte()) {
            uint8_t b = uart_get_byte();

            switch (state) {
                case 0:
                    if (b == SYNC_BYTE) state = 1;
                    break;

                case 1:
                    /* Only accept a byte that is actually a command letter.
                     * Anything else means we locked onto a 0xA5 that was
                     * really payload -- go back to hunting immediately
                     * instead of consuming three more bytes. */
                    if (b == CMD_FORWARD || b == CMD_FORWARD_PID ||
                        b == CMD_BACK    || b == CMD_PIVOT_LEFT  ||
                        b == CMD_PIVOT_RIGHT || b == CMD_STOP    ||
                        b == CMD_BRAKE) {
                        cmd = b;
                        state = 2;
                    } else {
                        state = (b == SYNC_BYTE) ? 1 : 0;
                    }
                    break;

                case 2: pl = b; state = 3; break;
                case 3: pr = b; state = 4; break;

                case 4: {
                    uint8_t want = (uint8_t)(cmd ^ pl ^ pr ^ CHK_SALT);
                    if (b == want) {
                        apply_command(cmd, pl, pr);
                        last_packet_ms  = millis_now();
                        failsafe_latched = 0;
                    }
                    /* bad checksum: silently drop, keep the previous command
                     * until the failsafe window expires */
                    state = 0;
                    break;
                }

                default:
                    state = 0;
                    break;
            }
        }

        /* ---- FAILSAFE ---------------------------------------------------
         * The single most important addition. Without it, an ESP32 brownout
         * or a Bluetooth stack crash leaves the motors running at whatever
         * they were last told. */
        if ((millis_now() - last_packet_ms) > CMD_TIMEOUT_MS) {
            if (!failsafe_latched) {
                motors_stop();
                failsafe_latched = 1;
                uart_puts("FAILSAFE: no valid packet, motors stopped\r\n");
            }
        }
    }

    return 0;
}
