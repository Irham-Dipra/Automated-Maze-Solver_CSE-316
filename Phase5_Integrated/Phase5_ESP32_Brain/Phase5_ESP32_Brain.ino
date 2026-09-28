/* ===========================================================================
 *  Phase5_ESP32_Brain.ino  --  gyro-stabilised maze solver (ESP32 master)
 * ===========================================================================
 *
 *  Pairs with Phase5_ATmega_Slave.c. Flash BOTH -- the UART protocol changed
 *  (it now has a sync byte and a checksum) and the two halves will not talk
 *  to each other across versions.
 *
 *  ---------------------------------------------------------------------
 *  WHAT WAS ACTUALLY WRONG, AND WHAT THIS CHANGES
 *  ---------------------------------------------------------------------
 *  Read against bluetooth_logs/ and maheeCode/cse316_project-main/code/main.
 *
 *  A. Unframed UART let a single dropped byte turn a PWM value into a turn
 *     command. Fixed in the slave; this file emits the framed packets.
 *
 *  B. serial_20260923_134105.txt: "Yaw:-206.64 Adj:826 [PWM_L:0 PWM_R:255]"
 *     repeating forever. The old heading PID had (i) no clamp on the output,
 *     (ii) no reset of the heading accumulator between legs, and (iii) a
 *     proportional term on ABSOLUTE accumulated yaw. Once the robot lost 200
 *     degrees it commanded a permanently saturated differential and could
 *     never recover. Here the correction is clamped to a fraction of the base
 *     speed, the accumulator is zeroed at the start of every leg and after
 *     every pivot, and a yaw governor (from mahee's drive.c) refuses to add
 *     rotation the chassis is already doing.
 *
 *  C. serial_20260923_133856.txt: "Yaw:-7.38 ... [PWM_L:121 PWM_R:179]"
 *     held constant to two decimals for four straight seconds. A 58-count
 *     differential that produces zero rotation means the wheels are not
 *     turning at all -- dead motor rail, blown L298N channel, or a lost
 *     common ground. The old firmware could not tell and just kept
 *     integrating. checkStall() now detects "commanded but not moving" and
 *     stops with a message naming the likely cause, instead of silently
 *     winding up.
 *
 *  D. Timed turns ("delay(600)") cannot be calibrated across a discharging
 *     battery. Replaced with mahee's closed-loop pivot: kick -> slow sweep to
 *     (target - margin) -> active brake -> settle while still integrating the
 *     coast -> error-scaled nudges until inside the deadband.
 *
 *  E. The gyro was calibrated once at boot. MPU6050 bias drifts thermally,
 *     and a stale bias integrates into a phantom heading error that drives
 *     the robot into a wall on a long run. Now: spread-checked calibration
 *     that REJECTS a sample set taken while the chassis was moving, plus a
 *     quick re-calibration after every stop and every pivot (mahee's
 *     heading.c).
 *
 *  F. The old loop() called rangingTest() three times plus delay() -- 100 to
 *     180 ms per iteration, and fully blind during turns. The ToF sensors now
 *     run in CONTINUOUS mode and are polled non-blocking on a fixed 20 ms
 *     control tick, so the gyro integration has a constant dt (which is what
 *     makes the calibration constant meaningful at all).
 *
 *  G. Single bad readings caused turns. Ported from mahee's sonar.c: median
 *     of 3, staleness check, implausible-jump gate, a distinct "too close to
 *     measure" state that must NEVER be confused with "open", and front
 *     obstacle detection by VOTING (2 of the last 5) rather than consecutive
 *     hits -- because a yawing chassis walks the front beam off a real wall
 *     for several pings at a time.
 *
 *  H. 27sep1.txt shows FRONT failing to init on every boot. initSensor() now
 *     retries with an I2C bus recovery between attempts, and the maze logic
 *     degrades gracefully around a sensor that never came up.
 *
 *  I. Bluetooth is brought up LAST and the robot does not require a client.
 *     Losing the phone no longer matters; the slave failsafe covers a crash.
 *
 *  ---------------------------------------------------------------------
 *  WIRING (unchanged, from Assets/connections.md)
 *    I2C:  SDA=21  SCL=22   -- 3x VL53L0X + MPU6050 share the bus
 *    XSHUT: front=19  left=18  right=4
 *    UART:  TX2 (GPIO17) -> level shifter -> ATmega PD0 (RXD)
 *
 *  ---------------------------------------------------------------------
 *  FIRST BRING-UP -- do these in order, do not skip
 *    MODE:0   sensors only, motors never move.
 *             Turn the robot LEFT by hand: Hdg must go POSITIVE.
 *             If it goes negative, send  GS:-1  and re-check.
 *             Confirm F/L/R distances are sane and none reads "--".
 *    MODE:2   one 90 deg right pivot. Measure with a protractor.
 *             Consistently short -> lower TM. Consistently long -> raise TM.
 *    MODE:1   straight corridor run. Tune BL/BR/LT/RT until it tracks
 *             straight, then KP/KD.
 *    MODE:3   full maze.
 * ======================================================================== */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <BluetoothSerial.h>
#include "esp_system.h"
#include <stdarg.h>
#include <math.h>

/* ==========================================================================
 *  1. CONFIGURATION
 *  Everything tunable lives here. Most of it is also settable over Bluetooth
 *  at runtime -- these are just the power-on defaults.
 * ======================================================================== */

/* --- pins ------------------------------------------------------------- */
#define PIN_SDA              21
#define PIN_SCL              22
#define XSHUT_FRONT          19
#define XSHUT_LEFT           18
#define XSHUT_RIGHT           4
#define UART_TX              17      /* TX2 -> level shifter -> ATmega RXD */
#define UART_RX              16      /* unused today; wire ATmega TXD here
                                        later if you want acknowledgements  */

#define ADDR_FRONT         0x30
#define ADDR_LEFT          0x31
#define ADDR_RIGHT         0x32
#define ADDR_MPU           0x68

/* --- timing ----------------------------------------------------------- */
#define CONTROL_TICK_MS      20      /* fixed. the gyro constant assumes it */
#define TURN_TICK_MS          5      /* finer sampling inside a pivot       */
#define TOF_PERIOD_MS        30      /* continuous-ranging period per sensor*/
#define TELEMETRY_MS        150

/* --- maze geometry, in MILLIMETRES (VL53L0X native unit) -------------- */
#define CORRIDOR_WIDTH_MM   360      /* your ~360 mm book-wall hallway      */
#define CORRIDOR_HALF_MM    (CORRIDOR_WIDTH_MM / 2)

#define TOF_MAX_RANGE_MM   1200      /* beyond this we call it "open"       */
#define TOF_TOO_CLOSE_MM     40      /* below this the sensor is unreliable */
#define TOF_STALE_MS        250
#define TOF_MAX_JUMP_MM     200      /* a wall cannot move this fast        */
#define TOF_NEAR_LATCH_MM   120      /* lost echo + last reading this close
                                        means the wall got NEARER, not gone */

#define OPENING_THRESHOLD_MM  (CORRIDOR_HALF_MM + 100)   /* 280 mm */
#define FRONT_BLOCKED_MM      220
#define FRONT_STOP_MM         120    /* stop this far off a wall before pivot */

#define OPENING_CONFIRM       2
#define DEADEND_CONFIRM       3
#define FRONT_VOTE_WINDOW     5
#define FRONT_VOTE_THRESHOLD  2

/* --- approach offset -------------------------------------------------- */
/* The ToF sees an opening when the SENSOR is level with it, but the robot
 * pivots about the axle, further back. Drive on by this much so the pivot
 * centre -- not the nose -- ends up in the middle of the opening. */
#define SENSOR_TO_AXLE_MM   120      /* [MEASURE ON YOUR CHASSIS]          */
#define TRAVEL_SPEED_MMS    200      /* [MEASURE] mm/s at BASE_PWM          */
#define APPROACH_MM         (SENSOR_TO_AXLE_MM + CORRIDOR_HALF_MM)

/* --- motors ----------------------------------------------------------- */
/* MIN is the stall floor: below it the TT motors buzz but do not turn.
 * Your two motors are badly mismatched, so LT/RT trim each side
 * independently -- that mismatch is what made positional PID untunable. */
#define DEF_BASE_PWM        110
#define DEF_MIN_PWM          60
#define DEF_MAX_PWM         230
#define DEF_LEFT_TRIM       100      /* percent */
#define DEF_RIGHT_TRIM      100      /* percent */
#define KICK_PWM            200      /* breakaway pulse from standstill     */
#define KICK_MS              60

/* --- heading-hold PD -------------------------------------------------- */
/* corr > 0 steers RIGHT.  left = base + corr,  right = base - corr.
 * Correction is capped at a FRACTION OF THE BASE SPEED, so lowering the base
 * automatically softens the steering instead of sharpening it. This cap is
 * the direct fix for the Adj:826 runaway. */
#define DEF_KP              4.0f     /* PWM counts per degree of drift      */
#define DEF_KD              0.25f    /* PWM counts per deg/s of yaw rate    */
#define DEF_KW              0.020f   /* deg of heading target per mm of
                                        wall-centring error                 */
#define WALL_TILT_MAX_DEG    8.0f
#define CORR_RATIO_PCT        40     /* max |corr| as % of base PWM         */
#define YAW_GOVERNOR_DPS    60.0f    /* stop winding up past this rate      */

/* --- pivots ----------------------------------------------------------- */
#define DEF_TURN_PWM        150
#define TURN_KICK_PWM       210
#define TURN_KICK_MS         50
#define TURN_BRAKE_MS        60
#define DEF_STOP_MARGIN_DEG  8.0f    /* cut the sweep this early; the coast
                                        is then measured, not guessed       */
#define TURN_DEADBAND_DEG    2.0f
#define TURN_NUDGE_PWM      210
#define TURN_NUDGE_MS_MIN    20
#define TURN_NUDGE_MS_MAX    70
#define TURN_NUDGE_MS_PER_DEG 7
#define TURN_MAX_NUDGES       5
#define TURN_SETTLE_MS      300
#define TURN_TIMEOUT_MS    7000
#define TURN_180_AS_TWO_90S   1

/* --- gyro ------------------------------------------------------------- */
#define GYRO_LSB_PER_DPS   65.5f     /* FS_SEL=1 -> +-500 dps               */
#define GYRO_CAL_FULL      400
#define GYRO_CAL_QUICK     120
#define GYRO_CAL_INTERVAL_MS 2
#define GYRO_CAL_MAX_SPREAD 250      /* raw LSB; larger = chassis was moving */
#define GYRO_CAL_RETRIES     3
#define GYRO_SETTLE_MS     250

/* --- safety ----------------------------------------------------------- */
#define STARTUP_DELAY_MS   3000
#define RECOVER_MS          400      /* gyro-only driving after a pivot      */
#define MAX_LEG_MS        12000UL
#define MAX_RUN_MS       300000UL
#define STALL_CHECK_MS     1500
#define STALL_MIN_DEG       2.0f     /* rotation expected in that window     */
#define STALL_MIN_MM         40      /* or this much change in front range   */

/* --- protocol (must match the slave) ---------------------------------- */
#define SYNC_BYTE          0xA5
#define CHK_SALT           0x5A
#define HEARTBEAT_MS         50      /* resend even if nothing changed       */

/* ==========================================================================
 *  1b. TYPE DECLARATIONS
 *
 *  These live up here on purpose. The Arduino IDE's ESP32 build step
 *  auto-generates function prototypes and splices them in just after the last
 *  #define at the top of the sketch. Any type used in a signature --
 *  ToFState, TurnResult, MazeState, Junction -- must already be declared at
 *  that point, or the generated prototypes reference an unknown type and the
 *  sketch fails to compile with errors pointing at the wrong line.
 *  (This is about the .ino toolchain, not about Arduino hardware.)
 * ======================================================================== */
struct GyroXYZ { int16_t x, y, z; };

enum { S_FRONT = 0, S_LEFT = 1, S_RIGHT = 2, S_COUNT = 3 };
const char *S_NAME[S_COUNT] = { "FRONT", "LEFT", "RIGHT" };

struct ToFState {
    bool     present;        /* did begin() ever succeed                    */
    uint16_t hist[3];
    uint8_t  n, i;
    uint16_t latest;         /* TOF_MAX_RANGE_MM means "nothing in range"   */
    uint16_t last_good;
    bool     has_good;
    bool     too_close;      /* wall present but nearer than measurable     */
    bool     confident;
    uint32_t stamp;
};

struct TurnResult {
    float   achieved_deg;
    float   initial_err_deg; /* + undershot, - overshot, BEFORE any nudging */
    uint8_t nudges;
    bool    timed_out;
};

enum MazeState {
    ST_IDLE, ST_STARTUP, ST_DRIVING, ST_APPROACHING, ST_CONFIRM_EXIT,
    ST_STOPPING, ST_RECALIBRATING, ST_DECIDING, ST_RECOVERING, ST_FINISHED
};
const char *ST_NAME[] = { "IDLE","STARTUP","DRIVING","APPROACH","CONFIRM_EXIT",
                          "STOPPING","RECAL","DECIDING","RECOVER","FINISHED" };

enum Junction { J_CORRIDOR, J_DEAD_END, J_ALL_OPEN, J_FWD_OR_LEFT,
                J_FWD_OR_RIGHT, J_LEFT_OR_RIGHT, J_FORCED_LEFT, J_FORCED_RIGHT };

/* ==========================================================================
 *  2. RUNTIME-TUNABLE STATE
 * ======================================================================== */
int   cfg_base_pwm   = DEF_BASE_PWM;
int   cfg_min_pwm    = DEF_MIN_PWM;
int   cfg_max_pwm    = DEF_MAX_PWM;
int   cfg_left_trim  = DEF_LEFT_TRIM;
int   cfg_right_trim = DEF_RIGHT_TRIM;
int   cfg_turn_pwm   = DEF_TURN_PWM;
int   cfg_front_stop = FRONT_STOP_MM;
float cfg_kp         = DEF_KP;
float cfg_kd         = DEF_KD;
float cfg_kw         = DEF_KW;
float cfg_margin     = DEF_STOP_MARGIN_DEG;
int   cfg_gyro_sign  = 1;            /* flip with GS:-1 if mounted inverted */
int   cfg_mode       = 3;            /* 0 telemetry 1 straight 2 turn 3 maze */

bool  g_running      = false;        /* START/STOP from the phone           */

/* ==========================================================================
 *  3. LOGGING
 * ======================================================================== */
BluetoothSerial SerialBT;
bool bt_ready = false;

void LOG(const String &m) {
    Serial.println(m);
    if (bt_ready) SerialBT.println(m);
}
void LOGf(const char *fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.println(buf);
    if (bt_ready) SerialBT.println(buf);
}

/* ==========================================================================
 *  4. MOTOR LINK  --  framed packets + heartbeat
 *
 *  The slave stops the motors if it does not see a valid packet for 300 ms,
 *  so we must keep sending. Every blocking wait in this sketch calls
 *  motorHeartbeat() for exactly that reason.
 * ======================================================================== */
static char     g_last_cmd = 'S';
static uint8_t  g_last_l = 0, g_last_r = 0;
static uint32_t g_last_tx = 0;

void motorSend(char cmd, int left, int right) {
    if (left  < 0)   left  = 0;
    if (left  > 255) left  = 255;
    if (right < 0)   right = 0;
    if (right > 255) right = 255;

    uint8_t c = (uint8_t)cmd, l = (uint8_t)left, r = (uint8_t)right;
    uint8_t chk = c ^ l ^ r ^ CHK_SALT;

    Serial2.write((uint8_t)SYNC_BYTE);
    Serial2.write(c);
    Serial2.write(l);
    Serial2.write(r);
    Serial2.write(chk);

    g_last_cmd = cmd; g_last_l = l; g_last_r = r;
    g_last_tx  = millis();
}

void motorHeartbeat(void) {
    if (millis() - g_last_tx >= HEARTBEAT_MS)
        motorSend(g_last_cmd, g_last_l, g_last_r);
}

void motorStop(void)                      { motorSend('S', 0, 0); }
void motorBrake(void)                     { motorSend('K', 0, 0); }
void motorForward(int l, int r)           { motorSend('F', l, r); }
/* clockwise != 0 pivots the chassis to the RIGHT */
void motorPivot(bool clockwise, int pwm)  { motorSend(clockwise ? 'R' : 'L', pwm, pwm); }

/* ==========================================================================
 *  5. MPU6050  --  direct register access, no library
 * ======================================================================== */
bool mpu_ok = false;

static bool mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR_MPU);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool mpuInit(void) {
    if (!mpuWrite(0x6B, 0x00)) return false;   /* wake from sleep           */
    delay(50);
    /* DLPF at 44 Hz. The old code left this wide open, so every bit of
     * gear-motor vibration went straight into the heading integral. */
    mpuWrite(0x1A, 0x03);
    mpuWrite(0x1B, 0x08);                      /* FS_SEL=1 -> +-500 dps     */
    mpuWrite(0x19, 0x04);                      /* sample rate 200 Hz        */
    delay(20);
    return true;
}

bool mpuReadAll(GyroXYZ &g) {
    Wire.beginTransmission(ADDR_MPU);
    Wire.write(0x43);                          /* GYRO_XOUT_H               */
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)ADDR_MPU, (uint8_t)6) != 6) return false;
    uint8_t b[6];
    for (int i = 0; i < 6; i++) b[i] = Wire.read();
    g.x = (int16_t)((b[0] << 8) | b[1]);
    g.y = (int16_t)((b[2] << 8) | b[3]);
    g.z = (int16_t)((b[4] << 8) | b[5]);
    return true;
}

/* ==========================================================================
 *  6. HEADING  --  calibration, integration, drift detection
 *  Ported from maheeCode .../heading.c
 * ======================================================================== */
float g_off_z = 0, g_off_x = 0, g_off_y = 0;
float g_heading_deg = 0;     /* + = rotated LEFT since the last reset       */
float g_rate_dps    = 0;

void headingReset(void) { g_heading_deg = 0; }

/* Averages N stationary samples. If the spread across the run is too wide the
 * chassis was NOT still, so the average is meaningless and the previous
 * offsets are kept rather than corrupted. This rejection is the part the old
 * firmware was missing -- calibrating while someone was still holding the car
 * baked a phantom rate into every subsequent heading. */
static bool calibrateOnce(int samples) {
    long sx = 0, sy = 0, sz = 0;
    int16_t zmin = 32767, zmax = -32768;
    GyroXYZ g;

    for (int i = 0; i < samples; i++) {
        if (!mpuReadAll(g)) return false;
        sx += g.x; sy += g.y; sz += g.z;
        if (g.z < zmin) zmin = g.z;
        if (g.z > zmax) zmax = g.z;
        delay(GYRO_CAL_INTERVAL_MS);
        motorHeartbeat();
    }
    if ((long)zmax - (long)zmin > GYRO_CAL_MAX_SPREAD) return false;

    g_off_z = (float)sz / samples;
    g_off_x = (float)sx / samples;
    g_off_y = (float)sy / samples;
    return true;
}

bool gyroCalibrate(int samples, const char *what) {
    motorStop();
    delay(GYRO_SETTLE_MS);
    for (int t = 0; t < GYRO_CAL_RETRIES; t++) {
        if (calibrateOnce(samples)) {
            LOGf("%s ok  offZ=%.1f offX=%.1f offY=%.1f", what, g_off_z, g_off_x, g_off_y);
            return true;
        }
        LOGf("%s rejected (chassis moving), retry %d", what, t + 1);
        delay(GYRO_SETTLE_MS);
    }
    LOGf("%s FAILED -- keeping previous offset %.1f", what, g_off_z);
    return false;
}

/* One gyro sample, integrated over an explicit dt. Carrying dt explicitly is
 * what lets the 5 ms turn loop and the 20 ms drive loop share one constant. */
void headingSample(uint16_t dt_ms) {
    GyroXYZ g;
    if (!mpuReadAll(g)) return;
    g_rate_dps     = cfg_gyro_sign * ((float)g.z - g_off_z) / GYRO_LSB_PER_DPS;
    g_heading_deg += g_rate_dps * (dt_ms / 1000.0f);
}

/* ==========================================================================
 *  7. ToF SENSORS  --  continuous ranging, filtered
 *  Ported from maheeCode .../sonar.c, converted to millimetres
 * ======================================================================== */
Adafruit_VL53L0X tof[S_COUNT];
ToFState         st[S_COUNT];

uint8_t  front_votes[FRONT_VOTE_WINDOW];
uint8_t  front_vi = 0;

const uint8_t XSHUT[S_COUNT] = { XSHUT_FRONT, XSHUT_LEFT, XSHUT_RIGHT };
const uint8_t TOF_ADDR[S_COUNT] = { ADDR_FRONT, ADDR_LEFT, ADDR_RIGHT };

void i2cBusRecover(void) {
    /* Nine clocks with SDA released frees a slave that is mid-byte and
     * holding the bus down -- the classic "sensors freeze after a motor
     * spike" lockup from the progress report. */
    Wire.end();
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
        digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
    }
    pinMode(PIN_SDA, OUTPUT);
    digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);   /* STOP */

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);     /* 100 kHz tolerates long noisy wiring far
                                  better than 400 kHz */
    delay(20);
}

void tofClear(int id) {
    st[id].n = 0; st[id].i = 0;
    st[id].latest = TOF_MAX_RANGE_MM;
    st[id].last_good = 0;
    st[id].has_good = false;
    st[id].too_close = false;
    st[id].confident = false;
    st[id].stamp = 0;
    for (int k = 0; k < 3; k++) st[id].hist[k] = TOF_MAX_RANGE_MM;
}

void tofFlush(void) {
    for (int i = 0; i < S_COUNT; i++) tofClear(i);
    for (int i = 0; i < FRONT_VOTE_WINDOW; i++) front_votes[i] = 0;
    front_vi = 0;
}

/* Bring one sensor up at its own address. 27sep1.txt shows FRONT failing on
 * every single boot, so this retries with a bus recovery in between instead
 * of giving up after one try. */
bool tofInitOne(int id) {
    for (int attempt = 0; attempt < 3; attempt++) {
        if (tof[id].begin(TOF_ADDR[id], false, &Wire)) {
            tof[id].setMeasurementTimingBudgetMicroSeconds(20000);
            tof[id].startRangeContinuous(TOF_PERIOD_MS);
            LOGf("  [OK]   %-5s at 0x%02X", S_NAME[id], TOF_ADDR[id]);
            return true;
        }
        LOGf("  ...    %-5s attempt %d failed, recovering bus", S_NAME[id], attempt + 1);
        i2cBusRecover();
        delay(50);
    }
    LOGf("  [FAIL] %-5s NOT FOUND -- running degraded without it", S_NAME[id]);
    return false;
}

void tofInitAll(void) {
    for (int i = 0; i < S_COUNT; i++) {
        pinMode(XSHUT[i], OUTPUT);
        digitalWrite(XSHUT[i], LOW);
        tofClear(i);
        st[i].present = false;
    }
    delay(50);                       /* hold all three in reset */

    /* One at a time: each sensor must be readdressed away from the default
     * 0x29 before the next one is released, or they collide. Staggered with
     * pauses so their init current spikes do not stack. */
    for (int i = 0; i < S_COUNT; i++) {
        digitalWrite(XSHUT[i], HIGH);
        delay(100);
        st[i].present = tofInitOne(i);
        delay(50);
    }
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    uint16_t t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

/* Poll one sensor. Non-blocking: if the continuous measurement is not ready
 * yet we simply return and try again next tick. */
void tofPoll(int id) {
    if (!st[id].present) return;
    if (!tof[id].isRangeComplete()) return;

    uint16_t raw = tof[id].readRange();
    uint8_t  sts = tof[id].readRangeStatus();
    ToFState &p = st[id];

    uint16_t v;
    if (sts != 0 || raw == 0 || raw >= 8000) v = TOF_MAX_RANGE_MM;   /* no echo */
    else if (raw > TOF_MAX_RANGE_MM)         v = TOF_MAX_RANGE_MM;
    else                                     v = raw;

    /* Disambiguate a lost echo. Very near or very dark surfaces give the same
     * "out of range" status as open space. History resolves it: a wall that
     * was 8 cm away 60 ms ago did not vanish, it got closer. Conflating the
     * two is how the old code reported an OPENING at the moment of impact. */
    bool too_close = (v <= TOF_TOO_CLOSE_MM);
    if (v >= TOF_MAX_RANGE_MM && p.has_good && p.last_good <= TOF_NEAR_LATCH_MM)
        too_close = true;

    if (too_close) {
        v = TOF_TOO_CLOSE_MM;          /* a real number to steer away from */
        p.too_close = true;
        p.last_good = v;
        p.has_good  = true;
    } else {
        p.too_close = false;
        if (v < TOF_MAX_RANGE_MM) { p.last_good = v; p.has_good = true; }
    }

    /* Plausibility gate: a wall cannot appear to jump further than the robot
     * can travel between refreshes. Usually it means the beam clipped a
     * corner during a correction turn. */
    bool jump = false;
    if (p.n > 0 && v < TOF_MAX_RANGE_MM && p.latest < TOF_MAX_RANGE_MM) {
        int d = (int)v - (int)p.latest;
        if (d < 0) d = -d;
        jump = (d > TOF_MAX_JUMP_MM);
    }

    p.latest = v;
    p.hist[p.i] = v;
    p.i = (p.i + 1) % 3;
    if (p.n < 3) p.n++;
    p.stamp = millis();
    /* a too-close reading is a safety signal and is never filtered as noise */
    p.confident = p.too_close ? true : !jump;

    if (id == S_FRONT) {
        uint8_t blocked = p.too_close ? 1
                        : ((!jump && v < FRONT_BLOCKED_MM) ? 1 : 0);
        front_votes[front_vi] = blocked;
        front_vi = (front_vi + 1) % FRONT_VOTE_WINDOW;
    }
}

void tofTask(void) { for (int i = 0; i < S_COUNT; i++) tofPoll(i); }

uint16_t tofMedian(int id) {
    ToFState &p = st[id];
    if (p.n < 3) return p.latest;
    return median3(p.hist[0], p.hist[1], p.hist[2]);
}
uint16_t tofLatest(int id) { return st[id].latest; }

bool tofValid(int id) {
    ToFState &p = st[id];
    if (!p.present || p.n == 0) return false;
    if (millis() - p.stamp > TOF_STALE_MS) return false;
    if (!p.confident) return false;
    if (p.latest >= TOF_MAX_RANGE_MM) return false;
    return true;
}

bool tofOpen(int id) {
    if (!st[id].present) return true;      /* unknown side: assume passable */
    if (st[id].too_close) return false;    /* checked FIRST -- see above    */
    uint16_t v = tofLatest(id);
    if (v >= TOF_MAX_RANGE_MM) return true;
    return v > OPENING_THRESHOLD_MM;
}

bool tofTooClose(int id) { return st[id].present && st[id].too_close; }

uint8_t frontVotes(void) {
    uint8_t v = 0;
    for (int i = 0; i < FRONT_VOTE_WINDOW; i++) v += front_votes[i];
    return v;
}

/* Voting, not consecutive hits: a sustained correction turn walks the front
 * beam off a real wall for several pings in a row, so "2 consecutive" can
 * never fire. Obstacles do not vanish; intermittent evidence is enough. */
bool frontBlocked(void) {
    if (!st[S_FRONT].present) return false;
    return frontVotes() >= FRONT_VOTE_THRESHOLD;
}

/* ==========================================================================
 *  8. DRIVE  --  heading-hold PD with wall-centring as a heading BIAS
 *
 *  The old design ran two controllers against each other: a positional PID on
 *  the side sensors and, later, a heading PID on the gyro. They fought, and
 *  with mismatched motors the positional one oscillated between stalled and
 *  full power.
 *
 *  Here there is exactly ONE controller. The gyro holds the heading; the side
 *  walls only nudge the heading TARGET by a few degrees. Set KW:0 to make it
 *  pure gyro.
 * ======================================================================== */
float g_target_heading = 0;
int   g_pwm_l = 0, g_pwm_r = 0;
float g_last_corr = 0;

/* stall watchdog */
uint32_t g_stall_t0 = 0;
float    g_stall_hdg0 = 0;
uint16_t g_stall_front0 = 0;
bool     g_stalled = false;

void driveBegin(void) {
    headingReset();
    g_target_heading = 0;
    g_last_corr = 0;
    g_stalled = false;
    g_stall_t0 = millis();
    g_stall_hdg0 = g_heading_deg;
    g_stall_front0 = tofLatest(S_FRONT);
    /* breakaway kick: these motors will not start from rest at cruise PWM */
    motorForward((KICK_PWM * cfg_left_trim) / 100, (KICK_PWM * cfg_right_trim) / 100);
    uint32_t t0 = millis();
    while (millis() - t0 < KICK_MS) { motorHeartbeat(); }
}

void driveStop(void) {
    motorStop();
    g_pwm_l = g_pwm_r = 0;
    g_last_corr = 0;
}

/* "Commanded to move but neither rotating nor closing on anything." That is
 * the serial_20260923_133856 signature: a 121/179 differential with the yaw
 * frozen at -7.38 for four seconds. Report it instead of integrating into a
 * runaway. */
void checkStall(void) {
    if (g_pwm_l == 0 && g_pwm_r == 0) { g_stall_t0 = millis(); return; }

    /* Only judge while the front sensor has a real wall in range. Driving
     * straight down a long corridor with nothing ahead legitimately produces
     * no yaw change AND no front-range change -- with no encoders there is
     * nothing left to distinguish that from a seized wheel, so do not guess.
     * Inside a maze the front wall is almost always within range and closes
     * at TRAVEL_SPEED_MMS, which is the case this check is really for. */
    if (!tofValid(S_FRONT)) {
        g_stall_t0     = millis();
        g_stall_hdg0   = g_heading_deg;
        g_stall_front0 = tofLatest(S_FRONT);
        return;
    }

    if (millis() - g_stall_t0 < STALL_CHECK_MS) return;

    float dh = fabsf(g_heading_deg - g_stall_hdg0);
    int   df = (int)tofLatest(S_FRONT) - (int)g_stall_front0;
    if (df < 0) df = -df;

    if (dh < STALL_MIN_DEG && df < STALL_MIN_MM) {
        g_stalled = true;
        driveStop();
        LOG("");
        LOG("*** STALL DETECTED ***");
        LOGf("  commanded L=%d R=%d for %d ms, but the robot did not move",
             g_pwm_l, g_pwm_r, STALL_CHECK_MS);
        LOGf("  yaw changed %.2f deg, front range changed %d mm", dh, df);
        LOG("  This is a POWER/DRIVER fault, not a tuning problem. Check:");
        LOG("   1. motor battery voltage under load (should stay > 6.5 V)");
        LOG("   2. L298N GND -> breadboard GND  (the common ground)");
        LOG("   3. L298N output channels -- swap the motors to test");
        LOG("   4. raise BL if the wheels only buzz (below the stall floor)");
        LOG("  Send START to try again.");
        g_running = false;
        return;
    }
    g_stall_t0 = millis();
    g_stall_hdg0 = g_heading_deg;
    g_stall_front0 = tofLatest(S_FRONT);
}

void driveTick(void) {
    bool l_ok = tofValid(S_LEFT);
    bool r_ok = tofValid(S_RIGHT);
    int  l_mm = tofMedian(S_LEFT);
    int  r_mm = tofMedian(S_RIGHT);

    /* --- wall-centring, expressed as a heading TARGET ------------------- */
    float wall_err_mm = 0;
    if (l_ok && r_ok)        wall_err_mm = (float)(r_mm - l_mm);        /* width-independent */
    else if (l_ok)           wall_err_mm = (float)(CORRIDOR_HALF_MM - l_mm) * 2.0f;
    else if (r_ok)           wall_err_mm = (float)(r_mm - CORRIDOR_HALF_MM) * 2.0f;

    /* more room on the right -> aim a few degrees right (negative heading) */
    g_target_heading = -cfg_kw * wall_err_mm;
    if (g_target_heading >  WALL_TILT_MAX_DEG) g_target_heading =  WALL_TILT_MAX_DEG;
    if (g_target_heading < -WALL_TILT_MAX_DEG) g_target_heading = -WALL_TILT_MAX_DEG;

    /* Emergency override: a wall inside the too-close band needs a hard,
     * ramped steer-away, not a 3-degree hint. */
    if (tofTooClose(S_LEFT)  && !tofTooClose(S_RIGHT)) g_target_heading = -WALL_TILT_MAX_DEG * 2;
    if (tofTooClose(S_RIGHT) && !tofTooClose(S_LEFT))  g_target_heading =  WALL_TILT_MAX_DEG * 2;

    /* --- PD ------------------------------------------------------------ */
    float drift = g_heading_deg - g_target_heading;   /* + = drifted left   */
    float corr  = cfg_kp * drift + cfg_kd * g_rate_dps;

    /* YAW GOVERNOR. Past this rate the ToF sensors point far enough off-axis
     * that they stop describing the corridor -- the front beam in particular
     * walks off whatever is ahead. Stop ADDING rotation in that direction;
     * do not reverse, just let it decay. */
    if (corr > 0 && g_rate_dps < -YAW_GOVERNOR_DPS) corr = 0;
    if (corr < 0 && g_rate_dps >  YAW_GOVERNOR_DPS) corr = 0;

    /* THE CLAMP. This one line is what prevents Adj:826 / PWM 0-vs-255. */
    float cap = (cfg_base_pwm * CORR_RATIO_PCT) / 100.0f;
    if (corr >  cap) corr =  cap;
    if (corr < -cap) corr = -cap;
    g_last_corr = corr;

    int l = cfg_base_pwm + (int)corr;
    int r = cfg_base_pwm - (int)corr;

    /* Preserve the differential when a wheel would fall under the stall
     * floor: lift BOTH rather than clipping one, so the robot keeps steering
     * instead of just slowing down. (mahee's drive.c) */
    if (l < cfg_min_pwm) { r += (cfg_min_pwm - l); l = cfg_min_pwm; }
    if (r < cfg_min_pwm) { l += (cfg_min_pwm - r); r = cfg_min_pwm; }

    l = (l * cfg_left_trim)  / 100;
    r = (r * cfg_right_trim) / 100;
    if (l > cfg_max_pwm) l = cfg_max_pwm;
    if (r > cfg_max_pwm) r = cfg_max_pwm;
    if (l < 0) l = 0;
    if (r < 0) r = 0;

    g_pwm_l = l; g_pwm_r = r;
    motorForward(l, r);
    checkStall();
}

/* ==========================================================================
 *  9. PIVOTS  --  closed loop on the gyro
 *  Ported from maheeCode .../turn.c
 *
 *  Timed turns cannot work: the same 600 ms gives a different angle on a full
 *  battery than on a flat one, and a different angle again on carpet. This
 *  measures the angle instead, including the coast after the motors are cut.
 * ======================================================================== */
static void turnSampleUntil(uint32_t &next_ms) {
    while ((int32_t)(millis() - next_ms) < 0) { motorHeartbeat(); }
    headingSample(TURN_TICK_MS);
    next_ms += TURN_TICK_MS;
}

/* Drive the pivot for a fixed time WHILE STILL INTEGRATING, so no rotation
 * happens uncounted. Used for the kick, the brake and each nudge. */
static void pulseTracked(bool cw, int pwm, uint16_t ms, uint32_t &next_ms) {
    uint32_t t0 = millis();
    motorPivot(cw, pwm);
    while (millis() - t0 < ms) turnSampleUntil(next_ms);
}

/* Motors off, still integrating: the chassis coasts after power is cut and
 * that coast is real rotation. Not counting it is why timed turns overshoot. */
static void settleTracked(uint16_t ms, uint32_t &next_ms) {
    uint32_t t0 = millis();
    motorStop();
    while (millis() - t0 < ms) turnSampleUntil(next_ms);
}

void turnExecute(float degrees, bool right, TurnResult &res) {
    const bool cw = right;
    uint32_t next_ms = millis() + TURN_TICK_MS;
    uint32_t t_start = millis();

    headingReset();
    res.timed_out = false;
    res.nudges = 0;

    /* 1. kick -- a pivot skids the tyres sideways and needs more breakaway
     *    torque than rolling straight. Counted, because it is real rotation. */
    pulseTracked(cw, TURN_KICK_PWM, TURN_KICK_MS, next_ms);

    /* 2. slow sweep, stopping EARLY on purpose */
    float stop_at = degrees - cfg_margin;
    motorPivot(cw, cfg_turn_pwm);
    while (fabsf(g_heading_deg) < stop_at) {
        if (millis() - t_start > TURN_TIMEOUT_MS) { res.timed_out = true; break; }
        turnSampleUntil(next_ms);
    }

    /* 3. active brake (reverse pivot), tracked */
    pulseTracked(!cw, TURN_NUDGE_PWM, TURN_BRAKE_MS, next_ms);

    /* 4. settle, still counting the coast */
    settleTracked(TURN_SETTLE_MS, next_ms);

    res.initial_err_deg = degrees - fabsf(g_heading_deg);

    /* 5. closed-loop correction. The chassis is stopped, so the accumulator
     *    now holds the TRUE angle. Nudge length scales with the remaining
     *    error -- a fixed pulse either crawls toward a large gap or blows
     *    through a tiny one, which oscillates instead of converging. */
    for (int i = 0; i < TURN_MAX_NUDGES; i++) {
        float err = degrees - fabsf(g_heading_deg);
        if (fabsf(err) <= TURN_DEADBAND_DEG) break;
        if (millis() - t_start > TURN_TIMEOUT_MS) { res.timed_out = true; break; }

        int ms = (int)(fabsf(err) * TURN_NUDGE_MS_PER_DEG);
        if (ms < TURN_NUDGE_MS_MIN) ms = TURN_NUDGE_MS_MIN;
        if (ms > TURN_NUDGE_MS_MAX) ms = TURN_NUDGE_MS_MAX;

        pulseTracked((err > 0) ? cw : !cw, TURN_NUDGE_PWM, ms, next_ms);
        settleTracked(TURN_SETTLE_MS, next_ms);
        res.nudges++;
    }

    motorStop();
    res.achieved_deg = fabsf(g_heading_deg);

    /* The chassis is stationary here -- the only moment a valid bias
     * measurement is possible. And everything in the ToF filters was measured
     * pointing a different direction, so throw it away. */
    gyroCalibrate(GYRO_CAL_QUICK, "recal");
    tofFlush();
    headingReset();
}

void turn90(bool right, TurnResult &res) { turnExecute(90.0f, right, res); }

void turn180(TurnResult &res) {
#if TURN_180_AS_TWO_90S
    /* Two 90s with a settle between beats one long sweep: momentum has less
     * time to build, so there is less coast to correct for. */
    TurnResult a, b;
    turnExecute(90.0f, true, a);
    delay(200);
    turnExecute(90.0f, true, b);
    res.achieved_deg    = a.achieved_deg + b.achieved_deg;
    res.initial_err_deg = a.initial_err_deg + b.initial_err_deg;
    res.nudges          = a.nudges + b.nudges;
    res.timed_out       = a.timed_out || b.timed_out;
#else
    turnExecute(180.0f, true, res);
#endif
}

/* ==========================================================================
 *  10. MAZE STATE MACHINE  --  right-hand rule
 *  Structure ported from maheeCode .../maze.c; the decision rule is yours.
 * ======================================================================== */
MazeState g_state = ST_IDLE;
uint32_t  g_state_t0 = 0;
uint32_t  g_leg_t0 = 0;
uint32_t  g_run_t0 = 0;
bool      g_recover_begun = false;

uint8_t   cnt_open_l = 0, cnt_open_r = 0, cnt_block_f = 0, cnt_deadend = 0;
bool      pend_right = true;
bool      pend_180 = false;

char      g_path[128];
int       g_path_len = 0;

void enterState(MazeState s) {
    g_state = s;
    g_state_t0 = millis();
    g_recover_begun = false;
}
bool inStateFor(uint32_t ms) { return millis() - g_state_t0 >= ms; }

void recordTurn(char c) {
    if (g_path_len < (int)sizeof(g_path) - 1) g_path[g_path_len++] = c;
    g_path[g_path_len] = 0;
    LOGf("PATH: %s", g_path);
}

void updateDebounce(void) {
    if (tofOpen(S_LEFT))  { if (cnt_open_l  < 255) cnt_open_l++;  } else cnt_open_l  = 0;
    if (tofOpen(S_RIGHT)) { if (cnt_open_r  < 255) cnt_open_r++;  } else cnt_open_r  = 0;
    if (frontBlocked())   { if (cnt_block_f < 255) cnt_block_f++; } else cnt_block_f = 0;

    if (frontBlocked() && !tofOpen(S_LEFT) && !tofOpen(S_RIGHT)) {
        if (cnt_deadend < 255) cnt_deadend++;
    } else cnt_deadend = 0;
}

Junction classify(void) {
    bool lo = cnt_open_l  >= OPENING_CONFIRM;
    bool ro = cnt_open_r  >= OPENING_CONFIRM;
    bool fw = cnt_block_f >= OPENING_CONFIRM;

    if (cnt_deadend >= DEADEND_CONFIRM)  return J_DEAD_END;
    if (!fw &&  lo &&  ro)               return J_ALL_OPEN;
    if (!fw &&  lo && !ro)               return J_FWD_OR_LEFT;
    if (!fw && !lo &&  ro)               return J_FWD_OR_RIGHT;
    if ( fw &&  lo &&  ro)               return J_LEFT_OR_RIGHT;
    if ( fw &&  lo && !ro)               return J_FORCED_LEFT;
    if ( fw && !lo &&  ro)               return J_FORCED_RIGHT;
    return J_CORRIDOR;
}

/* RIGHT-HAND RULE: always take the rightmost available opening. Unlike
 * mahee's random chooser this is deterministic, which is what makes the
 * recorded path in g_path worth anything. */
bool chooseTurn(Junction j, bool &turn_now, bool &right) {
    turn_now = true;
    switch (j) {
        case J_FWD_OR_RIGHT:  right = true;  break;   /* right wins over fwd  */
        case J_FORCED_RIGHT:  right = true;  break;
        case J_LEFT_OR_RIGHT: right = true;  break;
        case J_FWD_OR_LEFT:   turn_now = false; break;/* prefer straight      */
        case J_FORCED_LEFT:   right = false; break;
        default:              turn_now = false; break;
    }
    return turn_now;
}

void mazeTick(void) {
    switch (g_state) {

    case ST_STARTUP:
        motorStop();
        if (inStateFor(STARTUP_DELAY_MS)) {
            LOG(">>> GO <<<");
            driveBegin();
            g_leg_t0 = millis();
            enterState(ST_DRIVING);
        }
        break;

    case ST_DRIVING: {
        driveTick();
        if (g_stalled) { enterState(ST_IDLE); break; }
        updateDebounce();
        Junction j = classify();

        if (j == J_DEAD_END) {
            LOG("DEAD END -> 180");
            pend_180 = true;
            driveStop();
            enterState(ST_STOPPING);
            break;
        }
        if (j == J_ALL_OPEN) {
            /* At a left-or-right T, both side openings can appear a moment
             * before the front wall closes in -- identical to the exit for a
             * short window. Drive on a little and re-check. */
            LOG("all open -- confirming exit");
            enterState(ST_CONFIRM_EXIT);
            break;
        }
        if (j != J_CORRIDOR) {
            bool turn_now, right;
            chooseTurn(j, turn_now, right);
            if (turn_now) {
                pend_right = right;
                pend_180 = false;
                enterState(ST_APPROACHING);   /* drive the axle to the opening */
            } else {
                /* chose to continue straight: suppress re-triggering on this
                 * same opening until it has passed out of view */
                cnt_open_l = cnt_open_r = 0;
            }
            break;
        }
        /* safety net for a wall the ToF never saw (angled or dark surfaces
         * reflect the beam away and read as clear) */
        if (millis() - g_leg_t0 > MAX_LEG_MS) {
            LOG("leg timeout -- forcing a decision");
            pend_180 = false; pend_right = true;
            driveStop();
            enterState(ST_STOPPING);
        }
        break;
    }

    case ST_APPROACHING:
        driveTick();
        if (g_stalled) { enterState(ST_IDLE); break; }
        if (tofValid(S_FRONT) && tofLatest(S_FRONT) < cfg_front_stop) {
            LOG("front wall close -- stopping short");
            driveStop();
            enterState(ST_STOPPING);
            break;
        }
        if (inStateFor((uint32_t)APPROACH_MM * 1000UL / TRAVEL_SPEED_MMS)) {
            driveStop();
            enterState(ST_STOPPING);
        }
        break;

    case ST_CONFIRM_EXIT:
        driveTick();
        updateDebounce();
        if (frontBlocked()) {
            LOG("not the exit -- front wall appeared");
            enterState(ST_DRIVING);
            break;
        }
        if (inStateFor(1500)) {
            if (classify() == J_ALL_OPEN) {
                driveStop();
                LOG("");
                LOG("========================");
                LOGf("MAZE COMPLETE. PATH: %s", g_path);
                LOG("========================");
                enterState(ST_FINISHED);
            } else {
                enterState(ST_DRIVING);
            }
        }
        break;

    case ST_STOPPING:
        motorStop();
        if (inStateFor(GYRO_SETTLE_MS)) enterState(ST_RECALIBRATING);
        break;

    case ST_RECALIBRATING:
        gyroCalibrate(GYRO_CAL_QUICK, "recal");
        headingReset();
        tofFlush();
        enterState(ST_DECIDING);
        break;

    case ST_DECIDING: {
        TurnResult r;
        if (pend_180) {
            LOG("180...");
            recordTurn('U');
            turn180(r);
        } else {
            LOGf("turn %s...", pend_right ? "RIGHT" : "LEFT");
            recordTurn(pend_right ? 'R' : 'L');
            turn90(pend_right, r);
        }
        LOGf("  achieved %.1f deg  (pre-nudge err %.1f)  nudges=%d%s",
             r.achieved_deg, r.initial_err_deg, r.nudges,
             r.timed_out ? "  TIMED OUT" : "");
        enterState(ST_RECOVERING);
        break;
    }

    case ST_RECOVERING:
        /* The ToF filters were flushed after the pivot. Drive gyro-only until
         * they refill, so a half-empty filter cannot steer the robot. */
        if (!g_recover_begun) { driveBegin(); g_recover_begun = true; }
        driveTick();
        if (g_stalled) { enterState(ST_IDLE); break; }
        if (inStateFor(RECOVER_MS)) {
            cnt_open_l = cnt_open_r = cnt_block_f = cnt_deadend = 0;
            g_leg_t0 = millis();
            enterState(ST_DRIVING);
        }
        break;

    case ST_FINISHED:
    case ST_IDLE:
    default:
        motorStop();
        break;
    }
}

/* ==========================================================================
 *  11. BLUETOOTH COMMAND MENU
 * ======================================================================== */
void printMenu(void) {
    LOG("");
    LOG("=== PHASE 5 GYRO MAZE SOLVER ===");
    LOGf("  MODE:%d   0=sensors 1=straight 2=turn-test 3=maze", cfg_mode);
    LOGf("  BL:%d     base PWM          MN:%d  stall floor", cfg_base_pwm, cfg_min_pwm);
    LOGf("  MX:%d     max PWM           TP:%d  pivot PWM",   cfg_max_pwm, cfg_turn_pwm);
    LOGf("  LT:%d     left trim %%       RT:%d  right trim %%", cfg_left_trim, cfg_right_trim);
    LOGf("  KP:%.2f  KD:%.2f  KW:%.4f   TM:%.1f pivot stop margin",
         cfg_kp, cfg_kd, cfg_kw, cfg_margin);
    LOGf("  FD:%d     front stop mm     GS:%d  gyro sign", cfg_front_stop, cfg_gyro_sign);
    LOG("  START | STOP | CAL | MENU");
    LOG("================================");
}

void handleCommand(String c) {
    c.trim();
    c.toUpperCase();
    if (!c.length()) return;

    if (c == "START") {
        if (!mpu_ok) { LOG("refusing to start: MPU6050 not responding"); return; }
        g_running = true;
        g_stalled = false;
        g_path_len = 0; g_path[0] = 0;
        g_run_t0 = millis();
        tofFlush();
        headingReset();
        cnt_open_l = cnt_open_r = cnt_block_f = cnt_deadend = 0;
        enterState(ST_STARTUP);
        LOG(">>> STARTING <<<");
        return;
    }
    if (c == "STOP")  { g_running = false; driveStop(); enterState(ST_IDLE); LOG(">>> HALTED <<<"); return; }
    if (c == "MENU")  { printMenu(); return; }
    if (c == "CAL")   { driveStop(); gyroCalibrate(GYRO_CAL_FULL, "full cal"); headingReset(); return; }

    int colon = c.indexOf(':');
    if (colon < 0) { LOG("? unknown command -- send MENU"); return; }
    String k = c.substring(0, colon);
    float  v = c.substring(colon + 1).toFloat();

    if      (k == "MODE") cfg_mode       = (int)v;
    else if (k == "BL")   cfg_base_pwm   = (int)v;
    else if (k == "MN")   cfg_min_pwm    = (int)v;
    else if (k == "MX")   cfg_max_pwm    = (int)v;
    else if (k == "TP")   cfg_turn_pwm   = (int)v;
    else if (k == "LT")   cfg_left_trim  = (int)v;
    else if (k == "RT")   cfg_right_trim = (int)v;
    else if (k == "FD")   cfg_front_stop = (int)v;
    else if (k == "KP")   cfg_kp         = v;
    else if (k == "KD")   cfg_kd         = v;
    else if (k == "KW")   cfg_kw         = v;
    else if (k == "TM")   cfg_margin     = v;
    else if (k == "GS")   cfg_gyro_sign  = (v < 0) ? -1 : 1;
    else { LOG("? unknown key -- send MENU"); return; }
    printMenu();
}

void pumpCommands(void) {
    static String line;
    while (Serial.available())   { char ch = Serial.read();
        if (ch == '\n' || ch == '\r') { handleCommand(line); line = ""; } else line += ch; }
    while (bt_ready && SerialBT.available()) { char ch = SerialBT.read();
        if (ch == '\n' || ch == '\r') { handleCommand(line); line = ""; } else line += ch; }
}

/* ==========================================================================
 *  12. TELEMETRY
 * ======================================================================== */
String rangeStr(int id) {
    if (!st[id].present) return String("--");
    if (tofTooClose(id)) return String("<<");
    uint16_t v = tofMedian(id);
    if (v >= TOF_MAX_RANGE_MM) return String("oo");
    return String(v);
}

void telemetry(void) {
    static uint32_t last = 0;
    if (millis() - last < TELEMETRY_MS) return;
    last = millis();
    LOGf("%-8s F:%-5s L:%-5s R:%-5s fv:%d | Hdg:%+6.1f Tgt:%+5.1f Rate:%+6.1f Corr:%+5.0f | L:%3d R:%3d",
         ST_NAME[g_state], rangeStr(S_FRONT).c_str(), rangeStr(S_LEFT).c_str(),
         rangeStr(S_RIGHT).c_str(), frontVotes(),
         g_heading_deg, g_target_heading, g_rate_dps, g_last_corr, g_pwm_l, g_pwm_r);
}

/* ==========================================================================
 *  13. SETUP / LOOP
 * ======================================================================== */
void printResetReason(void) {
    esp_reset_reason_t r = esp_reset_reason();
    const char *s = "unknown";
    switch (r) {
        case ESP_RST_POWERON:  s = "power-on"; break;
        case ESP_RST_BROWNOUT: s = "BROWNOUT -- the supply sagged. This is a"
                                   " POWER fault: add bulk capacitance at VIN"
                                   " and check the buck converter."; break;
        case ESP_RST_PANIC:    s = "panic/exception"; break;
        case ESP_RST_TASK_WDT: s = "task watchdog"; break;
        case ESP_RST_INT_WDT:  s = "interrupt watchdog"; break;
        case ESP_RST_SW:       s = "software"; break;
        case ESP_RST_EXT:      s = "external pin"; break;
        default: break;
    }
    Serial.printf("Reset reason: %s\n", s);
}

void setup() {
    /* Boot is deliberately staggered. Bringing Serial2, the I2C sensors and
     * the Bluetooth radio up in the same instant is what puts a marginal
     * battery rail into a brownout loop (see the progress report). */
    Serial.begin(115200);
    delay(50);
    Serial.println("\n>>> BOOT <<<");
    printResetReason();
    delay(300);                       /* let the rail settle */

    /* UART first, and STOP the motors immediately in case the ATmega was
     * mid-command when we reset. */
    Serial2.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
    delay(50);
    motorStop();

    /* I2C + sensors: much lower current than the radio, so do them first. */
    i2cBusRecover();
    tofInitAll();

    Serial.println("Bringing up MPU6050...");
    mpu_ok = mpuInit();
    Serial.println(mpu_ok ? "  [OK]   MPU6050" : "  [FAIL] MPU6050 NOT FOUND");

    /* Bluetooth LAST: the classic BT stack is the single biggest current
     * spike at boot. The robot does NOT need a client to run. */
    delay(200);
    SerialBT.begin("MazeSolver_P5");
    bt_ready = true;
    delay(200);

    if (mpu_ok) {
        LOG("Calibrating gyro -- keep the robot COMPLETELY STILL...");
        gyroCalibrate(GYRO_CAL_FULL, "full cal");
    }
    headingReset();
    tofFlush();

    printMenu();
    LOG("Send START when the robot is in the maze.");
    g_run_t0 = millis();
    enterState(ST_IDLE);
}

void loop() {
    static uint32_t next_tick = 0;
    if (next_tick == 0) next_tick = millis();

    pumpCommands();
    motorHeartbeat();

    if ((int32_t)(millis() - next_tick) < 0) return;
    next_tick += CONTROL_TICK_MS;
    /* If we fell badly behind, re-base rather than firing a burst of catch-up
     * ticks with no spacing -- a burst would corrupt the gyro integration. */
    if ((int32_t)(millis() - next_tick) > (int32_t)(CONTROL_TICK_MS * 3))
        next_tick = millis() + CONTROL_TICK_MS;

    if (mpu_ok) headingSample(CONTROL_TICK_MS);
    tofTask();

    if (g_running) {
        switch (cfg_mode) {

        case 0:                                   /* sensors only, no motion */
            motorStop();
            break;

        case 1: {                                 /* straight-line test      */
            static bool begun = false, stopped = false;
            if (!begun) { driveBegin(); begun = true; }
            if (!stopped) {
                if (frontBlocked()) {
                    driveStop(); stopped = true;
                    LOG("MODE1: front obstacle -- stopped");
                } else driveTick();
            }
            if (g_stalled) { stopped = true; }
            break;
        }

        case 2: {                                 /* single 90 deg pivot     */
            TurnResult r;
            motorStop();
            delay(STARTUP_DELAY_MS);
            turn90(true, r);
            LOGf("TURN TEST: achieved %.1f deg | pre-nudge err %.1f | nudges %d%s",
                 r.achieved_deg, r.initial_err_deg, r.nudges,
                 r.timed_out ? " | TIMED OUT" : "");
            LOG("err > 0 means it undershot -> LOWER TM.  err < 0 -> RAISE TM.");
            g_running = false;
            break;
        }

        default:                                  /* full maze               */
            mazeTick();
            if (millis() - g_run_t0 > MAX_RUN_MS) {
                driveStop();
                LOG("RUN LIMIT reached -- halted");
                g_running = false;
            }
            break;
        }
    } else {
        motorStop();
    }

    telemetry();
}
