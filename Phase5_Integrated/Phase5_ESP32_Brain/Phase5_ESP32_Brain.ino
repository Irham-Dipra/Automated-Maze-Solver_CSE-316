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

/* --- maze geometry, in MILLIMETRES (VL53L0X native unit) --------------
 *
 * Three DIFFERENT lengths used to be conflated into one CORRIDOR_HALF_MM,
 * which is why the stop distance could never be made to work:
 *
 *   CW/2                  half the corridor, wall face to wall face. This is
 *                         where the AXLE must sit to pivot into a side
 *                         opening and come out centred.
 *   (CW - RW)/2           what a SIDE sensor reads when the robot is centred.
 *                         Smaller than CW/2 by half the chassis width, and it
 *                         is the correct reference for wall centring.
 *   CW/2 - AX             what the FRONT sensor reads when the axle is on
 *                         that centreline. This is the correct place to stop
 *                         before a forced turn -- and nothing else is.
 *
 * All three are derived from CW/RW/AX below, so measuring the robot and the
 * maze is enough; there is nothing left to guess. */
#define DEF_CORRIDOR_W_MM   400      /* wall face to wall face  [MEASURE]   */
#define DEF_ROBOT_W_MM      160      /* widest point of chassis [MEASURE]   */
#define DEF_ROBOT_L_MM      230      /* nose to tail            [MEASURE]   */
#define DEF_SENSOR_AXLE_MM  130      /* FRONT sensor face -> axle [MEASURE] */

/* The side sensors are their own measurement, and assuming they sit at the
 * widest point and level with the front sensor is wrong on most chassis.
 *
 *   SS  span between the LEFT and RIGHT sensor faces. Centring compares a
 *       side reading against (CW - SS)/2, so using the chassis width here
 *       instead biases the robot by however far the sensors are inset.
 *   SA  how far the SIDE sensors sit ahead of the axle. A side OPENING is
 *       spotted by a side sensor, so the approach travel is measured from
 *       that sensor -- not from the front one, which is further forward. */
#define DEF_SENSOR_SPAN_MM  105      /* left face to right face [MEASURE]   */
#define DEF_SIDE_AXLE_MM     60      /* side sensors -> axle    [MEASURE]   */

#define FRONT_STOP_FLOOR_MM  50      /* never park closer than this         */

#define TOF_MAX_RANGE_MM   1200      /* beyond this we call it "open"       */
#define TOF_TOO_CLOSE_MM     40      /* below this the sensor is unreliable */
#define TOF_STALE_MS        250
#define TOF_MAX_JUMP_MM     200      /* a wall cannot move this fast        */
#define TOF_NEAR_LATCH_MM   120      /* lost echo + last reading this close
                                        means the wall got NEARER, not gone */
#define TOF_LATCH_MS        400      /* how long that inference stays valid.
                                        MUST be finite: after a pivot the view
                                        ahead really does change from 100 mm
                                        to infinity, and an unbounded latch
                                        can never notice.                    */
#define TOF_FLUSH_DISCARD     2      /* samples binned after a flush         */

/* These three are power-on DEFAULTS for the runtime-tunable cfg_* values
 * below. FRONT_BLOCKED is the distance at which a wall ahead is BELIEVED (it
 * drives the vote window); FRONT_STOP is only how close the robot coasts in
 * APPROACH before pivoting. Tuning FRONT_STOP can never fix late detection --
 * that is what FB is for, and it is why 220 mm was not enough stopping
 * distance once the chassis was running fast. */
#define DEF_OPENING_MM        280    /* side reads beyond this => opening   */
#define DEF_FRONT_BLOCKED_MM  300
/* FD:0 means "derive it from the geometry", which is almost always right.
 * A non-zero FD overrides it, for when the chassis needs extra clearance. */
#define DEF_FRONT_STOP_MM       0

/* Side wall closer than this gets an active, ramped steer-away. Avoidance
 * used to fire only on the sub-measurable "too close" flag (40 mm), i.e. once
 * the chassis was already touching -- the 59 mm approach in the 14:18 log got
 * nothing but a 7 degree hint.
 *
 * WE:0 derives it, because the right answer is pure arithmetic. Two facts set
 * it: a centred chassis has (CW-RW)/2 of gap per side, and a side sensor
 * inset from the widest point reads (RW-SS)/2 MORE than that gap. Panic when
 * half the gap is used up, converted into what the sensor will read:
 *
 *     WE = (CW-RW)/4  +  (RW-SS)/2
 *
 * Forgetting the inset term is what makes a hand-picked WE fire late: the
 * chassis is always nearer the wall than its sensor claims. */
#define DEF_WALL_EMERG_MM       0    /* 0 = derive from the geometry        */
#define WALL_EMERG_EXTRA_DEG  12.0f  /* added to the tilt cap at contact     */

/* Ticks a side must stay open before it counts as a junction. At 2 this was
 * 40 ms: tofOpen() calls an OUT-OF-RANGE reading an opening, so two noisy
 * pings off a dark or angled wall were enough to send the robot through the
 * whole approach-park-look sequence and back out again with "no turning here
 * after all" -- the little stall in the middle of a clean corridor. Five
 * ticks is 100 ms, about 20 mm of travel: too short to miss a real opening
 * 400 mm across, long enough that noise never survives it. */
#define OPENING_CONFIRM       5
/* The FRONT keeps its own, much shorter confirm. It shares no noise mode with
 * the sides -- it is already vote-filtered over a 5-sample window -- and a
 * wall ahead is the one reading the robot cannot afford to sit on. Slowing it
 * to match the sides costs 60 ms of braking distance for nothing. */
#define FRONT_CONFIRM         2
#define DEADEND_CONFIRM       3
#define FRONT_VOTE_WINDOW     5
#define FRONT_VOTE_THRESHOLD  2

/* --- approach offset -------------------------------------------------- */
/* The ToF sees an opening when the SENSOR is level with it, but the robot
 * pivots about the axle, further back. Drive on by AX + CW/2 so the pivot
 * centre -- not the nose -- ends up in the middle of the opening. Only used
 * when there is NO front wall to range off; with a wall ahead the stop is
 * closed-loop on distance instead, which needs no speed calibration. */
#define TRAVEL_SPEED_MMS    200      /* [MEASURE] mm/s at BASE_PWM          */

/* Deceleration ramp into a junction. Approaching at full speed and then
 * cutting power means the stop distance is FD + coast, and coast grows with
 * speed -- which is why one FD was too close at speed and too far when slow.
 * Speed is now ramped down from cfg_front_blk to the stop point, so the robot
 * arrives already crawling and the stop is repeatable. */
#define JUNCTION_BRAKE_MS    90      /* active brake pulse when parking      */

/* A TIMED approach (side opening, no wall ahead) is the one manoeuvre whose
 * accuracy depends on knowing the speed. Rather than try to predict whatever
 * speed the robot happened to build up -- which really is higher after a long
 * straight than a short one -- it is forced down to a fixed crawl for the
 * whole timed run. Then SP is one number, measured once at that crawl, and it
 * does not matter how long the approach corridor was. */
#define APPROACH_CRAWL      0.25f    /* fraction of the BL-MN span           */

/* Longest the came-from block may last if that side never shows a wall (a
 * turn into a corridor that opens again immediately). Generous, but finite so
 * it can never latch on and hide a real turning forever. */
#define BLOCK_BACKSTOP_MS   4000

/* No front measurement for this long means the sensor has stopped answering.
 * Driving on is not a degraded mode, it is driving blind: frontBlocked()
 * reports false for a sensor that is not there, so the robot would never
 * believe in a wall ahead and would meet the next one at full speed. */
#define FRONT_DEAD_MS        600

/* How long to stand at the junction gathering clean readings before choosing
 * which way to go. The chassis is stationary and the side sensors are level
 * with the openings -- the best view of the junction in the whole manoeuvre. */
#define LOOK_SETTLE_MS       250
#define LOOK_TIMEOUT_MS      900

/* --- CREEP TO THE PARK POINT ------------------------------------------
 * Braking distance and parking accuracy are two different problems and were
 * being solved with one number. FB has to be generous, or a robot arriving
 * down a long straight at full speed cannot shed its momentum before the
 * wall. But the same generous FB makes a robot that is already crawling --
 * one that has just finished a pivot a few hundred mm from an obstacle --
 * begin its ramp immediately and coast to a halt far short of the junction.
 * It then pivots from too far back and clips the corner.
 *
 * So: stop wherever the braking happens to end, then close the remaining gap
 * in short measured pulses until the front sensor reads the park distance.
 * FB can stay as large as the fast case needs, because it no longer has any
 * say in where the robot finally sits. Pulse length scales with the error,
 * exactly like the turn nudges, and the gap is re-measured after every pulse.
 * It reverses too, which recovers the case where it arrived touching. */
#define CREEP_TOL_MM          15     /* close enough; stop nudging           */
#define CREEP_MAX_PULSES       6
#define CREEP_TIMEOUT_MS    4000
#define CREEP_MS_PER_MM      1.6f    /* bias short: an extra pulse is cheap  */
#define CREEP_MS_MIN          60
#define CREEP_MS_MAX         260
#define CREEP_BRAKE_MS        70
#define CREEP_SETTLE_MS      160     /* let it stop before re-measuring      */
#define DEF_CREEP_PWM        165     /* must break static friction from rest */
/* A creep pulse used to be open loop: the same PWM to both wheels, for a few
 * tens of milliseconds, with nothing watching. Two mismatched TT motors do
 * not travel straight under that, so every backward nudge added a few degrees
 * of yaw -- always the same way, so six nudges over a run compound into a
 * heading error big enough to clip the next corner. The gyro is live
 * throughout (headingSample runs from loop(), not from driveTick), so close
 * the loop: steer each pulse back onto the heading held at the stop. */
#define CREEP_KP             3.0f    /* PWM counts per degree, during a nudge */
#define CREEP_CORR_MAX        45     /* never let the differential dominate   */

/* --- motors ----------------------------------------------------------- */
/* MIN is the stall floor: below it the TT motors buzz but do not turn.
 * Your two motors are badly mismatched, so LT/RT trim each side
 * independently -- that mismatch is what made positional PID untunable. */
#define DEF_BASE_PWM        135
#define DEF_MIN_PWM          95
#define DEF_MAX_PWM         230
#define DEF_LEFT_TRIM       100      /* percent */
#define DEF_RIGHT_TRIM      120      /* percent -- right motor is the weak one */
#define KICK_PWM            200      /* breakaway pulse from standstill     */
#define KICK_MS              60

/* --- heading-hold PD -------------------------------------------------- */
/* corr > 0 steers RIGHT.  left = base + corr,  right = base - corr.
 * Correction is capped at a FRACTION OF THE BASE SPEED, so lowering the base
 * automatically softens the steering instead of sharpening it. This cap is
 * the direct fix for the Adj:826 runaway. */
#define DEF_KP              4.5f     /* PWM counts per degree of drift      */
#define DEF_KD              0.35f    /* PWM counts per deg/s of yaw rate    */
/* How hard the centring pulls. This is NOT a steering gain -- it sets a
 * heading TARGET, and the robot's sideways position is the integral of that
 * heading. So the closed loop settles with a time constant of
 *
 *     tau = 1 / (2 * v * KW_in_radians)
 *
 * At KW 0.020 deg/mm and 250 mm/s that is about SIX SECONDS: a 1.1 m corridor
 * is finished in four, so the robot never visibly re-centres and the only
 * correction you ever see is the emergency one firing at the wall. 0.060
 * brings it to ~2 s, which is quick enough to matter and still far slower
 * than the heading loop underneath it, so the two do not fight. */
#define DEF_KW              0.060f   /* deg of heading target per mm of
                                        wall-centring error                 */
#define DEF_WALL_TILT_DEG    8.0f    /* WT -- max centring tilt, degrees    */
/* Most the heading target may move in one 20 ms tick. Without this a side
 * reading that flickers between a number and out-of-range whips the target
 * across its whole range in a single tick -- the 10:35:24 log swings it from
 * +5.1 to -8.0 and back to 0 in three ticks. The heading controller then
 * chases a target that is pure noise, which reads as "KP cannot hold it
 * straight" when KP is in fact tracking faithfully. */
#define WALL_TILT_SLEW_DEG   2.0f
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
/* Spread across a calibration run, in raw LSB, above which the chassis is
 * judged to have been moving and the samples are thrown away. 250 LSB is only
 * 3.8 deg/s and was rejecting three times at every junction on a chassis that
 * had merely just braked -- so the bias was never refreshed at all. 600 LSB
 * (~9 deg/s) still catches a robot that is genuinely still rolling while
 * tolerating settle wobble. */
#define GYRO_CAL_MAX_SPREAD 600
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
    uint32_t latch_until;    /* the near-wall latch EXPIRES at this time.
                                Without a deadline the latch fed itself and
                                jammed the sensor permanently -- see tofPoll */
    uint8_t  warmup;         /* samples to discard after a flush. The sensor
                                has a measurement already in flight, taken
                                while the chassis was mid-pivot and aimed at
                                a wall 100 mm away; letting that land is what
                                armed the latch after every turn.            */
};

struct TurnResult {
    float   achieved_deg;
    float   initial_err_deg; /* + undershot, - overshot, BEFORE any nudging */
    uint8_t nudges;
    bool    timed_out;
};

enum MazeState {
    ST_IDLE, ST_STARTUP, ST_DRIVING, ST_APPROACHING, ST_CONFIRM_EXIT,
    ST_STOPPING, ST_CREEPING, ST_RECALIBRATING, ST_LOOKING, ST_DECIDING,
    ST_RECOVERING, ST_FINISHED
};
const char *ST_NAME[] = { "IDLE","STARTUP","DRIVING","APPROACH","CONFIRM_EXIT",
                          "STOPPING","CREEPING","RECAL","LOOKING","DECIDING",
                          "RECOVER","FINISHED" };

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
int   cfg_front_stop = DEF_FRONT_STOP_MM;      /* FD -- 0 means auto        */
int   cfg_corridor_w = DEF_CORRIDOR_W_MM;      /* CW -- wall face to face   */
int   cfg_robot_w    = DEF_ROBOT_W_MM;         /* RW -- widest chassis      */
int   cfg_robot_l    = DEF_ROBOT_L_MM;         /* RL -- nose to tail        */
int   cfg_axle       = DEF_SENSOR_AXLE_MM;     /* AX -- front sensor->axle  */
int   cfg_span       = DEF_SENSOR_SPAN_MM;     /* SS -- side sensor span    */
int   cfg_side_axle  = DEF_SIDE_AXLE_MM;       /* SA -- side sensor->axle   */
int   cfg_front_blk  = DEF_FRONT_BLOCKED_MM;   /* FB -- "wall ahead" belief  */
int   cfg_opening    = DEF_OPENING_MM;         /* OP -- "side is open"       */
int   cfg_wall_emerg = DEF_WALL_EMERG_MM;      /* WE -- hard steer-away      */
int   cfg_speed      = TRAVEL_SPEED_MMS;       /* SP -- measured mm/s        */
int   cfg_creep_pwm  = DEF_CREEP_PWM;          /* CP -- park-nudge power     */
float cfg_kp         = DEF_KP;
float cfg_kd         = DEF_KD;
float cfg_kw         = DEF_KW;
float cfg_wall_tilt  = DEF_WALL_TILT_DEG;      /* WT -- centring tilt cap   */
float cfg_margin     = DEF_STOP_MARGIN_DEG;
int   cfg_gyro_sign  = 1;            /* flip with GS:-1 if mounted inverted */
int   cfg_mode       = 3;            /* 0 telemetry 1 straight 2 turn 3 maze */
bool  cfg_log        = true;         /* LOG:0 silences the telemetry stream */

bool  g_running      = false;        /* START/STOP from the phone           */

/* STOP must work even in the middle of a pivot or a gyro calibration, which
 * block for many seconds. Those routines poll g_abort and bail out, and they
 * call serviceWhileBusy() so the command is actually READ while they run --
 * the original code only heartbeated the motors in those loops, so a STOP
 * sent during a turn was not seen until the turn had already finished. */
volatile bool g_abort = false;

/* Telemetry is held off for a moment after the menu prints, so the menu is
 * still readable instead of being buried by the next telemetry line. */
uint32_t g_quiet_until = 0;
#define MENU_QUIET_MS 4000

/* MODE 1 run state. These were function-level statics, which meant they were
 * never reset: the first START worked and every START after a STOP silently
 * did nothing. */
bool  m1_begun   = false;
bool  m1_stopped = false;

/* Forward declarations -- the blocking routines in sections 6 and 9 need to
 * pump the command parser, which is defined later in section 11. */
void pumpCommands(void);
void printMenu(void);
void driveStop(void);
String rangeStr(int id);   /* telemetry formatter, used by the junction log */

/* Called from inside every long-running blocking loop. Keeps the slave's
 * failsafe fed AND keeps STOP responsive. */
void serviceWhileBusy(void);

/* ---- derived geometry ---------------------------------------------------
 * Measure CW, RW and AX and these three fall out. They are the only lengths
 * the navigation actually needs, and deriving them means they cannot drift
 * out of agreement with one another. */

/* What a SIDE sensor reads with the robot centred -- the centring reference.
 * Built from the SENSOR SPAN, not the chassis width: sensors inset from the
 * widest point read further than the chassis gap, and using the chassis width
 * here makes a centred robot look off-centre by exactly that inset. */
int wallRefMm(void) {
    int v = (cfg_corridor_w - cfg_span) / 2;
    return (v < 10) ? 10 : v;
}

/* How far the AXLE must travel past a side opening's near edge to sit on its
 * centreline. Measured from the SIDE sensor, because a side sensor is what
 * spots the opening -- using the front sensor's offset overshoots by the gap
 * between them. Used only when there is no front wall to range off. */
int approachMm(void) { return cfg_side_axle + cfg_corridor_w / 2; }

/* The largest a SIDE sensor can read and still be looking at a corridor wall:
 * chassis hard against the opposite wall, so the gap is (CW - RW), plus the
 * distance the sensor sits inside the widest point, (RW - SS)/2. Anything
 * beyond this is not a wall to centre against -- it is the mouth of an
 * opening, and steering to hold a fixed distance from it walks the robot into
 * the wall behind. */
int maxWallMm(void) {
    int v = cfg_corridor_w - cfg_robot_w / 2 - cfg_span / 2;
    return (v < 20) ? 20 : v;
}

/* Radius of the circle the furthest chassis corner sweeps while pivoting.
 * Checked against the half-corridor so an impossible geometry is reported at
 * the menu instead of discovered by watching the robot grind into a corner. */
float sweptRadiusMm(void) {
    float hw = cfg_robot_w / 2.0f;
    float front = sqrtf((float)cfg_axle * cfg_axle + hw * hw);
    int   tail  = cfg_robot_l - cfg_axle;
    if (tail < 0) tail = 0;
    float rear  = sqrtf((float)tail * tail + hw * hw);
    return (front > rear) ? front : rear;
}

/* Where the FRONT sensor should read when the axle is on the perpendicular
 * corridor's centreline -- exactly where to park before a forced turn.
 * Stopping further back than this leaves the axle short of the opening, so
 * the pivot swings the chassis into the corner instead of into the corridor. */
int frontStopMm(void) {
    if (cfg_front_stop > 0) return cfg_front_stop;      /* manual override */
    int v = cfg_corridor_w / 2 - cfg_axle;
    return (v < FRONT_STOP_FLOOR_MM) ? FRONT_STOP_FLOOR_MM : v;
}

/* Side clearance, as a SENSOR READING, at which to steer away hard. See
 * DEF_WALL_EMERG_MM for the derivation. */
int wallEmergMm(void) {
    if (cfg_wall_emerg > 0) return cfg_wall_emerg;      /* manual override */
    int gap   = (cfg_corridor_w - cfg_robot_w) / 2;     /* chassis gap when centred */
    int inset = (cfg_robot_w - cfg_span) / 2;           /* sensor sits inside that  */
    if (inset < 0) inset = 0;
    int v = gap / 2 + inset;
    return (v < 30) ? 30 : v;
}

/* Time-based fallback for a junction with no wall ahead to measure against. */
uint32_t approachMs(void) {
    int sp = (cfg_speed > 10) ? cfg_speed : 10;
    return (uint32_t)approachMm() * 1000UL / (uint32_t)sp;
}

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

/* The single thing every blocking wait must call. Feeds the slave failsafe
 * and reads incoming commands, so STOP is honoured mid-pivot instead of
 * queueing up behind it. */
void serviceWhileBusy(void) {
    motorHeartbeat();
    pumpCommands();
}

void motorStop(void)                      { motorSend('S', 0, 0); }
void motorBrake(void)                     { motorSend('K', 0, 0); }
void motorForward(int l, int r)           { motorSend('F', l, r); }
/* clockwise != 0 pivots the chassis to the RIGHT */
/* A pivot must apply LT/RT exactly like driving does. Sending the same raw
 * PWM to both wheels looks symmetric but is not: with one motor weaker, the
 * forward wheel out-pushes the reverse wheel, so the chassis rotates AND
 * creeps forward instead of turning on the spot. Parked 65 mm off a wall,
 * that creep is enough to jam the nose into it partway through the turn. */
void motorPivot(bool clockwise, int pwm) {
    int l = (pwm * cfg_left_trim)  / 100;
    int r = (pwm * cfg_right_trim) / 100;
    if (l > cfg_max_pwm) l = cfg_max_pwm;
    if (r > cfg_max_pwm) r = cfg_max_pwm;
    motorSend(clockwise ? 'R' : 'L', l, r);
}

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
        if (g_abort) return false;          /* STOP pressed mid-calibration */
        if (!mpuReadAll(g)) return false;
        sx += g.x; sy += g.y; sz += g.z;
        if (g.z < zmin) zmin = g.z;
        if (g.z > zmax) zmax = g.z;
        delay(GYRO_CAL_INTERVAL_MS);
        serviceWhileBusy();
    }
    if ((long)zmax - (long)zmin > GYRO_CAL_MAX_SPREAD) return false;

    g_off_z = (float)sz / samples;
    g_off_x = (float)sx / samples;
    g_off_y = (float)sy / samples;
    return true;
}

bool gyroCalibrate(int samples, const char *what) {
    /* HOLD the brake rather than coasting. Releasing lets each wheel free-
     * wheel at its own rate, and the two are not equal -- RT:120 exists
     * precisely because the right motor has more drag. The left therefore
     * rolls a little further and the chassis yaws right just as we try to
     * measure "stationary", which is a good part of why this used to reject
     * three times at every junction. A shorted-motor brake at full duty is a
     * static output with no switching and, once stopped, no current: it is
     * quieter than coasting, not noisier. */
    motorBrake();
    delay(GYRO_SETTLE_MS);
    for (int t = 0; t < GYRO_CAL_RETRIES; t++) {
        if (g_abort) { LOG("calibration aborted"); return false; }
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

    /* Both lines should now be pulled high. If one is still low the bus is
     * physically held down and no amount of retrying will help -- which is
     * worth SAYING, because "all three sensors failed" otherwise looks
     * identical to "nothing is powered". SDA stuck low is usually a device
     * (often the MPU6050, which XSHUT cannot reset) caught mid-byte by a
     * reset. SCL stuck low, or both, is normally wiring or a dead rail. */
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
    delayMicroseconds(50);
    {
        int sda = digitalRead(PIN_SDA), scl = digitalRead(PIN_SCL);
        if (!sda || !scl)
            LOGf("  I2C BUS STUCK: SDA=%s SCL=%s -- a device is holding the "
                 "bus down, or the 3.3 V rail is not up",
                 sda ? "high" : "LOW", scl ? "high" : "LOW");
    }

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
    /* Stamp it now: a flush means "no history", not "sensor stopped
     * answering". Leaving this at 0 would make the liveness check below
     * declare the sensor dead for a moment after every pivot. */
    st[id].stamp = millis();
    st[id].latch_until = 0;
    st[id].warmup = TOF_FLUSH_DISCARD;
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

    /* Every single sensor failing is a different fault from one failing: it
     * points at the shared resources -- the 3.3 V rail or the bus -- not at
     * any one module. Those are exactly the faults a bus recovery sometimes
     * clears, so it is worth a second full attempt before giving up. */
    if (!st[S_FRONT].present && !st[S_LEFT].present && !st[S_RIGHT].present) {
        LOG("  ALL sensors failed -- shared fault (rail or bus). Retrying...");
        i2cBusRecover();
        delay(200);
        for (int i = 0; i < S_COUNT; i++) {
            digitalWrite(XSHUT[i], LOW);
        }
        delay(100);
        for (int i = 0; i < S_COUNT; i++) {
            digitalWrite(XSHUT[i], HIGH);
            delay(120);
            st[i].present = tofInitOne(i);
            delay(50);
        }
    }

    LOGf("sensors: FRONT %s  LEFT %s  RIGHT %s",
         st[S_FRONT].present ? "ok" : "MISSING",
         st[S_LEFT].present  ? "ok" : "MISSING",
         st[S_RIGHT].present ? "ok" : "MISSING");
    if (!st[S_FRONT].present)
        LOG("  no FRONT sensor: MODE 3 will refuse to start. Send RESCAN to "
            "retry without rebooting.");
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

    /* Throw away the first samples after a flush: they were captured while
     * the robot was still pivoting and describe a wall it is no longer
     * pointing at. */
    if (p.warmup) { p.warmup--; return; }

    /* Record the genuine measurement BEFORE any interpretation. last_good
     * must only ever hold something the sensor actually measured. The old
     * code wrote the synthetic TOF_TOO_CLOSE_MM back into last_good, which
     * made the latch below self-sustaining: once set, last_good was 40, 40 is
     * under TOF_NEAR_LATCH_MM, so every subsequent out-of-range reading
     * re-armed the latch and wrote 40 again. In an open corridor (where
     * out-of-range is the normal reading) the front sensor then reported
     * "wall touching me" forever and the robot turned at every junction.    */
    bool lost_echo = (v >= TOF_MAX_RANGE_MM);
    if (!lost_echo) { p.last_good = v; p.has_good = true; }

    /* Disambiguate a lost echo. Very near or very dark surfaces give the same
     * "out of range" status as open space. History resolves it: a wall that
     * was 8 cm away 60 ms ago did not vanish, it got closer.
     *
     * But that inference is only valid for a short while. It is evidence with
     * a shelf life, so it now carries an explicit deadline -- after a pivot
     * the view ahead legitimately changes from 100 mm to infinity, and
     * without the deadline the robot could never notice.                     */
    bool measured_near = (v <= TOF_TOO_CLOSE_MM);
    if (measured_near) p.latch_until = millis() + TOF_LATCH_MS;

    bool latch_live = ((int32_t)(millis() - p.latch_until) < 0);
    bool inferred_near = lost_echo && p.has_good &&
                         p.last_good <= TOF_NEAR_LATCH_MM && latch_live;

    if (measured_near || inferred_near) {
        v = TOF_TOO_CLOSE_MM;          /* a real number to steer away from */
        p.too_close = true;
    } else {
        p.too_close = false;
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
                        : ((!jump && v < cfg_front_blk) ? 1 : 0);
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
    /* No data yet (just flushed, still warming up) is NOT evidence of an
     * opening. tofClear() parks `latest` at TOF_MAX_RANGE_MM, so without this
     * guard a freshly flushed sensor claims "open" and three flushed sensors
     * look exactly like the maze exit. */
    if (st[id].n == 0) return false;
    if (st[id].too_close) return false;    /* checked FIRST -- see above    */
    uint16_t v = tofLatest(id);
    if (v >= TOF_MAX_RANGE_MM) return true;
    return v > cfg_opening;
}

bool tofTooClose(int id) { return st[id].present && st[id].too_close; }

/* Is the front sensor still answering at all? Distinct from tofValid(), which
 * is false for a perfectly healthy sensor staring down an empty corridor. */
bool frontAlive(void) {
    if (!st[S_FRONT].present) return false;
    return (millis() - st[S_FRONT].stamp) < FRONT_DEAD_MS;
}

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
/* 1.0 = full BL. Ramped down while closing on a junction so the robot
 * arrives already crawling; at 0 the base falls to MN, the slowest speed the
 * wheels still turn at. This is what makes the stop repeatable instead of
 * "FD plus however far it coasted at whatever speed it happened to be". */
float g_speed_scale = 1.0f;
float g_wall_err    = 0;       /* last centring error, mm (+ = room on right) */
char  g_wall_src    = '-';     /* which walls produced it: B / L / R / -      */
float g_last_corr = 0;

/* stall watchdog */
uint32_t g_stall_t0 = 0;
float    g_stall_hdg0 = 0;
uint16_t g_stall_front0 = 0;
bool     g_stalled = false;

void driveBegin(void) {
    g_speed_scale = 1.0f;
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
    while (millis() - t0 < KICK_MS && !g_abort) { serviceWhileBusy(); }
    if (g_abort) motorStop();
}

/* Park at a junction: brake rather than coast. The ATmega has supported the
 * 'K' active-brake command all along (both H-bridge inputs high, shorting the
 * motor terminals so back-EMF stops the rotor) but nothing ever sent it, so
 * every stop was a coast of speed-dependent length. */
void driveStopHard(void) {
    motorBrake();
    g_pwm_l = g_pwm_r = 0;
    g_last_corr = 0;
    g_speed_scale = 1.0f;
}

void driveStop(void) {
    motorStop();
    g_speed_scale = 1.0f;
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
    int  l_mm = tofMedian(S_LEFT);
    int  r_mm = tofMedian(S_RIGHT);
    /* tofValid() judges p.latest, but the value used below is the MEDIAN, and
     * the two disagree whenever the newest sample is in range while the
     * median still holds the out-of-range clamp. The side then counts as
     * usable while feeding TOF_MAX_RANGE_MM into the centring maths, which
     * slams the target straight to its clamp -- the Tgt:-8.0 alongside R:oo
     * at 10:35:24.871. Judge and use the same number. */
    bool l_ok = tofValid(S_LEFT)  && l_mm < TOF_MAX_RANGE_MM;
    bool r_ok = tofValid(S_RIGHT) && r_mm < TOF_MAX_RANGE_MM;

    /* --- wall-centring, expressed as a heading TARGET ------------------- */
    /* With BOTH walls the error is a difference, so it stays meaningful
     * however wide the corridor. With only one, it is measured against a
     * fixed reference -- which is only valid if that reading really is a
     * corridor wall. A left sensor reading 275 against a 147 reference is the
     * edge of an opening, and holding station off it steers the robot into
     * whatever is on the other side. */
    float wall_err_mm = 0;
    /* 'B'oth walls, 'L'eft only, 'R'ight only, '-' nothing to centre against.
     * Printed in the telemetry, because "the centring is not working" and
     * "the centring has no walls to work with" look identical from outside
     * and need completely different fixes. */
    char  src = '-';
    if (l_ok && r_ok)                     { wall_err_mm = (float)(r_mm - l_mm);                      src = 'B'; }
    else if (l_ok && l_mm <= maxWallMm()) { wall_err_mm = (float)(wallRefMm() - l_mm) * 2.0f;        src = 'L'; }
    else if (r_ok && r_mm <= maxWallMm()) { wall_err_mm = (float)(r_mm - wallRefMm()) * 2.0f;        src = 'R'; }
    g_wall_err = wall_err_mm;
    g_wall_src = src;

    /* more room on the right -> aim a few degrees right (negative heading) */
    float want = -cfg_kw * wall_err_mm;
    if (want >  cfg_wall_tilt) want =  cfg_wall_tilt;
    if (want < -cfg_wall_tilt) want = -cfg_wall_tilt;
    {   /* rate-limit it, so a flickering sensor cannot whip the target */
        float d = want - g_target_heading;
        if (d >  WALL_TILT_SLEW_DEG) d =  WALL_TILT_SLEW_DEG;
        if (d < -WALL_TILT_SLEW_DEG) d = -WALL_TILT_SLEW_DEG;
        g_target_heading += d;
    }

    /* --- EMERGENCY: wall closer than the gentle heading bias can handle ----
     * Fires on the sub-measurable "too close" flag OR on a valid reading
     * inside cfg_wall_emerg. Checking only the former (as this did) meant no
     * avoidance until the wall was under 40 mm -- by which point the chassis
     * is already touching it.
     *
     * Severity RAMPS with proximity instead of stepping to full authority at
     * the threshold: mahee's config.h records that a hard step measured 71
     * deg/s of yaw and bounced the robot off one wall straight into the
     * other. */
    int  we = wallEmergMm();
    bool l_near = tofTooClose(S_LEFT)  || (l_ok && l_mm <= we);
    bool r_near = tofTooClose(S_RIGHT) || (r_ok && r_mm <= we);

    if (l_near != r_near) {                 /* pinned on exactly one side */
        int  d   = l_near ? (tofTooClose(S_LEFT)  ? 0 : l_mm)
                          : (tofTooClose(S_RIGHT) ? 0 : r_mm);
        float sev = 1.0f - (float)d / (float)(we > 0 ? we : 1);
        if (sev < 0.0f) sev = 0.0f;
        if (sev > 1.0f) sev = 1.0f;
        float away = cfg_wall_tilt + WALL_EMERG_EXTRA_DEG * sev;
        g_target_heading = l_near ? -away : away;   /* steer off that wall */
    }

    /* --- PD ------------------------------------------------------------ */
    float drift = g_heading_deg - g_target_heading;   /* + = drifted left   */
    float corr  = cfg_kp * drift + cfg_kd * g_rate_dps;

    /* YAW GOVERNOR. Past this rate the ToF sensors point far enough off-axis
     * that they stop describing the corridor -- the front beam in particular
     * walks off whatever is ahead. Stop ADDING rotation in that direction;
     * do not reverse, just let it decay. */
    if (corr > 0 && g_rate_dps < -YAW_GOVERNOR_DPS) corr = 0;
    if (corr < 0 && g_rate_dps >  YAW_GOVERNOR_DPS) corr = 0;

    /* Effective base speed for this tick, after any junction deceleration. */
    int base = cfg_min_pwm + (int)((cfg_base_pwm - cfg_min_pwm) * g_speed_scale);
    if (base < cfg_min_pwm)   base = cfg_min_pwm;
    if (base > cfg_base_pwm)  base = cfg_base_pwm;

    /* THE CLAMP. This one line is what prevents Adj:826 / PWM 0-vs-255. */
    float cap = (base * CORR_RATIO_PCT) / 100.0f;
    if (corr >  cap) corr =  cap;
    if (corr < -cap) corr = -cap;
    g_last_corr = corr;

    int l = base + (int)corr;
    int r = base - (int)corr;

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
    while ((int32_t)(millis() - next_ms) < 0 && !g_abort) { serviceWhileBusy(); }
    headingSample(TURN_TICK_MS);
    next_ms += TURN_TICK_MS;
}

/* Drive the pivot for a fixed time WHILE STILL INTEGRATING, so no rotation
 * happens uncounted. Used for the kick, the brake and each nudge. */
static void pulseTracked(bool cw, int pwm, uint16_t ms, uint32_t &next_ms) {
    uint32_t t0 = millis();
    if (g_abort) return;
    motorPivot(cw, pwm);
    while (millis() - t0 < ms && !g_abort) turnSampleUntil(next_ms);
    if (g_abort) motorStop();
}

/* Motors off, still integrating: the chassis coasts after power is cut and
 * that coast is real rotation. Not counting it is why timed turns overshoot. */
static void settleTracked(uint16_t ms, uint32_t &next_ms) {
    uint32_t t0 = millis();
    motorStop();
    while (millis() - t0 < ms && !g_abort) turnSampleUntil(next_ms);
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
        if (g_abort) break;
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
        if (g_abort) break;
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

    if (g_abort) {           /* STOP during the pivot: do not start a recal */
        LOG("pivot aborted by STOP");
        tofFlush();
        headingReset();
        return;
    }

    /* The chassis is stationary here -- the only moment a valid bias
     * measurement is possible. And everything in the ToF filters was measured
     * pointing a different direction, so throw it away. */
    gyroCalibrate(GYRO_CAL_QUICK, "recal");
    tofFlush();
    headingReset();
}

void turn90(bool right, TurnResult &res) { turnExecute(90.0f, right, res); }

/* Which way to swing a U-turn. Both sides are walls at a dead end, but they
 * are rarely the SAME distance away -- a robot parked off-centre has more
 * room on one side, and sweeping into the tighter side is what grinds the
 * chassis corner along the wall. Pick the roomier side and the swept corner
 * has somewhere to go. A side that is too close to measure counts as zero. */
bool uturnGoesRight(void) {
    int l = tofTooClose(S_LEFT)  ? 0 : (int)tofMedian(S_LEFT);
    int r = tofTooClose(S_RIGHT) ? 0 : (int)tofMedian(S_RIGHT);
    if (!st[S_LEFT].present  || st[S_LEFT].n  == 0) l = -1;
    if (!st[S_RIGHT].present || st[S_RIGHT].n == 0) r = -1;
    /* No usable pair to compare -> keep the old right-hand default. */
    if (l < 0 || r < 0) return true;
    return r >= l;
}

void turn180(TurnResult &res, bool right) {
#if TURN_180_AS_TWO_90S
    /* Two 90s with a settle between beats one long sweep: momentum has less
     * time to build, so there is less coast to correct for. */
    TurnResult a, b;
    memset(&b, 0, sizeof(b));
    turnExecute(90.0f, right, a);
    delay(200);
    if (!g_abort) turnExecute(90.0f, right, b);
    res.achieved_deg    = a.achieved_deg + b.achieved_deg;
    res.initial_err_deg = a.initial_err_deg + b.initial_err_deg;
    res.nudges          = a.nudges + b.nudges;
    res.timed_out       = a.timed_out || b.timed_out;
#else
    turnExecute(180.0f, right, res);
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

/* --- "do not turn back the way you came" -------------------------------
 * After a right turn the corridor you arrived from lies to your RIGHT; after
 * a left turn it lies to your LEFT. The right-hand rule sees that perfectly
 * genuine opening and turns straight back into it, which is the repeated
 * double-turn.
 *
 * RECOVER_MS alone cannot fix this: it is a race between a fixed 400 ms and
 * however long the robot needs to clear an opening a whole corridor wide, and
 * at these speeds it loses about half the time. Lengthening it just re-runs
 * the same race at a different speed.
 *
 * Instead, ignore openings on that side until the side reports a solid WALL
 * again -- the new corridor's wall coming alongside is proof the junction is
 * physically behind us. That is self-timing: no dependence on SP, battery
 * charge or corridor width. */
int      s_blocked_side  = -1;      /* S_LEFT / S_RIGHT, or -1 for none     */
uint32_t s_block_expires = 0;       /* backstop, in case no wall ever shows */
uint8_t  s_block_wall_hits = 0;     /* consecutive valid wall readings      */

/* creep-to-park cycle state */
uint8_t  s_creep_pulses = 0;
bool     s_creep_driving = false;
uint32_t s_creep_until = 0;

#define BLOCK_WALL_CONFIRM  2       /* wall sightings needed to release     */
bool      pend_right = true;
bool      pend_180 = false;

char      g_path[128];
int       g_path_len = 0;

void enterState(MazeState s) {
    g_state = s;
    g_state_t0 = millis();
    g_recover_begun = false;
    if (s == ST_CREEPING) {
        s_creep_pulses  = 0;
        s_creep_driving = false;
        s_creep_until   = 0;
    }
}
bool inStateFor(uint32_t ms) { return millis() - g_state_t0 >= ms; }

void recordTurn(char c) {
    if (g_path_len < (int)sizeof(g_path) - 1) g_path[g_path_len++] = c;
    g_path[g_path_len] = 0;
    LOGf("PATH: %s", g_path);
}

/* Arm the block. Called right after a 90 degree pivot completes. A 180 needs
 * no block: at a dead end, going back the way you came IS the correct move. */
void blockCameFrom(bool turned_right) {
    s_blocked_side    = turned_right ? S_RIGHT : S_LEFT;
    s_block_expires   = millis() + BLOCK_BACKSTOP_MS;
    s_block_wall_hits = 0;
    LOGf("ignoring %s openings until a wall appears there (came from)",
         turned_right ? "RIGHT" : "LEFT");
}

/* Release once that side genuinely sees a wall, or when the backstop runs
 * out. The wall sighting must come from VALID data: the ToF history is wiped
 * after every pivot, and an empty filter reads as "not open", which would
 * clear the block instantly and achieve nothing. */
void updateCameFromBlock(void) {
    if (s_blocked_side < 0) return;

    if ((int32_t)(millis() - s_block_expires) >= 0) {
        LOG("came-from block timed out, openings live again");
        s_blocked_side = -1;
        return;
    }
    if (tofValid(s_blocked_side) && !tofOpen(s_blocked_side)) {
        if (++s_block_wall_hits >= BLOCK_WALL_CONFIRM) {
            LOG("wall alongside -- junction cleared, openings live again");
            s_blocked_side = -1;
        }
    } else {
        s_block_wall_hits = 0;
    }
}

void updateDebounce(void) {
    updateCameFromBlock();

    /* Decide ONCE what counts as open, and use that everywhere below.
     *
     * These used to disagree: the counters had the came-from block applied
     * but the dead-end test called tofOpen() directly. So after a right turn,
     * with a wall ahead and a wall on the left, the right still read open to
     * the dead-end test (killing cnt_deadend) while the counters saw it as
     * closed. classify() then matched none of its cases and fell through to
     * J_CORRIDOR -- "nothing here, carry on" -- with a wall dead ahead. The
     * 10:35:16 log is exactly this: F goes 320, 280, 238, 191, 138, 92, 45
     * with fv:5 the whole way, and the state never leaves DRIVING. */
    bool l_open = tofOpen(S_LEFT)  && (s_blocked_side != S_LEFT);
    bool r_open = tofOpen(S_RIGHT) && (s_blocked_side != S_RIGHT);

    if (l_open) { if (cnt_open_l  < 255) cnt_open_l++;  } else cnt_open_l  = 0;
    if (r_open) { if (cnt_open_r  < 255) cnt_open_r++;  } else cnt_open_r  = 0;
    if (frontBlocked()) { if (cnt_block_f < 255) cnt_block_f++; } else cnt_block_f = 0;

    if (frontBlocked() && !l_open && !r_open) {
        if (cnt_deadend < 255) cnt_deadend++;
    } else cnt_deadend = 0;
}

Junction classify(void) {
    bool lo = cnt_open_l  >= OPENING_CONFIRM;
    bool ro = cnt_open_r  >= OPENING_CONFIRM;
    bool fw = cnt_block_f >= FRONT_CONFIRM;

    if (cnt_deadend >= DEADEND_CONFIRM)  return J_DEAD_END;
    if (!fw &&  lo &&  ro)               return J_ALL_OPEN;
    if (!fw &&  lo && !ro)               return J_FWD_OR_LEFT;
    if (!fw && !lo &&  ro)               return J_FWD_OR_RIGHT;
    if ( fw &&  lo &&  ro)               return J_LEFT_OR_RIGHT;
    if ( fw &&  lo && !ro)               return J_FORCED_LEFT;
    if ( fw && !lo &&  ro)               return J_FORCED_RIGHT;

    /* A wall ahead with nowhere to turn is a dead end, never a clear
     * corridor. Falling through to J_CORRIDOR here is what let the robot
     * drive into a wall at full speed while the front sensor was reporting
     * it perfectly well. Whatever else is uncertain, never answer "carry
     * straight on" while the front is blocked. */
    if (fw) return J_DEAD_END;
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
        /* Front sensor stopped answering mid-run: stop NOW. Carrying on means
         * meeting the next wall at full speed with nothing to see it. */
        if (!frontAlive()) {
            driveStopHard();
            LOG("");
            LOG("*** FRONT SENSOR LOST -- stopping ***");
            LOG("  No front measurement for 600 ms. The robot cannot see walls");
            LOG("  ahead, so it will not keep driving. Likely a loose XSHUT or");
            LOG("  power connection shaken out by vibration.");
            g_running = false;
            enterState(ST_IDLE);
            break;
        }
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
                /* Provisional only -- ST_LOOKING re-decides at the junction
                 * itself, where the view is actually good. */
                pend_right = right;
                pend_180 = (j == J_DEAD_END);
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

    case ST_APPROACHING: {
        /* Two different junctions, two different ways to know you have
         * arrived -- conflating them is what made FD impossible to tune.
         *
         *   Wall ahead  -> the ToF gives an ABSOLUTE distance, so close the
         *                  loop on it: ramp the speed down and park at
         *                  frontStopMm(). Needs no speed calibration and is
         *                  therefore repeatable.
         *   No wall     -> nothing to range against, so fall back to the
         *                  timer, which does depend on SP being measured. */
        int stop_mm = frontStopMm();
        bool have_wall = tofValid(S_FRONT);
        int  f_mm = tofLatest(S_FRONT);

        if (have_wall) {
            if (f_mm <= stop_mm) {
                LOGf("braked at %d mm (target %d) -- creeping to close the gap",
                     f_mm, stop_mm);
                driveStopHard();
                enterState(ST_STOPPING);
                break;
            }
            /* Linear ramp: full speed at the detection distance, crawling by
             * the time it reaches the stop point. */
            int span = cfg_front_blk - stop_mm;
            float sc = (span > 1) ? (float)(f_mm - stop_mm) / (float)span : 1.0f;
            if (sc < 0.0f) sc = 0.0f;
            if (sc > 1.0f) sc = 1.0f;
            g_speed_scale = sc;
        } else {
            /* No wall to range off: crawl, so the timed distance is governed
             * by a speed we imposed rather than one we have to guess. */
            g_speed_scale = APPROACH_CRAWL;
        }

        driveTick();
        if (g_stalled) { enterState(ST_IDLE); break; }

        /* The timer is now only a fallback, and a backstop if the wall is
         * never seen at all. */
        if (!have_wall && inStateFor(approachMs())) {
            driveStopHard();
            enterState(ST_STOPPING);
        }
        break;
    }

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
        /* Hold the brake for the whole settle. It used to release after 90 ms
         * and coast, which is what produced the little yaw to the right every
         * time the robot stopped. */
        motorBrake();
        if (inStateFor(GYRO_SETTLE_MS)) enterState(ST_CREEPING);
        break;

    /* ------------------------------------------------------------------
     * CREEP: close the last few centimetres under measurement.
     * Runs as a small cycle -- pulse, brake, settle, re-measure -- driven by
     * the state timer so the control loop, telemetry and STOP all keep
     * working underneath it.
     * ------------------------------------------------------------------ */
    case ST_CREEPING: {
        int target = frontStopMm();

        /* No usable front reading: nothing to close the loop on, so accept
         * wherever the braking left us rather than nudging blind. */
        if (!tofValid(S_FRONT)) {
            if (s_creep_pulses == 0) LOG("creep skipped: no front reading");
            motorBrake();
            enterState(ST_RECALIBRATING);
            break;
        }

        int err = (int)tofLatest(S_FRONT) - target;   /* + = still too far */

        /* Mid-pulse or mid-settle: leave the motors alone until it expires. */
        if ((int32_t)(millis() - s_creep_until) < 0) break;

        if (s_creep_driving) {            /* pulse just ended -> brake+settle */
            motorBrake();
            s_creep_driving = false;
            s_creep_until   = millis() + CREEP_BRAKE_MS + CREEP_SETTLE_MS;
            break;
        }

        if (abs(err) <= CREEP_TOL_MM ||
            s_creep_pulses >= CREEP_MAX_PULSES ||
            (millis() - g_state_t0) > CREEP_TIMEOUT_MS) {
            motorBrake();
            LOGf("parked %d mm off the wall (target %d) after %d nudge%s",
                 (int)tofLatest(S_FRONT), target, s_creep_pulses,
                 s_creep_pulses == 1 ? "" : "s");
            enterState(ST_RECALIBRATING);
            break;
        }

        {
            int ms = (int)(abs(err) * CREEP_MS_PER_MM);
            if (ms < CREEP_MS_MIN) ms = CREEP_MS_MIN;
            if (ms > CREEP_MS_MAX) ms = CREEP_MS_MAX;

            bool fwd = (err > 0);

            /* Steer the nudge straight. corr > 0 means "go right" -- but that
             * is only true driving forward. Reversing the wheels reverses
             * which way a given differential swings the nose, so the sign has
             * to flip or the correction doubles the error instead of removing
             * it. */
            float drift = g_heading_deg - g_target_heading;
            int   corr  = (int)(CREEP_KP * drift);
            if (corr >  CREEP_CORR_MAX) corr =  CREEP_CORR_MAX;
            if (corr < -CREEP_CORR_MAX) corr = -CREEP_CORR_MAX;
            if (!fwd) corr = -corr;

            int l = cfg_creep_pwm + corr;
            int r = cfg_creep_pwm - corr;
            /* Lift both rather than clip one, so the differential survives. */
            if (l < cfg_min_pwm) { r += (cfg_min_pwm - l); l = cfg_min_pwm; }
            if (r < cfg_min_pwm) { l += (cfg_min_pwm - r); r = cfg_min_pwm; }
            l = (l * cfg_left_trim)  / 100;
            r = (r * cfg_right_trim) / 100;
            if (l > cfg_max_pwm) l = cfg_max_pwm;
            if (r > cfg_max_pwm) r = cfg_max_pwm;
            motorSend(fwd ? 'F' : 'B', l, r);
            LOGf("  nudge %d: %s %d ms  err %+d mm  hdg %+.1f  corr %+d",
                     s_creep_pulses + 1, fwd ? "fwd" : "back", ms, err,
                     drift, corr);
            s_creep_driving = true;
            s_creep_until   = millis() + ms;
            s_creep_pulses++;
        }
        break;
    }

    case ST_RECALIBRATING:
        gyroCalibrate(GYRO_CAL_QUICK, "recal");
        headingReset();
        tofFlush();          /* everything measured while moving is history */
        enterState(ST_LOOKING);
        break;

    /* ------------------------------------------------------------------
     * LOOK, THEN CHOOSE.
     *
     * The direction used to be chosen back in DRIVING, the instant a
     * junction was first suspected -- up to FB millimetres away, at speed,
     * with the side sensors still pointed at corridor walls. In the 21:19 log
     * that meant deciding while the left had opened and the right had not yet
     * come into view: "forced left" at a junction that was actually open both
     * ways, and the robot turned the wrong way.
     *
     * By here the chassis is parked with its axle on the junction centreline
     * and both side sensors are looking straight down their openings. Costs
     * no extra stop: under the right-hand rule every outcome except "carry
     * straight on" required stopping anyway, and "carry straight on" never
     * reaches this state.
     * ------------------------------------------------------------------ */
    case ST_LOOKING: {
        motorBrake();          /* stay held; do not coast out of position */

        /* Let the flushed filters refill before believing anything. */
        bool ready = tofValid(S_LEFT) || tofOpen(S_LEFT);
        ready = ready && (tofValid(S_RIGHT) || tofOpen(S_RIGHT));
        if (!inStateFor(LOOK_SETTLE_MS)) break;
        if (!ready && !inStateFor(LOOK_TIMEOUT_MS)) break;

        updateCameFromBlock();

        bool right_open = tofOpen(S_RIGHT) && (s_blocked_side != S_RIGHT);
        bool left_open  = tofOpen(S_LEFT)  && (s_blocked_side != S_LEFT);
        bool front_wall = frontBlocked();

        LOGf("at junction: F:%s L:%s R:%s -> %s",
             rangeStr(S_FRONT).c_str(), rangeStr(S_LEFT).c_str(),
             rangeStr(S_RIGHT).c_str(),
             right_open ? "RIGHT" : (!front_wall ? "STRAIGHT"
                        : (left_open ? "LEFT" : "U-TURN")));

        /* Right-hand rule, in priority order. */
        if (right_open)        { pend_180 = false; pend_right = true;  }
        else if (!front_wall)  {
            /* The opening we came to take is not there after all. Carrying
             * on beats pivoting into a wall on stale information. */
            LOG("no turning here after all -- carrying straight on");
            cnt_open_l = cnt_open_r = cnt_block_f = cnt_deadend = 0;
            g_leg_t0 = millis();
            enterState(ST_RECOVERING);
            break;
        }
        else if (left_open)    { pend_180 = false; pend_right = false; }
        else                   { pend_180 = true;  }

        enterState(ST_DECIDING);
        break;
    }

    case ST_DECIDING: {
        TurnResult r;
        if (pend_180) {
            bool u_right = uturnGoesRight();
            LOGf("180 via the %s (L:%s R:%s -- more room that side)...",
                 u_right ? "RIGHT" : "LEFT",
                 rangeStr(S_LEFT).c_str(), rangeStr(S_RIGHT).c_str());
            recordTurn('U');
            turn180(r, u_right);
        } else {
            LOGf("turn %s...", pend_right ? "RIGHT" : "LEFT");
            recordTurn(pend_right ? 'R' : 'L');
            turn90(pend_right, r);
        }
        LOGf("  achieved %.1f deg  (pre-nudge err %.1f)  nudges=%d%s",
             r.achieved_deg, r.initial_err_deg, r.nudges,
             r.timed_out ? "  TIMED OUT" : "");
        /* A 90 leaves the corridor we arrived from on the side we turned
         * toward. A 180 does not: at a dead end, back the way we came is
         * exactly where we want to go. */
        if (!pend_180) blockCameFrom(pend_right);
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
const char *MODE_NAME[4] = { "SENSORS-ONLY", "STRAIGHT-TEST", "TURN-TEST", "MAZE" };

const char *modeName(int m) {
    return (m >= 0 && m <= 3) ? MODE_NAME[m] : "INVALID";
}

void printMenu(void) {
    /* Hold the telemetry stream off so the menu stays on screen long enough
     * to actually read it. */
    g_quiet_until = millis() + MENU_QUIET_MS;
    LOG("");
    LOG("=== PHASE 5 GYRO MAZE SOLVER ===");
    LOGf("  CURRENT MODE : %d (%s)", cfg_mode, modeName(cfg_mode));
    LOGf("  STATE        : %s", g_running ? "RUNNING" : "IDLE - send START to run");
    LOG("  --------------------------------");
    LOGf("  MODE:%d   0=sensors 1=straight 2=turn-test 3=maze", cfg_mode);
    LOGf("  BL:%d     base PWM          MN:%d  stall floor", cfg_base_pwm, cfg_min_pwm);
    LOGf("  MX:%d     max PWM           TP:%d  pivot PWM",   cfg_max_pwm, cfg_turn_pwm);
    LOGf("  LT:%d     left trim %%       RT:%d  right trim %%", cfg_left_trim, cfg_right_trim);
    LOGf("  KP:%.2f  KD:%.2f  KW:%.4f  WT:%.1f  TM:%.1f pivot stop margin",
         cfg_kp, cfg_kd, cfg_kw, cfg_wall_tilt, cfg_margin);
    LOGf("  CW:%d     corridor mm       RW:%d  widest mm   RL:%d  length mm",
         cfg_corridor_w, cfg_robot_w, cfg_robot_l);
    LOGf("  AX:%d     front sensor->axle  SA:%d  side sensor->axle",
         cfg_axle, cfg_side_axle);
    LOGf("  SS:%d     side sensor span mm", cfg_span);
    LOGf("  -> derived: centre-ref %d mm | park at %d mm | approach %d mm",
         wallRefMm(), frontStopMm(), approachMm());
    {
        float r = sweptRadiusMm();
        int half = cfg_corridor_w / 2;
        LOGf("  -> pivot sweeps %d mm radius, half-corridor %d mm, %d mm spare",
             (int)r, half, (int)(half - r));
        if (r >= half)
            LOG("  !! the chassis CANNOT pivot in this corridor -- it will hit a wall");
        else if (half - r < 25)
            LOG("  !! under 25 mm of pivot clearance -- expect corner contact");
    }
    LOGf("  timed approach crawls at PWM %d -- measure SP at THAT speed",
         cfg_min_pwm + (int)((cfg_base_pwm - cfg_min_pwm) * APPROACH_CRAWL));
    LOGf("  FB:%d     wall-ahead mm     FD:%d  front stop (0=auto=%d)",
         cfg_front_blk, cfg_front_stop, frontStopMm());
    LOGf("  OP:%d     side-open mm      WE:%d  wall emergency (0=auto=%d)",
         cfg_opening, cfg_wall_emerg, wallEmergMm());
    LOGf("  SP:%d     travel mm/s (approach %lu ms)  GS:%d  gyro sign",
         cfg_speed, (unsigned long)approachMs(), cfg_gyro_sign);
    LOGf("  CP:%d     creep PWM for the final park nudges", cfg_creep_pwm);
    /* The lift-both-wheels rule in driveTick() only pushes the OTHER wheel
     * up. With little headroom between BL and MN it stops being steering and
     * becomes acceleration -- which is how raising MN to 100 against BL 110
     * turned into front-wall collisions. */
    if (cfg_base_pwm - cfg_min_pwm < 40)
        LOGf("  !! BL-MN is only %d. Steering will inflate speed instead of"
             " turning. Aim for BL >= MN+40.", cfg_base_pwm - cfg_min_pwm);
    LOGf("  LOG:%d     telemetry stream on/off", cfg_log ? 1 : 0);
    LOG("  START | STOP | CAL | MENU | RESCAN");
    LOG("================================");
}

/* Everything that must be forgotten between runs. Called by both START and
 * STOP, so a STOP followed by a START genuinely restarts -- MODE 1 used to
 * keep its progress in function-level statics and silently refuse to rerun. */
void resetRunState(void) {
    m1_begun = false;
    m1_stopped = false;
    s_blocked_side    = -1;
    s_block_wall_hits = 0;
    g_stalled = false;
    g_path_len = 0; g_path[0] = 0;
    cnt_open_l = cnt_open_r = cnt_block_f = cnt_deadend = 0;
    tofFlush();
    headingReset();
}

void handleCommand(String c) {
    c.trim();
    c.toUpperCase();
    if (!c.length()) return;

    /* STOP is handled FIRST and unconditionally. It is the one command that
     * must never be refused or queued. */
    if (c == "STOP") {
        g_abort   = true;
        g_running = false;
        driveStop();
        motorStop();
        enterState(ST_IDLE);
        resetRunState();
        LOG("");
        LOGf(">>> HALTED <<<  mode stays %d (%s). Send START to run it again.",
             cfg_mode, modeName(cfg_mode));
        g_quiet_until = millis() + MENU_QUIET_MS;
        return;
    }

    if (c == "START") {
        if (!mpu_ok) { LOG("refusing to start: MPU6050 not responding"); return; }
        /* Without a front sensor frontBlocked() is permanently false, so the
         * robot never believes in a wall ahead and drives into the first one
         * at full speed. That is not a degraded mode worth offering. */
        if (cfg_mode == 3 && !st[S_FRONT].present) {
            LOG("refusing to start the maze: FRONT sensor never initialised.");
            LOG("  Without it the robot cannot see walls ahead and will drive");
            LOG("  into them. Check the wiring and XSHUT on GPIO19, or swap the");
            LOG("  front and left modules to find out which is at fault.");
            LOG("  MODE:0 and MODE:1 still work for diagnosis.");
            return;
        }
        if (g_running) {
            LOG("already running -- send STOP first");
            return;
        }
        g_abort = false;            /* clear any latched abort from last STOP */
        resetRunState();
        g_running = true;
        g_run_t0 = millis();
        enterState(ST_STARTUP);
        LOG("");
        LOGf(">>> STARTING  mode %d (%s) <<<", cfg_mode, modeName(cfg_mode));
        return;
    }

    if (c == "MENU")  { printMenu(); return; }

    /* Sensor init used to happen only inside setup(), so a bad boot scan was
     * unrecoverable: the ESP32 sat there for the rest of its life with every
     * sensor marked absent and no command could make it try again. */
    if (c == "RESCAN") {
        if (g_running) { LOG("send STOP before rescanning"); return; }
        LOG("re-running I2C recovery and sensor init...");
        i2cBusRecover();
        tofInitAll();
        if (!mpu_ok) {
            mpu_ok = mpuInit();
            LOG(mpu_ok ? "  [OK]   MPU6050 came back" : "  [FAIL] MPU6050 still absent");
        }
        tofFlush();
        g_quiet_until = millis() + MENU_QUIET_MS;
        return;
    }

    if (c == "CAL") {
        if (g_running) { LOG("send STOP before calibrating"); return; }
        g_abort = false;
        driveStop();
        gyroCalibrate(GYRO_CAL_FULL, "full cal");
        headingReset();
        return;
    }

    int colon = c.indexOf(':');
    if (colon < 0) { LOG("? unknown command -- send MENU"); return; }
    String k = c.substring(0, colon);
    float  v = c.substring(colon + 1).toFloat();

    /* Changing the mode is starting a new session, so it always goes through
     * STOP -> MODE -> START. Silently swapping the mode out from under a
     * running maze was how the old build ended up in states nobody could
     * account for. */
    if (k == "MODE") {
        int m = (int)v;
        if (m < 0 || m > 3) { LOG("? mode must be 0, 1, 2 or 3"); return; }
        if (g_running) {
            LOGf("send STOP first -- still running mode %d (%s)",
                 cfg_mode, modeName(cfg_mode));
            return;
        }
        cfg_mode = m;
        LOG("");
        LOGf(">>> MODE SET TO %d (%s) -- send START to run it <<<", cfg_mode, modeName(cfg_mode));
        g_quiet_until = millis() + MENU_QUIET_MS;
        return;
    }

    if      (k == "LOG")  cfg_log        = (v != 0);
    else if (k == "BL")   cfg_base_pwm   = (int)v;
    else if (k == "MN")   cfg_min_pwm    = (int)v;
    else if (k == "MX")   cfg_max_pwm    = (int)v;
    else if (k == "TP")   cfg_turn_pwm   = (int)v;
    else if (k == "LT")   cfg_left_trim  = (int)v;
    else if (k == "RT")   cfg_right_trim = (int)v;
    else if (k == "FD")   cfg_front_stop = (int)v;
    else if (k == "FB")   cfg_front_blk  = (int)v;
    else if (k == "OP")   cfg_opening    = (int)v;
    else if (k == "WE")   cfg_wall_emerg = (int)v;
    else if (k == "SP")   cfg_speed      = (int)v;
    else if (k == "CP")   cfg_creep_pwm  = (int)v;
    else if (k == "CW")   cfg_corridor_w = (int)v;
    else if (k == "RW")   cfg_robot_w    = (int)v;
    else if (k == "AX")   cfg_axle       = (int)v;
    else if (k == "RL")   cfg_robot_l    = (int)v;
    else if (k == "SS")   cfg_span       = (int)v;
    else if (k == "SA")   cfg_side_axle  = (int)v;
    else if (k == "KP")   cfg_kp         = v;
    else if (k == "KD")   cfg_kd         = v;
    else if (k == "KW")   cfg_kw         = v;
    else if (k == "WT")   cfg_wall_tilt  = v;
    else if (k == "TM")   cfg_margin     = v;
    else if (k == "GS")   cfg_gyro_sign  = (v < 0) ? -1 : 1;
    else { LOG("? unknown key -- send MENU"); return; }
    printMenu();
}

void pumpCommands(void) {
    static String line;
    static bool   busy = false;

    /* Re-entrancy guard. serviceWhileBusy() calls this from inside blocking
     * routines, and some commands (CAL, and STOP during a pivot) run code
     * that calls serviceWhileBusy() again. Without this, handleCommand()
     * could recurse and the shared line buffer would be shredded mid-parse. */
    if (busy) return;
    busy = true;

    while (Serial.available())   { char ch = Serial.read();
        if (ch == '\n' || ch == '\r') { handleCommand(line); line = ""; } else line += ch; }
    while (bt_ready && SerialBT.available()) { char ch = SerialBT.read();
        if (ch == '\n' || ch == '\r') { handleCommand(line); line = ""; } else line += ch; }

    busy = false;
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

    /* Three separate gates, all of which caused the "menu is unreadable"
     * problem when they were missing:
     *   - LOG:0 turns the stream off entirely
     *   - an idle robot has nothing to report, so it stays quiet and you can
     *     actually read the menu and type commands
     *   - a short quiet window after the menu or a mode change keeps that
     *     output on screen instead of burying it in 150 ms                  */
    if (!cfg_log) return;
    if (!g_running) return;
    if ((int32_t)(millis() - g_quiet_until) < 0) return;

    if (millis() - last < TELEMETRY_MS) return;
    last = millis();
    LOGf("M%d %-8s F:%-5s L:%-5s R:%-5s fv:%d | ctr:%c%+5.0f | Hdg:%+6.1f Tgt:%+5.1f Rate:%+6.1f Corr:%+5.0f | L:%3d R:%3d",
         cfg_mode, ST_NAME[g_state], rangeStr(S_FRONT).c_str(), rangeStr(S_LEFT).c_str(),
         rangeStr(S_RIGHT).c_str(), frontVotes(), g_wall_src, g_wall_err,
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
            /* m1_begun/m1_stopped are file-scope and reset by START and STOP.
             * As function-level statics they survived a STOP, so the second
             * START looked like the firmware had hung. */
            if (!m1_begun) { driveBegin(); m1_begun = true; }
            if (!m1_stopped) {
                if (frontBlocked()) {
                    driveStop(); m1_stopped = true;
                    LOG("MODE1: front obstacle -- stopped. Send STOP then START to rerun.");
                } else driveTick();
            }
            if (g_stalled) m1_stopped = true;
            break;
        }

        case 2: {                                 /* single 90 deg pivot     */
            TurnResult r;
            motorStop();
            /* Serviced wait, not delay(): STOP must be honoured during the
             * three seconds before the robot lurches into its pivot. */
            LOG("TURN TEST: pivoting in 3 s -- send STOP to cancel");
            {
                uint32_t t0 = millis();
                while (millis() - t0 < STARTUP_DELAY_MS && !g_abort) serviceWhileBusy();
            }
            if (!g_abort) {
                turn90(true, r);
                if (!g_abort) {
                    LOGf("TURN TEST: achieved %.1f deg | pre-nudge err %.1f | nudges %d%s",
                         r.achieved_deg, r.initial_err_deg, r.nudges,
                         r.timed_out ? " | TIMED OUT" : "");
                    LOG("err > 0 means it undershot -> LOWER TM.  err < 0 -> RAISE TM.");
                }
            }
            motorStop();
            g_running = false;
            if (!g_abort)
                LOG("TURN TEST done -- send START to repeat, or MODE:n for another mode.");
            g_quiet_until = millis() + MENU_QUIET_MS;
            break;
        }

        default:                                  /* full maze               */
            mazeTick();
            if (g_abort) { driveStop(); g_running = false; enterState(ST_IDLE); break; }
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
