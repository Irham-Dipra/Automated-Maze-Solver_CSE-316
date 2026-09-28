# Phase 5 — Gyro-Stabilised Maze Solver

Replaces `CurrentOnlySensorCode.ino` + `Phase3_ATmega_Slave.cpp`.
Integrates the proven techniques from `maheeCode/cse316_project-main/code/main/`
into your existing ESP32 + ATmega32 + VL53L0X + MPU6050 hardware.

**Flash both halves.** The UART protocol changed (sync byte + checksum); the
old and new firmware cannot talk to each other.

| File | Target | Status |
|---|---|---|
| `Phase5_ESP32_Brain/Phase5_ESP32_Brain.ino` | ESP32 (Arduino IDE, ESP32 core) | compiles clean, **not hardware-tested** |
| `Phase5_ATmega_Slave/Phase5_ATmega_Slave.c` | ATmega32 (avr-gcc) | compiles clean at 1 MHz and 16 MHz, **not hardware-tested** |

Libraries needed on the ESP32 side: `Adafruit_VL53L0X`, plus `Wire` and
`BluetoothSerial` from the ESP32 core. The MPU6050 is driven by direct
register access — no library.

---

## 1. What was actually wrong

Derived from `bluetooth_logs/`, `Reports/Progress_Report.txt`, and comparing
against Mahee's working firmware.

### 1.1 The UART protocol could turn a speed into a turn command

`Phase3_ATmega_Slave.cpp` parsed a bare 3-byte stream `[cmd][L][R]` — no sync
byte, no checksum. The command letters are:

```
'B'=66  'F'=70  'L'=76  'P'=80  'R'=82  'S'=83
```

Those sit **inside the PWM range you actually send**. Drop one byte on a noisy
9600-baud line and the state machine resyncs onto a PWM byte that happens to
equal `'R'` — and the robot pivots at full power while the ESP32 believes it is
driving straight. That is the "violently steered left", "the turn became a u
turn", and "did some hazardous moves" signature in `logsFor7-9.txt`.

**Fixed:** packets are now `[0xA5][cmd][L][R][checksum]`. A bad checksum is
dropped, never half-executed. The parser also rejects a byte in the command
slot that is not a real command letter, so it resyncs in one byte instead of
consuming three more.

### 1.2 Nothing stopped the motors when the master died

The slave held its last command forever. `serial_20260925_000240.txt` shows
`Connection lost` while the robot was driving; `serial_20260925_013930.txt`
shows the ESP32 taking 37 s to boot then dropping out. In both cases the
motors kept whatever they last had.

**Fixed:** the slave stops the motors if no valid packet arrives for 300 ms.
The ESP32 heartbeats every 50 ms. Every blocking wait in the ESP32 sketch
calls `motorHeartbeat()` for exactly this reason.

### 1.3 The PWM carrier was 61 Hz

`init_pwm()` used Fast PWM with prescaler 64. At the ATmega32's factory-default
**1 MHz internal RC**, that is `1e6 / (64 × 256) = 61 Hz`.

61 Hz into a TT motor through an L298N is the *"motors emit a high-pitched
buzzing sound but refuse to spin"* from your progress report. At low duty the
motor receives 16 ms pulses that shake the gearbox instead of turning it, and
the effective torque collapses. This is also a large part of why you needed
`BL=200` against `BR=10` — you were tuning around a broken carrier.

**Fixed:** phase-correct PWM with the prescaler chosen from `F_CPU`:
- 16 MHz → `/64` → 490 Hz
- 1 MHz → `/8` → 245 Hz

### 1.4 The heading PID had no clamp, no reset, and no windup protection

`serial_20260923_134105.txt`, repeating for the entire log:

```
Yaw:-206.64  Adj:826  (Steering Left)  [PWM_L:0 PWM_R:255]
```

Three separate faults compounding:
1. The P term acted on **absolute accumulated yaw**, which was never reset
   between legs. After the robot lost track once, the error only grew.
2. The output had **no clamp**. `Adj:826` on a 0–255 scale is meaningless —
   the controller had saturated to one wheel dead, one wheel flat out.
3. `PWM_L:0` put the left wheel **below its stall floor**, so it stopped
   contributing at all and the robot could not physically correct.

**Fixed:**
- Correction is clamped to `CORR_RATIO_PCT` (40%) of the base PWM. Lowering
  the base speed now *softens* steering instead of sharpening it.
- The heading accumulator is zeroed at the start of every leg and after every
  pivot.
- When a wheel would fall under `MN` (stall floor), **both** wheels are lifted
  to preserve the differential rather than clipping one to zero — taken
  directly from Mahee's `drive.c`.
- A yaw governor refuses to add rotation in a direction the chassis is already
  turning fast (`YAW_GOVERNOR_DPS`).

### 1.5 The robot could not tell "stalled" from "driving"

`serial_20260923_133856.txt`:

```
13:38:43.146  Yaw:-7.38 ... [PWM_L:121 PWM_R:179]
13:38:45.585  Yaw:-7.40 ... [PWM_L:121 PWM_R:179]
```

A 58-count differential held for **four seconds** with the yaw frozen to two
decimal places. The robot was not moving at all. That is a dead motor rail, a
blown L298N channel, or a lost common ground — and the firmware had no way to
notice, so it just kept integrating.

**Added:** `checkStall()`. If the motors are commanded but the yaw has not
changed and the front range has not closed for 1.5 s, it stops and prints the
hardware checklist instead of winding up a phantom correction. It deliberately
only judges while the front sensor has a wall in range — with no encoders,
an empty corridor is genuinely indistinguishable from a seized wheel, so it
does not guess there.

### 1.6 Timed turns cannot be calibrated

`turn90()` was `delay(600)`. The same 600 ms gives a different angle on a full
battery than on a flat one, and different again on a different floor.
`serial_20260917_*.txt` is you re-tuning `TR`/`TL` over and over and never
converging.

**Replaced** with Mahee's closed-loop pivot from `turn.c`:
kick → slow sweep to `(target − margin)` → active brake → settle **while still
integrating the coast** → error-scaled nudges until inside a 2° deadband.
Counting the coast is the key part: that is the rotation a timed turn always
loses.

### 1.7 The gyro was calibrated once, unchecked

MPU6050 bias drifts thermally. A stale bias integrates into a phantom heading
error that steers the robot into a wall on a long run. Worse, if anyone is
still touching the robot during calibration, the motion is baked in as the
zero point.

**Fixed**, from Mahee's `heading.c`:
- Calibration measures the **spread** across its samples and **rejects** the
  set if the chassis was not genuinely still (keeping the previous offset
  rather than corrupting it).
- A quick re-calibration runs after every stop and after every pivot — the only
  moments a valid bias measurement is possible.
- X and Y are calibrated too, not just Z.
- The DLPF is now enabled (44 Hz). The old code left it wide open, so every bit
  of gear-motor vibration went straight into the heading integral.

### 1.8 The control loop was slow and blind

The old `loop()` called `rangingTest()` three times plus `delay()` — 100–180 ms
per iteration (visible in the log timestamps), and completely blind during
turns. Gyro integration with a variable `dt` makes the calibration constant
meaningless.

**Fixed:** the ToF sensors run in **continuous ranging** mode and are polled
non-blocking on a fixed 20 ms control tick.

### 1.9 One bad reading could trigger a turn

**Ported from `sonar.c`:** median-of-3, staleness check, implausible-jump gate,
and a distinct *"too close to measure"* state that is never confused with
*"open"* — the old code reported an **opening** at the exact moment of impending
collision, because a sub-minimum reading and an out-of-range reading looked
identical.

Front-obstacle detection uses **voting** (2 of the last 5 pings), not
consecutive hits. A yawing chassis walks the front beam off a real wall for
several pings at a time, so a "2 consecutive" rule can never fire — Mahee's
`config.h` documents measuring `F` reading 51,60,57,65,65 *increasing* while
driving into a wall.

### 1.10 Front ToF failed to initialise on every boot

`27sep1.txt` shows `[FAIL] Sensor FRONT NOT FOUND at 0x30!` on every attempt.
`initSensor()` now retries three times with an I2C bus recovery between
attempts, and the maze logic degrades gracefully around a sensor that never
comes up rather than acting on a missing reading.

### 1.11 Bluetooth brownouts

Boot is staggered (UART → I2C/sensors → radio last), the robot does not require
a BT client to run, and losing the phone no longer matters because the slave
failsafe covers a master crash.

---

## 2. Two things software cannot fix

Be clear about these in your report.

**The 1 MHz internal RC oscillator.** It is factory-trimmed to only ±3% at
5 V / 25 °C and drifts further with supply voltage and temperature. UART needs
both ends within about ±2%. On battery, with the rail sagging under motor
load, this is a genuine source of the corrupted packets in §1.1. The new
checksum turns that corruption into *dropped* packets (safe) instead of *wrong
moves* (dangerous) — but if the `FAILSAFE` message appears constantly, **fit
the 16 MHz crystal and set the fuses.** Build instructions are in the header of
`Phase5_ATmega_Slave.c`.

**The motor rail.** `BL=200` against `BR=10` is not a tuning result, it is a
fault report. Two identical TT motors on an intact L298N do not differ by 20×.
Before tuning anything, with the robot on blocks:
1. Measure the motor battery **under load** — it must stay above ~6.5 V.
2. Confirm `L298N GND → breadboard GND`. Without it the PWM logic has no
   reference and the driver behaves randomly.
3. Swap the two motors between OUT1/2 and OUT3/4. If the weakness *follows the
   motor*, it is the motor. If it *stays on the same channel*, that L298N
   channel is blown.

The `LT`/`RT` trim settings let you compensate a modest mismatch. They cannot
compensate a dead channel.

---

## 3. Bring-up order — do not skip steps

Everything is switchable at runtime over Bluetooth (`MazeSolver_P5`).
Send `MENU` at any time. All commands are `KEY:VALUE`, plus `START`, `STOP`,
`CAL`, `MENU`.

### Step 1 — `MODE:0` (sensors only, motors never move)

```
MODE:0
START
```

Check the telemetry line:
- `F`/`L`/`R` show plausible distances. `--` means that sensor never
  initialised; `oo` means nothing in range; `<<` means too close to measure.
- **Verify the gyro sign.** Rotate the robot **left** by hand. `Hdg` must go
  **positive**. If it goes negative, send `GS:-1` and re-check.
  *Everything downstream depends on this.*

### Step 2 — `MODE:2` (single 90° right pivot)

```
MODE:2
START
```

It reports:

```
TURN TEST: achieved 88.4 deg | pre-nudge err 3.2 | nudges 1
```

Measure the real angle with a protractor.
- `achieved` disagrees with the protractor → adjust `GYRO_LSB_PER_DPS`
  (`new = old × commanded / measured`).
- `pre-nudge err` consistently **positive** (undershoots) → **lower** `TM`.
- Consistently **negative** (overshoots) → **raise** `TM`.
- `nudges` routinely at 5 → `TP` (pivot PWM) is too high; lower it.

Repeat until `pre-nudge err` is within about ±3° with zero or one nudge.

### Step 3 — `MODE:1` (straight corridor run)

```
MODE:1
BL:110
START
```

Tune in this order, one at a time:
1. `BL` — lowest value at which **both** wheels reliably start. Too low and
   they buzz.
2. `MN` — the stall floor. Lower it until the slow wheel in a correction stops
   dead, then back off.
3. `LT` / `RT` — percent trim per side. With `KP:0 KD:0 KW:0` the robot should
   roll roughly straight on trims alone before you enable the controller.
4. `KP` — raise until it corrects briskly; back off when it starts weaving.
5. `KD` — raise to damp the weave. `KD` opposes `KP`; if it fights the
   correction rather than damping it, your gyro sign is wrong (back to step 1).
6. `KW` — wall centring, expressed as degrees of heading target per mm of
   error. Start at `0` (pure gyro), then raise gently. `0.020` tilts about
   2° for a 100 mm offset.

### Step 4 — measure the two placeholder constants

These are estimates in the source and **must** be replaced:

- `SENSOR_TO_AXLE_MM` — from the front ToF face back to the wheel axle.
  Measure it. This is what makes the robot stop with its *axle*, not its nose,
  centred in an opening.
- `TRAVEL_SPEED_MMS` — drive a timed 1 m run at your final `BL` and divide.

### Step 5 — `MODE:3` (full maze)

```
MODE:3
START
```

`STOP` halts it at any time. The recorded path prints on completion.

---

## 4. Design note: why there is only one controller now

Your `Assets/context.md` describes positional PID on the side sensors, and the
23 Sep logs show a heading PID added on top. Two controllers writing to the
same two motors will fight — and with mismatched motors the positional one
oscillates between stalled and full power, which is exactly problem #2 in your
own context notes.

Phase 5 has **one** controller: the gyro holds the heading. The side walls do
not touch the motors; they only nudge the heading **target** by a few degrees
(`KW`, capped at `WALL_TILT_MAX_DEG`). So the ToF sensors do what they are good
at — detecting walls and openings — and the gyro does what it is good at —
holding a straight line. `KW:0` makes it pure gyro, which is a useful fallback
if the side sensors are unreliable.

---

## 5. Protocol reference

```
ESP32 -> ATmega32,  9600 8N1,  5 bytes:

    [0xA5] [cmd] [left_pwm] [right_pwm] [chk]
    chk = cmd ^ left_pwm ^ right_pwm ^ 0x5A

cmd:  'F' forward      (both wheels forward, independent PWM)
      'B' backward
      'L' pivot left   (left wheel reverse, right forward)
      'R' pivot right  (left forward, right wheel reverse)
      'S' stop         (coast)
      'K' brake        (both H-bridge inputs high — active braking)
      'P' accepted as an alias for 'F'

The slave stops the motors if no valid packet arrives within 300 ms.
```

The ATmega's TXD is currently unconnected, so the link is one-way. If you want
acknowledgements later, wire `ATmega PD1 (TXD) → level shifter → ESP32 RX2
(GPIO16)`; the ESP32 side already has `UART_RX` defined for it.
