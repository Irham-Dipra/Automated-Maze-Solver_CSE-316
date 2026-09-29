# Autonomous Maze-Solving Robot — Project Report

**CSE 316 — Microcontroller Sessional**
ESP32 + ATmega32 · 3× VL53L0X ToF · MPU6050 · L298N · 2WD TT-motor chassis

This document records what the robot does, how the firmware works, how it got
there, and — for the one goal that was not reached — exactly why it failed and
what would be needed to finish it.

---

## Contents

1. [What the robot does](#1-what-the-robot-does)
2. [Timeline](#2-timeline)
3. [The starting point and what was wrong with it](#3-the-starting-point-and-what-was-wrong-with-it)
4. [Everything that was fixed](#4-everything-that-was-fixed)
5. [Architecture](#5-architecture)
6. [The ESP32 firmware, section by section](#6-the-esp32-firmware-section-by-section)
7. [The ATmega32 slave](#7-the-atmega32-slave)
8. [The control law in full](#8-the-control-law-in-full)
9. [Geometry: three measurements, everything else derived](#9-geometry-three-measurements-everything-else-derived)
10. [The maze state machine](#10-the-maze-state-machine)
11. [Tooling](#11-tooling)
12. [Parameter reference](#12-parameter-reference)
13. [The shortest-path mode: what was built and why it failed](#13-the-shortest-path-mode-what-was-built-and-why-it-failed)
14. [How to finish the shortest-path mode](#14-how-to-finish-the-shortest-path-mode)
15. [Open issues](#15-open-issues)
16. [File map](#16-file-map)

---

## 1. What the robot does

**Working, demonstrated, evaluated:**

- Drives a corridor holding a straight line on the gyro, centring itself
  between the walls from the two side ToF sensors.
- Detects walls ahead, decelerates, parks at a measured distance, and pivots
  90° or 180° closed-loop on the gyro.
- Solves an unknown maze by the right-hand rule, recording the turns it took.
- Recovers from a dead end with a U-turn toward whichever side has more room.
- Reports everything over Bluetooth, and accepts commands and live parameter
  changes from a phone or PC.
- Refuses to run when a sensor it depends on is missing, rather than driving
  blind.

**Built but not working:**

- A shortest-path mode (explore once, then drive the route with every dead end
  removed). The path-reduction algorithm is correct and unit-tested; the data
  it was fed is not reliable. Sections 13 and 14 cover this in full.

**Modes**

| Mode | Name | Purpose |
|---|---|---|
| `MODE:0` | SENSORS-ONLY | Telemetry only; motors never move. Bring-up and diagnosis. |
| `MODE:1` | STRAIGHT-TEST | Drive straight until a wall. Tunes the heading loop and the trims. |
| `MODE:2` | TURN-TEST | One 90° pivot, reporting the achieved angle. Tunes `TM`. |
| `MODE:3` | MAZE-EXPLORE | Full right-hand-rule maze solve. **The working demo.** |
| `MODE:4` | SPEED-RUN | Follow a stored shortest route. **Does not work — see §13.** |

---

## 2. Timeline

Dates are commit dates on `claude/vibrant-curie-uhsy3h`.

### Before this work — 19 Aug to 28 Sep

The project existed as several iterations (`f74bcf2` Initial Commit through
`9a325c7` "before claude"). The robot drove, but erratically. A gyro had
recently been added and the car had stopped working reliably. A friend's
working ATmega + sonar + MPU6050 project (`maheeCode/`) was available as a
reference for approach.

### 28 Sep — rebuild and bring-up

| Commit | What |
|---|---|
| `4be62c1` | **Phase 5 written from scratch.** Framed UART, failsafe, proper PWM carrier, clamped heading PD, closed-loop gyro pivots, right-hand-rule state machine, Bluetooth menu, host test suite. |
| `b8f00ac` | STOP made unconditional and always reachable; mode changes gated behind STOP → MODE → START; telemetry flooding stopped; current mode made visible. |
| `26b173b` | Fixed the ToF near-wall latch that jammed the front sensor after every pivot — the robot had been turning at every junction. |
| `d9122b4` | Control law and every tunable documented. |
| `6ec52e5` | Junction geometry derived rather than hand-tuned; deceleration ramp into the stop instead of cutting power and coasting. |
| `4f9005c` | Side sensors measured separately from the chassis; timed approaches forced to a fixed crawl so one `SP` works regardless of corridor length. |
| `7975957` | Measured chassis and tuning values shipped as defaults; `WE` derived. |
| `9e1892a` | Turn decision moved from the moment of detection to the junction itself (`ST_LOOKING`); came-from blocking added to stop the robot turning back down the corridor it had just left. |
| `fe0fa4e` | Creep-to-target parking; brake held through the settle; a failed sensor boot made recoverable with `RESCAN`. |

**Milestone: modes 0, 1 and 2 verified on the bench. Mode 3 solving mazes.**

### 29 Sep — control fixes, tooling, shortest path

| Commit | What |
|---|---|
| `15029e4` | `classify()` could return "clear corridor" with a wall dead ahead — the robot drove into walls at full speed while the front sensor reported them perfectly. Also fixed a `tofValid`/`tofMedian` mismatch that fed garbage into the centring, and rate-limited the heading target. |
| `48f664b` | Wall-centring gain raised ~3× (it had been converging in ~6 s against a 4 s corridor); tilt ceiling exposed as `WT`; `ctr:` telemetry column added; phantom junctions suppressed; U-turns aimed at the roomier side; creep pulses closed-loop on the gyro. **PC Bluetooth monitor written.** |
| `efdaf31`, `78a0f5d`, `867cfbe`, `c5c58d9`, `acac559` | Monitor hardening: correct advertised name, SDP channel discovery, real error diagnosis instead of bare timeouts, non-blocking stdin fix, and a startup probe so the monitor stops claiming "connected" when nothing is on the other end. |
| `227f753` | Straights recorded during exploration; path reduction (dead-end elimination); `MODE:4` speed run; `PATH:` commands; route saved to flash. |

**Milestone: evaluation passed on mode 3.**

### After evaluation

`MODE:4` trialled and **failed**: mode 3 did not record `S` at every junction
it passed straight through, so the reduced route did not match the maze. Work
stopped here. Section 13 diagnoses this precisely.

---

## 3. The starting point and what was wrong with it

The original firmware had eleven distinct defects. They are listed because
several are classic and worth recognising again.

**3.1 The UART protocol could turn a speed into a turn command.** Raw bytes,
no framing. One dropped byte and a PWM value of `0x4C` was read as the command
`'L'`. The robot appeared to take random turns.

**3.2 Nothing stopped the motors when the master died.** The slave held the
last command forever. An ESP32 reset left the robot driving into a wall at
full power.

**3.3 The PWM carrier was 61 Hz.** Far below the motors' electrical time
constant, so they buzzed and heated instead of turning. This was being
compensated by raising the minimum PWM, which made the stall floor look much
higher than it was.

**3.4 The heading PID had no clamp, no reset and no windup protection.** Logs
showed `Adj:826` against a 255-count range — one wheel at 0, the other at 255,
spinning on the spot.

**3.5 The robot could not tell "stalled" from "driving".** A flat battery
looked identical to a tuning problem.

**3.6 Timed turns cannot be calibrated.** The same 600 ms gives a different
angle on a full battery than a flat one, and different again on carpet.

**3.7 The gyro was calibrated once, unchecked.** A calibration taken while the
chassis was still rocking poisoned every subsequent heading.

**3.8 The control loop was slow and blind.** ~180 ms per iteration, and fully
blind during turns.

**3.9 One bad reading could trigger a turn.** No filtering, no confirmation.

**3.10 The front ToF failed to initialise on most boots**, and the failure was
unrecoverable — the robot sat idle for the rest of its life.

**3.11 Bluetooth brownouts** from initialising the classic BT stack alongside
everything else at boot.

---

## 4. Everything that was fixed

| Problem | Cause | Fix |
|---|---|---|
| Random turns | Unframed UART | `[0xA5][cmd][L][R][chk]`, checksum verified, bad frames dropped |
| Runaway after a crash | No failsafe | 300 ms watchdog on the slave; motors stop if no valid packet |
| Motors buzz, don't turn | 61 Hz PWM | Timer1 phase-correct 8-bit, prescaler chosen from `F_CPU` (490 Hz @16 MHz) |
| `Adj:826`, one wheel at 0 | Unclamped PID | Correction capped at 40 % of base speed |
| Turns inconsistent | Timed turns | Closed-loop gyro pivots, coast measured not guessed |
| Can't stop the robot | STOP queued behind blocking sections | STOP handled first, unconditionally; blocking loops service commands |
| Telemetry unreadable | Flood at full rate | 150 ms throttle, quiet window after menu, `LOG:0` |
| Turns at every junction after a pivot | Near-wall latch fed itself | Latch releases after 380 ms of contrary evidence |
| Geometry impossible to tune | Three different lengths conflated into one constant | `CW`, `RW`, `RL`, `AX`, `SS`, `SA` measured separately; the rest derived |
| Braking distance vs parking accuracy | One constant doing two jobs | `FB` detects, `FD` parks, creep closes the gap under measurement |
| Little yaw right on every stop | Brake released into a coast | Brake held for the whole settle |
| Dead robot after a bad boot | Sensor init only in `setup()` | `RESCAN`, auto-retry, I²C bus recovery and line diagnostics |
| Turning back down the corridor just left | No memory of arrival | Came-from block: that side ignored until a wall appears there |
| Left turn where right was open | Decision taken up to `FB` mm out, at speed | `ST_LOOKING`: stop at the junction, then decide |
| Drives into walls at full speed | `classify()` fell through to `J_CORRIDOR` with the front blocked | `if (fw) return J_DEAD_END` — never answer "carry on" with a wall ahead |
| Heading target thrashing | `tofValid()` judged `latest`, controller used `median` | Judge and use the same number; ignore readings too far to be a corridor wall; rate-limit the target |
| Never re-centres in a corridor | `KW` ~3× too low for the corridor length | `KW` 0.020 → 0.060; ceiling exposed as `WT`; `ctr:` column added |
| Random stalls in clean corridor | 2-tick (40 ms) opening confirmation | Sides need 5 ticks; front keeps its own 2-tick confirm |
| Crash on dead-end U-turn | Always turned right | Turns toward whichever side reads further |
| Drift accumulating on reverse nudges | Creep pulses open-loop | Closed on the gyro, with the sign flipped for reverse |

---

## 5. Architecture

```
        ┌──────────────────────────────────────────┐
        │  ESP32  (master)                         │
        │   · 3× VL53L0X over I²C (XSHUT sequenced)│
        │   · MPU6050 over I²C (direct registers)  │
        │   · Bluetooth SPP menu + telemetry       │
        │   · maze logic, heading PD, geometry     │
        └───────────────┬──────────────────────────┘
                        │ UART 9600, framed packets
                        │ [0xA5][cmd][L][R][chk]
        ┌───────────────▼──────────────────────────┐
        │  ATmega32  (slave)                       │
        │   · interrupt-driven RX ring buffer      │
        │   · Timer1 phase-correct PWM             │
        │   · 300 ms failsafe                      │
        └───────────────┬──────────────────────────┘
                        │
                   ┌────▼────┐
                   │  L298N  │ → 2× TT motors
                   └─────────┘
```

**Why two MCUs.** The ESP32 has the RAM, the I²C throughput and the Bluetooth
stack; the ATmega32 is what the course requires and is a reliable PWM
generator. The split is clean: the ESP32 never touches a motor pin, the
ATmega32 never makes a decision. Every command crossing that boundary is
framed and checksummed, and the slave stops on its own if the master goes
quiet.

**Why the slave is deliberately stupid.** It has no PID, no state, no
knowledge of the maze. It applies a PWM pair and counts down a watchdog. That
means a master crash is survivable and the slave never needs reflashing while
tuning.

---

## 6. The ESP32 firmware, section by section

`Phase5_ESP32_Brain.ino`, ~2 600 lines, organised into numbered sections.

**§1 CONFIGURATION (line 96).** Every constant, each with the reasoning for
its value in a comment. Pin assignments, timings, thresholds.

**§1b TYPE DECLARATIONS (381).** All structs and enums are declared here,
before anything else. This is not stylistic: the Arduino IDE auto-generates
function prototypes and splices them in *above* user code, so a function
signature mentioning a user type fails to compile unless the type is declared
first.

**§2 RUNTIME-TUNABLE STATE (436).** Everything adjustable over Bluetooth.
Tuning without reflashing is the difference between a two-minute change and a
ten-minute one, and on a robot that drains batteries while you work, that
matters.

**§3 LOGGING (567).** `LOG`/`LOGf` write to both USB serial and Bluetooth.
Throttled, with a quiet window after the menu so the menu stays readable.

**§4 MOTOR LINK (587).** Frames and sends `[0xA5][cmd][L][R][chk]`, and a
heartbeat that repeats the current command so the slave's failsafe never trips
during normal operation.

**§5 MPU6050 (647).** Direct register access, no library. `FS_SEL = 1` gives
±500 °/s at 65.5 LSB/°/s. A library was avoided because the ones available
hide the register writes, and a silently failed scale-select write is very hard
to diagnose from the outside (see §15).

**§6 HEADING (684).** Bias calibration with spread rejection — samples are
taken, and if their spread shows the chassis was moving, the calibration is
rejected and retried rather than baked in. Integration runs at a fixed 20 ms
tick from `loop()`, so it continues during turns, creeps and approaches, not
only while driving.

**§7 ToF SENSORS (754).** Three VL53L0X on one I²C bus. They all boot at
address 0x29, so XSHUT is used to bring them up one at a time and readdress
each. Continuous ranging, median-of-3 filtering, an implausible-jump gate, and
a "too close to measure" inference: a VL53L0X returns garbage below ~40 mm, so
that condition is *inferred* rather than read, and latched briefly with an
expiry (the missing expiry was bug 3.11).

**§8 DRIVE (1036).** One controller, not two. See §8 below.

**§9 PIVOTS (1256).** Kick, sweep, brake, measure, nudge. The sweep stops `TM`
degrees early and the coast is then *measured*; if the total falls outside a
deadband, short nudge pulses correct it. A 180° turn is done as two 90°s with a
settle between, because momentum has less time to build and there is less coast
to correct for.

**§10 MAZE STATE MACHINE (1390).** See §10 below.

**§11 BLUETOOTH COMMAND MENU (2151).** Line-based parser. `STOP` is handled
first and unconditionally. Mode changes require STOP → MODE → START.

**§12 TELEMETRY (2429).** One line per 150 ms:

```
M3 DRIVING  F:412  L:180  R:175  fv:2 | ctr:B  +12 | Hdg: -2.1 Tgt: +0.5 Rate: -8.3 Corr: -11 | L:100 R:152
```

| Field | Meaning |
|---|---|
| `M3` / `DRIVING` | mode, state machine state |
| `F/L/R` | distance mm. `oo` nothing in range, `<<` too close to measure, `--` sensor never initialised |
| `fv` | front votes 0–5; a wall is believed at 2 |
| `ctr` | which walls the centring is using (`B`oth / `L` / `R` / `-` none) and the error in mm, + = more room on the right |
| `Hdg` / `Tgt` | heading and target, degrees; + = left |
| `Rate` | yaw rate °/s |
| `Corr` | final clamped correction; + = steer right |
| `L:` `R:` | PWM actually sent, after trim |

**§13 SETUP / LOOP (2462).** Fixed 20 ms tick with re-basing if it falls
behind (a burst of catch-up ticks would corrupt the gyro integration).
Bluetooth is initialised *last* because the classic BT stack is the largest
current spike at boot.

---

## 7. The ATmega32 slave

`Phase5_ATmega_Slave.c`, 325 lines, compiles clean under `-Wall -Wextra` at
both 1 MHz and 16 MHz.

**Protocol.** `[0xA5][cmd][left][right][chk]`, `chk = cmd ^ left ^ right ^ 0x5A`.
Commands: `F` forward, `B` back, `L` pivot left, `R` pivot right, `S` stop,
`K` brake (`P` accepted as an alias). A bad checksum drops the whole frame and
returns to hunting for the sync byte — it never half-executes. A `0xA5`
appearing inside a payload cannot desynchronise the parser permanently.

**Failsafe.** If no valid packet arrives for 300 ms the motors stop and a
message is printed. This is the single most important safety property in the
system.

**PWM.** Timer1 in phase-correct 8-bit mode, prescaler chosen from `F_CPU`:
490 Hz at 16 MHz, 245 Hz at 1 MHz. If the fuses are still at the factory
1 MHz internal oscillator, the UART bit timing is marginal and the failsafe
will trip constantly — that symptom points at the crystal, not the code.

**RX.** Interrupt-driven ring buffer, so a byte is never missed while the main
loop is doing something else.

---

## 8. The control law in full

There is exactly **one** controller. The original design ran a positional PID
on the side sensors *and* a heading PID on the gyro, and they fought each
other; with mismatched motors the positional one oscillated between stalled and
full power.

Here, the gyro holds the heading. The side walls only nudge the heading
*target* by a few degrees. `KW:0` disables centring entirely and the robot
still drives straight.

Every tick (20 ms), in order:

```
 1.  wall_err = r_mm - l_mm                      both walls visible
              = (wallRef - l_mm) * 2             left wall only
              = (r_mm - wallRef) * 2             right wall only
              = 0                                nothing usable

     A single-wall reading is only used if it is close enough to be a
     corridor wall at all (<= maxWallMm). A left sensor reading 275 mm
     against a 147 mm reference is the mouth of an opening, and holding
     station off it steers the robot into whatever is behind.

 2.  target = clamp(-KW * wall_err, ±WT)
     target is slew-limited to 2°/tick, so a flickering sensor cannot whip
     it across its whole range in one tick.

 3.  EMERGENCY: if exactly one side is inside WE, override the target with
     a steer-away whose severity RAMPS with proximity. A hard step to full
     authority measured 71 °/s of yaw and bounced the robot off one wall
     into the other.

 4.  drift = heading - target
     corr  = KP * drift + KD * yaw_rate

 5.  YAW GOVERNOR: past 60 °/s, stop *adding* rotation in that direction.
     Beyond that rate the ToF beams point far enough off-axis that they stop
     describing the corridor.

 6.  base = MN + (BL - MN) * speed_scale         speed_scale set by approach ramp

 7.  THE CLAMP: |corr| <= 40 % of base.
     One line. It is what prevents Adj:826 and PWM 0-vs-255.

 8.  left = base + corr,  right = base - corr

 9.  STALL FLOOR: if a wheel would fall below MN, lift BOTH rather than
     clipping one, so the differential — the steering — survives.

10.  Apply LT/RT percentage trims, clamp to [0, MX], send.
```

**Why centring is expressed as a heading target, not a steering term.** A
steering term would fight the heading controller. A target does not: the
heading loop still does all the work, it is simply given a slightly different
angle to hold. The cost is that position is now the *integral* of heading, so
it settles more slowly — which is exactly the bug found on 29 Sep, where the
time constant was about 6 s against a corridor traversed in 4 s.

---

## 9. Geometry: three measurements, everything else derived

Measure `CW`, `RW`, `AX`, `SS`, `SA`, `RL` once. Everything the navigation
needs falls out, and cannot drift out of agreement:

| Derived | Formula | Meaning |
|---|---|---|
| centring reference | `(CW - SS) / 2` | what a side sensor reads when centred. Built from the *sensor span*, not the chassis width — sensors inset from the widest point read further, and using `RW` here makes a centred robot look off-centre by exactly that inset |
| park distance | `CW/2 - AX` | front gap that leaves the axle on the junction centreline |
| approach distance | `SA + CW/2` | how far past an opening's near edge the axle must travel. Measured from the *side* sensor, because a side sensor is what spots the opening |
| max wall reading | `CW - RW/2 - SS/2` | beyond this a side reading is not a wall to centre against |
| swept radius | `max(hypot(AX, RW/2), hypot(RL-AX, RW/2))` | the circle the furthest corner sweeps while pivoting; checked against the half-corridor so an impossible geometry is reported at the menu instead of discovered by watching the robot grind into a corner |
| `WE` | `(CW-RW)/4 + (RW-SS)/2` | emergency steer-away distance. Forgetting the inset term is what makes a hand-picked `WE` fire late — the chassis is always nearer the wall than its sensor claims |

---

## 10. The maze state machine

```
STARTUP ──► DRIVING ──┬──► (corridor) ───────────────┐
                      │                              │
                      ├──► APPROACHING ──► STOPPING ──► CREEPING
                      │        (ramp down)   (brake)   (nudge to target)
                      │                                    │
                      ├──► CONFIRM_EXIT ──► FINISHED        ▼
                      │                              RECALIBRATING
                      └──► (dead end) ─────────┐            │
                                               │            ▼
                      ┌────────────────────────┴───────► LOOKING
                      │                                    │
                      │   RECOVERING ◄── DECIDING ◄────────┘
                      └────────┘         (pivot)
```

| State | What it does |
|---|---|
| `STARTUP` | 3 s pause so hands are clear |
| `DRIVING` | heading hold + centring; classify the junction ahead |
| `APPROACHING` | ramp the speed down; with a wall ahead, close the loop on distance; with no wall, crawl on a timer |
| `STOPPING` | hold the brake through the whole gyro settle |
| `CREEPING` | short forward/back pulses, re-measuring between each, until the front gap is within 15 mm of target |
| `RECALIBRATING` | re-zero the gyro bias — the only moment the chassis is genuinely stationary — and flush the ToF history |
| `LOOKING` | stopped on the junction centreline, both side sensors looking straight down their openings. **Decide here**, not on approach |
| `DECIDING` | execute the pivot; arm the came-from block |
| `RECOVERING` | 400 ms of gyro-only driving while the ToF filters refill |
| `CONFIRM_EXIT` | "all open" can be a T-junction a moment before the front wall closes in; drive on 1.5 s and re-check |
| `FINISHED` / `IDLE` | stopped |

**Why decide at the junction, not on detection.** The original code chose the
direction the instant a junction was suspected — up to `FB` millimetres away,
at speed, with the side sensors still pointed at corridor walls. In one logged
run that meant deciding while the left had opened and the right had not yet
come into view: "forced left" at a junction that was open both ways. The stop
costs nothing, because under the right-hand rule every outcome except "carry
straight on" required stopping anyway.

**The came-from block.** After a 90° pivot, the corridor just left is now on
one side and looks exactly like a fresh opening. That side is ignored until a
wall is genuinely seen there, or for 4 s, whichever comes first. A 180° needs
no block: at a dead end, back the way you came *is* the correct move.

---

## 11. Tooling

### PC Bluetooth monitor — `tools/maze_monitor.py`

Phone apps show about twenty lines; a run produces a few thousand, and the line
that explains a crash is always the one that just scrolled off. This keeps
every line on disk, timestamped, flushed per line, while you type commands.

```sh
pip install pyserial
python3 tools/maze_monitor.py                  # find the port, connect, log
python3 tools/maze_monitor.py --list           # if it cannot tell which
python3 tools/maze_monitor.py --mac AA:BB:...  # Linux, direct RFCOMM
```

Local commands (never sent to the robot): `/f <text>` filter the screen only,
`/mark <note>` label a run in the log, `/stats`, `/log`, `/q` — which sends
`STOP` on the way out, so closing the window cannot leave the robot driving.

Three things learned the hard way and now handled:

- **The ESP32 serves exactly one Bluetooth client.** While the phone is
  connected the PC cannot get in, and the failure looks like a dead robot.
- **Opening a port is not being connected to something.** `rfcomm bind`
  creates `/dev/rfcomm0` whether or not the link ever comes up, and pyserial
  opens a tty non-blocking, so a dead link looks like a healthy session. The
  monitor now sends `MENU` at startup and warns loudly if nothing answers.
- **The RFCOMM channel is not always 1.** It is discovered over SDP.

On Linux, if `--mac` times out while `l2ping` gets replies, the radio link is
fine and it is the socket bluez will not complete — use:

```sh
sudo rfcomm connect 0 <MAC> 1     # in a second terminal, left running
python3 maze_monitor.py --port /dev/rfcomm0
```

### Host test suite — `test/run_tests.sh`

The real `.ino` is compiled on a PC against stub Arduino headers, with a
scripted fake VL53L0X and a clock the test drives by hand. No robot needed.
64 checks across five suites:

| Suite | Covers |
|---|---|
| `test_common.cpp` | the near-wall latch releases instead of jamming forever |
| `test_junction.cpp` | junction classification, came-from blocking |
| `test_creep.cpp` | creep to the park point; no nudging on a dead sensor |
| `test_frontwall.cpp` | a wall ahead is never "clear corridor"; heading-target slew limiting; opening-mouth rejection |
| `test_path.cpp` | all nine LSRB collapses against a hand-written table; reduction properties |
| `test_replay.cpp` | one symbol per junction; desync detection |

This suite caught several real bugs before they reached the robot, including
two in the speed-run code (§13).

---

## 12. Parameter reference

All changeable live over Bluetooth as `KEY:VALUE`.

### Chassis (measure once)

| Key | Default | Meaning |
|---|---|---|
| `CW` | 400 mm | corridor width |
| `RW` | 160 mm | robot width at its widest |
| `RL` | 230 mm | robot length |
| `AX` | 130 mm | front sensor to axle |
| `SS` | 105 mm | distance between the two side sensors |
| `SA` | 60 mm | side sensors behind the front sensor |

### Motors

| Key | Default | Meaning |
|---|---|---|
| `MN` | 95 | stall floor — below this the wheels buzz but do not turn |
| `BL` | 135 | base speed |
| `MX` | 230 | ceiling after trim |
| `LT` / `RT` | 100 / 120 % | per-motor trim for mismatched motors |
| `TP` | 150 | pivot speed |
| `TM` | 8.0° | how early the pivot sweep stops, leaving room for the coast |
| `CP` | 165 | creep PWM — must break static friction from rest |

### Control

| Key | Default | Meaning |
|---|---|---|
| `KP` | 4.5 | PWM counts per degree of heading error |
| `KD` | 0.35 | damping against yaw rate |
| `KW` | 0.060 | degrees of target tilt per mm off centre |
| `WT` | 8.0° | ceiling on that tilt |

### Navigation

| Key | Default | Meaning |
|---|---|---|
| `FB` | 300 mm | distance at which a wall ahead is believed |
| `FD` | 0 (auto 70) | how close it parks before pivoting |
| `OP` | 280 mm | beyond this a side counts as open |
| `WE` | 0 (auto 87) | side distance triggering the ramped steer-away |
| `SP` | 200 mm/s | measured travel speed; sets timed approach duration |

### Other

`MODE`, `GS` (gyro sign — turning left must make `Hdg` go positive), `LOG`,
`PATH`, `START`, `STOP`, `CAL`, `MENU`, `RESCAN`.

### Compile-time

`CONTROL_TICK_MS` 20, `CORR_RATIO_PCT` 40, `YAW_GOVERNOR_DPS` 60,
`GYRO_LSB_PER_DPS` 65.5, `OPENING_CONFIRM` 5, `FRONT_CONFIRM` 2,
`FRONT_VOTE_THRESHOLD` 2 of 5, `TOF_LATCH_MS` 400, `RECOVER_MS` 400,
`CREEP_KP` 3.0, `WALL_TILT_SLEW_DEG` 2.0.

---

## 13. The shortest-path mode: what was built and why it failed

### The idea

Solve the maze once by the right-hand rule, recording the turns. Then reduce
that string by eliminating dead ends, and drive the reduced route. No grid, no
coordinates, no distance measurement — the turn string alone is enough.

This is a standard technique (LSRB / dead-end elimination), and it is provably
correct for a maze with no loops.

### The rule, and why it works

A `U` in the path means "drove into a dead end and came back". The move
*before* it and the move *after* it happened at the **same junction** —
nothing else was recorded in between, which is exactly what makes them
adjacent in the string. So the three collapse into the single move that skips
the dead end.

Write each move as a quarter turn, counter-clockwise positive:

```
S = 0      L = +1      U = +2      R = +3  (= -1)

a U b   ->   (a + 2 + b) mod 4
```

That one line generates the entire nine-case table:

| | `L` | `S` | `R` |
|---|---|---|---|
| **`L` U** | S | R | U |
| **`S` U** | R | U | L |
| **`R` U** | U | L | S |

Worked example: `R U S` = `3 + 2 + 0 = 5 mod 4 = 1` = `L`. Arrive at a
junction, poke right into a dead end, come back, carry straight on — the net
effect is identical to just turning left. Repeat to a fixpoint.

A sentinel `S` is appended before reducing, because the run ends by driving
out of the exit, which is not a junction and records nothing. Without a move
after the final `U` there is nothing to fold it into and the last dead end
survives. The sentinel is stripped afterwards.

### What was verified

`test_path.cpp` checks all nine collapses against a table written out by hand,
independently of the arithmetic — plus idempotence, cascading collapses,
leading-`S` preservation, buffer limits, and correct rejection of an
irreducible `U`. **All pass. The algorithm is not the problem.**

`test_replay.cpp` checks the speed run spends exactly one symbol per junction
and stops rather than turning into a wall when the route stops matching. It
caught two real bugs before they reached the robot:

1. `replayAtJunction()` spent a symbol when handed a plain corridor — safe
   only because its one caller happened to guard the call.
2. The one-action-per-junction latch covered only the straight branch. Passing
   a junction straight consumed the `S`, then a tick later — with the junction
   still in the side sensors — consumed the **next** symbol at that same
   junction. A turn leaves `ST_DRIVING` immediately so it could not repeat,
   which is exactly what hid the asymmetry.

### What failed

On the real maze, **`MODE:3` did not record an `S` at every junction it passed
straight through.** Several were missing. The reduced route therefore did not
correspond to the maze, and `MODE:4` could not follow it.

### Why — the root cause

**The path string was produced as a side-effect of the decision logic, not by
a junction counter.** `recordTurn('S')` lives in exactly one branch:

```c
// ST_DRIVING, and only ST_DRIVING
if (j != J_CORRIDOR) {
    chooseTurn(j, turn_now, right);
    if (turn_now)  { ... enterState(ST_APPROACHING); }
    else           { if (!s_junction_acted) { recordTurn('S'); ... } }   // <-- here
}
```

`chooseTurn()` returns `turn_now == false` for exactly one junction type,
`J_FWD_OR_LEFT` (front open, left open, right closed). Every *other* way the
robot can physically pass a junction bypasses that line. There are at least
six:

**G1 — the robot is not in `ST_DRIVING`.** Junctions are only classified in
`ST_DRIVING` and partly in `ST_CONFIRM_EXIT`. The robot also drives forward in:

- `ST_APPROACHING` — from detection until it parks, potentially 300 mm+
- `ST_RECOVERING` — 400 ms immediately after **every single pivot**, with the
  ToF filters deliberately flushed
- `ST_CONFIRM_EXIT` — 1 500 ms

A junction encountered in any of these is invisible. `ST_RECOVERING` is the
worst offender: it runs right after every turn, which in a tight maze is
exactly where the next junction is.

**G2 — the came-from block hides junctions.** For up to 4 s after a turn, one
side is forced closed. That is correct for the corridor just left, but the
release is time-and-wall based, not position based, so the block can still be
armed when the robot reaches the *next* junction. A junction whose only
opening is on the blocked side classifies as `J_CORRIDOR` — completely
invisible.

**G3 — `J_ALL_OPEN` mid-maze records nothing.** A four-way junction, or a T
where both sides open while the front is still clear, routes to
`ST_CONFIRM_EXIT`. If it turns out not to be the exit, the robot returns to
`ST_DRIVING` having driven 1.5 s past the junction, with no symbol recorded
and the junction now behind it.

**G4 — the "no turning here after all" path.** In `ST_LOOKING`, if the front
is open and no side opening confirms, the robot logs *"no turning here after
all — carrying straight on"*, clears the counters and goes to `ST_RECOVERING`.
It detected a junction, drove to it, stopped at it, and recorded nothing.

**G5 — the latch merges adjacent junctions.** `s_junction_acted` only clears
when `classify()` returns `J_CORRIDOR`. Two junctions close enough that one
opening is still in view as the next appears produce **one** symbol between
them.

**G6 — no confirmation threshold is right for both jobs.** A side must read
open for 5 consecutive ticks (100 ms). An opening entered at an angle, or one
whose far wall is dark or oblique, can fail to hold that. Lowering the
threshold was already tried — at 2 ticks (40 ms) it produced *phantom*
junctions that sent the robot through the whole approach-park-look sequence in
the middle of a clean corridor. There is no single value that both rejects
noise and never misses a real opening.

### The honest assessment

The reduction was tested exhaustively. The **recording** was never tested at
all, because it can only be validated against a real maze — and it was the
recording that was wrong. The algorithm was proven; its input was not. Unit
tests on a transformation say nothing about whether the data reaching it is
faithful.

The design error was making the path a by-product of decisions rather than an
independent observation.

---

## 14. How to finish the shortest-path mode

The reduction code (`reducePath()` in the sketch, `test_path.cpp`) is correct
and can be kept as-is. **All the remaining work is in the data source.** Four
approaches, in increasing order of both effort and reliability.

### Step 0 — validate before driving (do this first, costs nothing)

Before any further code, make the recording *observable and checkable*:

- Print every junction event over Bluetooth with a sequence number, what the
  three sensors saw, and what the robot did:
  ```
  J#04  F:oo  L:open  R:wall  -> S     (t=12.4s)
  J#05  F:250 L:wall  R:open  -> R     (t=14.1s)
  ```
- Draw the maze on paper and number its junctions.
- Drive `MODE:3` and compare the printed list to the drawing, junction by
  junction.
- Do it twice. Only if the lists match the maze *and* each other is the
  recording trustworthy.

This is the step that was skipped. It would have found the problem in one run
instead of after building the whole speed-run mode.

### Approach 1 — plug the six gaps (smallest change, still fragile)

- Classify and record in **every** forward-driving state, not just
  `ST_DRIVING` — at minimum add `ST_RECOVERING`, `ST_APPROACHING` and
  `ST_CONFIRM_EXIT`.
- Record a symbol for a mid-maze `J_ALL_OPEN` and for the "no turning here
  after all" path.
- Release the came-from block on *distance travelled* or *heading change*
  rather than elapsed time.

This would help substantially. It would not be provably correct, because the
recording would still be spread across the decision logic.

### Approach 2 — a junction recorder decoupled from decisions (the right fix)

Write one piece of code whose **only** job is to detect that a junction began
and that it ended. Run it every tick, in every state where the robot is moving
forward. It makes no decisions and consults none.

On "junction ended", emit one record:

```c
struct JunctionEvent {
    uint8_t exits;      // bit 0 front, bit 1 left, bit 2 right
    char    taken;      // 'S' 'L' 'R' 'U'
};
```

Crucially, derive `taken` from the **gyro** — the net heading change between
junction entry and exit, rounded to the nearest 90° — not from the decision
variable. Then the move is recorded correctly no matter which code path made
it, including paths nobody anticipated. The gyro is already integrated
continuously in `loop()`, so the data is there.

This removes G1, G3, G4 and most of G2 by construction, because there is no
longer a branch that can be missed.

### Approach 3 — record the junction *signature*, making replay self-correcting

This is the single highest-value change, and it composes with Approach 2.

Store the `exits` bitmask alongside each move. On the speed run, compare what
the sensors actually see at each junction against what the record says should
be there:

- **Match** → execute the move, advance.
- **Mismatch** → search forward and back two entries for a signature that does
  match, and resynchronise. Only stop if none does.

This converts "the count must be exactly right, forever, or everything after
is wrong" into an ordinary sequence-matching problem with error recovery. One
miscounted junction becomes recoverable instead of fatal. Given that G6 means
*some* detection error is unavoidable, this is what makes the whole approach
viable on real hardware.

### Approach 4 — add wheel encoders (removes the entire problem class)

Two slotted-optical or magnetic encoders, inexpensive, mounted on the output
shafts.

With distance you no longer need to infer topology from turn sequences at all.
You can build an actual occupancy map, run flood-fill, and compute a **true**
shortest path — not merely dead-end elimination, which only removes branches
the robot happened to explore and cannot find a shortcut it never drove.

Encoders would also fix, as a side effect:

- parking accuracy (no more creep-pulse guessing)
- speed calibration — `SP` becomes measured, not assumed
- the battery-voltage dependence of `MN`, `BL` and `SP`
- stall detection, which currently relies on the gyro and the front sensor

This is what every serious micromouse does, and it is the honest
recommendation if the project is ever resumed.

### Recommended order

**0 → 3 → 2 → 4.** Validate first; make replay tolerant before making
recording perfect, since perfection is not achievable with three ToF sensors
and no odometry; then fix the recorder properly; then add encoders if a true
shortest path is wanted rather than a dead-end-free one.

---

## 15. Open issues

**Front ToF reliability.** The front sensor jumps from out-of-range straight
to ~316 mm, blind between 1 200 and 316. Suspects: the 20 ms timing budget
(short-range profile) and the practice of discarding all non-zero-status
readings. `VL53L0X_SENSE_LONG_RANGE` exists in the library and was never
tried. Diagnosis path: in `MODE:0`, watch `F` while walking a wall toward the
robot — `--` means the sensor never initialised (wiring/XSHUT; swap the front
and left modules to find out which is at fault), `oo` right up to contact
means a configuration problem, numbers that freeze mean power or a loose
connection under vibration.

**The gyro full-scale write is unchecked.** `mpuWrite(0x1B, 0x08)` selects
±500 °/s. Its return value is never examined. If it fails the MPU stays at
±250 °/s and **every angle reads exactly 2× too high** — turns would overshoot
by a factor of two while the log reports them as perfect. Test: rotate the
robot by hand through exactly 90° in `MODE:0` and read `Hdg`. ~90 means the
scale is right; ~180 means the write failed. *This was never performed and
should be the first check if turning behaviour is ever suspect.*

**Creep occasionally exhausts its pulses.** It sometimes hits the 6-pulse
limit and parks at 46–56 mm against a 70 mm target — the reverse pulses are
not moving the robot. `CP` (165) is probably below what breaks static friction
in reverse on this surface.

**Battery voltage shifts everything.** `MN`, `BL` and `SP` all change
measurably between a fresh and a half-drained pack — `MN` was observed moving
from 95 to 75 on fresh cells. The proper fix is an ADC voltage divider on the
motor pack and scaling the PWM values by measured voltage, so one calibration
holds all session. Not implemented.

**`MODE:4` does not work.** See §13.

---

## 16. File map

```
Phase5_Integrated/
├── Phase5_ESP32_Brain/
│   └── Phase5_ESP32_Brain.ino     2613 lines — master firmware
├── Phase5_ATmega_Slave/
│   └── Phase5_ATmega_Slave.c       325 lines — motor driver, framed protocol
├── tools/
│   └── maze_monitor.py             463 lines — PC Bluetooth monitor
├── test/
│   ├── run_tests.sh                       — host regression suite (64 checks)
│   ├── test_common.cpp                    — ToF near-wall latch
│   ├── test_junction.cpp                  — junction classification
│   ├── test_creep.cpp                     — creep-to-park
│   ├── test_frontwall.cpp                 — front wall + heading stability
│   ├── test_path.cpp                      — path reduction
│   ├── test_replay.cpp                    — speed-run symbol accounting
│   └── stub/                              — Arduino, Wire, BluetoothSerial,
│                                            VL53L0X, Preferences stubs
└── README.md                              — bring-up guide, parameter reference
```

**Bring-up order** (documented fully in `Phase5_Integrated/README.md`):
`MODE:0` sensors → `MODE:2` one pivot, tune `TM` → `MODE:1` straight, tune
`KP`/`KD`/trims → measure `SP` → `MODE:3` maze.

Run the tests with `./Phase5_Integrated/test/run_tests.sh` — no robot needed.

---

*Report generated 29 September 2026. Firmware at commit `227f753` on branch
`claude/vibrant-curie-uhsy3h`.*
