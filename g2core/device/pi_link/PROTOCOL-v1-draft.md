# Pi link protocol, v1 (motion) - DRAFT for review

Status: design only, nothing here is implemented yet. v0 (PROTOCOL.md: framing, PING,
STATUS) stays as it is; v1 adds frame types for motion. Frame version stays 0 until v1
ships.

## Goals

- LinuxCNC on the Pi drives the four motors through g2core's existing step generator.
- USB behaviour is unchanged unless the Pi has taken control.
- Safety stays on the board: E-stop, the door interlock and a link watchdog stop the
  machine without help from the Pi.

## Control model: velocity commands, position feedback

Every servo period (1 ms) the Pi sends a velocity for each motor, in steps/s. The board
replies with the exact number of steps each motor has emitted. The LinuxCNC driver closes
the position loop: `vel = feed-forward + P x (position command - position feedback)`.

This is how Mesa stepgens and Remora work. Compared with sending target positions:

- Clock drift between the Pi and the board doesn't matter, because the feedback corrects it.
- A late or lost frame costs no position: the board keeps the last velocity, and the next
  feedback shows where the motors really are.
- The board has no trajectory to plan; it plays velocities into g2core's segment queue.

## Ownership

```
            ENABLE accepted
    USB  ------------------->  PI
     ^                          |  DISABLE, watchdog, E-stop, door timeout,
     |       stop finished      v  g2core alarm/shutdown
     +--------------------- STOPPING
```

- **USB** (power-up default): g2core as today. MOTION frames are ignored; the reply says
  the Pi is not in control.
- **PI**: the Pi drives the motors.
  - USB JSON reads and status reports keep working.
  - USB G-code and JSON writes get error 106 (`STAT_COMMAND_NOT_ACCEPTED`).
  - All motors stay energized.
- **STOPPING**: a board stop is running (see below). It then returns to USB and records
  the reason.

ENABLE is accepted only if all of these hold:
- g2core is idle: stat 1, 3 or 4, planner and steppers empty, no homing, probe or jog
  cycle running;
- the E-stop is clear (`estp` 0);
- g2core is not in alarm, shutdown or panic.

An open door is allowed: stock firmware lets the machine move with the door open as long
as the spindle is off.

When control returns to USB, the board resets g2core's position from the step counters
(`kn_forward_kinematics`, the same path probing uses). Without that, the next USB G-code
move would start from where g2core last thought the machine was. Homed flags are kept,
except after an E-stop (g2core clears them itself).

## Board stop

The board's own stop, used whenever it ends Pi control while the motors may be moving:

- Each motor ramps linearly to zero. All motors take the same time, which is set by the
  slowest one to stop at its `stop_decel`. That keeps the machine on a straight line along
  its current direction of travel.
- At the Desktop CNC's top speed (84 mm/s) and 1000 mm/s^2 that takes about 84 ms and
  3.5 mm.
- Spindle: stopped at the start of every board stop. The one exception is DISABLE, where
  the spindle is the host's responsibility.

| Trigger | Reason code |
| --- | --- |
| DISABLE from the host | 1 |
| Watchdog: no valid MOTION for `watchdog_ms` | 2 |
| E-stop pressed (g2core also shuts down: spindle off, axes unhomed) | 3 |
| Door opened with the spindle on, and the host didn't stop in time | 4 |
| g2core alarm, shutdown or panic | 5 |
| Bad MOTION frame (NaN, wrong length) | 6 |

## Door interlock while the Pi has control

- **Spindle off:** reported only (`safe` bit 0), as in stock firmware.
- **Spindle on:**
  - The board stops the spindle at once (the ESC loses power shortly after anyway) and sets
    `hold_request` in FEEDBACK.
  - The host should pause (LinuxCNC feed hold), so the machine decelerates along its path
    and can resume.
  - If any velocity is still non-zero `hold_timeout_ms` after the door opened, the board
    stops on its own (reason 4).
  - The spindle can't restart until the door is closed and the ESC has rebooted (5 s), as
    in stock firmware.

## Timing

- The board runs segments of `segment_us` (default 1000 µs) on its own clock.
- Each segment takes the newest velocity command:
  - start velocity = the end velocity of the previous segment;
  - end velocity = the command, clamped;
  - steps = the trapezoid area.
- This gives continuous velocity, using `st_prep_line()`'s per-motor ramp form.
  - A motor whose velocity changes sign within a segment runs that segment at constant
    velocity, because the DDA can't reverse mid-segment.
- g2core prepares the next segment while the current one runs, so a command reaches the
  motors 1 to 2 segments (1-2 ms) after it arrives. The driver's feedback loop absorbs this.
- If no new command arrived since the last segment, the previous one is reused and
  `stale_segments` counts it.
- Link budget at 2 Mbaud: MOTION ~34 bytes on the wire, FEEDBACK ~50. That's about 0.4 ms
  of a 1 ms period, and the link is full duplex.

## Units and the Desktop CNC numbers

- Motor 1-4 = X, Y, Z, A. The units are steps, steps/s and steps/s^2.
- Positive = the axis's positive direction under g2core's motor polarity (`$1po`..`$4po`).
- LinuxCNC's `SCALE` converts to mm and degrees. g2core's axis settings (`$xvm`, jerk, ...)
  are not used in Pi mode.

| Motor | Steps per unit | Max speed (stock) | Max step rate |
| --- | --- | --- | --- |
| 1 X | 200 /mm | 84 mm/s | 16.9 k/s |
| 2 Y | 200 /mm | 84 mm/s | 16.9 k/s |
| 3 Z | 400 /mm | 33 mm/s | 13.3 k/s |
| 4 A | 17.78 /deg | 33 deg/s | 0.6 k/s |

The DDA runs at 61 kHz, so no motor can exceed 61 k steps/s. The board also clamps each
motor to `max_vel` from ENABLE.

## New frame types

Header, COBS and CRC are as in v0. Every request below is answered with FEEDBACK (0x91),
so the driver handles one reply type. MOTION is sent every servo period without waiting
for the previous reply; the v0 one-request-in-flight rule still applies to the other types.

### 0x10 ENABLE (host -> board), 52 bytes

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u16 | `watchdog_ms`, 2-100 (suggest 5) |
| 2 | u16 | `segment_us`, 500-2000 (suggest 1000) |
| 4 | u16 | `hold_timeout_ms`, 50-2000 (suggest 250) |
| 6 | u16 | reserved, 0 |
| 8 | f32 x4 | `max_vel`, steps/s per motor |
| 24 | f32 x4 | `stop_decel`, steps/s^2 per motor, > 0 |
| 40 | u8 x12 | reserved, 0 |

### 0x11 MOTION (host -> board), 24 bytes

| Offset | Type | Field |
| --- | --- | --- |
| 0 | f32 x4 | velocity, steps/s per motor |
| 16 | u8 | spindle: 0 off, 1 CW (M3), 2 CCW (M4) |
| 17 | u8 | reserved, 0 |
| 18 | u16 | reserved for digital outputs, 0 |
| 20 | f32 | spindle speed, RPM |

Spindle requests go through the same ESC toolhead code and safety checks as M3/M4/M5.

### 0x12 DISABLE (host -> board), no payload

The board starts a board stop (reason 1) and replies.

### 0x13 SAFETY (host -> board), 1 byte

1 = acknowledge E-stop (as `{"estpc":n}`), 2 = clear alarm or shutdown (as `{"clear":n}`).
Only accepted while USB has control.

### 0x91 FEEDBACK (board -> host), 40 bytes

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | `result` for the request answered: 0 ok, else a reason (table below) |
| 1 | u8 | owner: 0 USB, 1 PI, 2 STOPPING |
| 2 | u8 | last release reason (board stop table), 0 = none since the last ENABLE |
| 3 | u8 | flags: bit0 `hold_request`, bit1 velocity clamped this period, bit2 stale segment since last FEEDBACK |
| 4 | u32 | board time, µs (wraps after about 71 minutes) |
| 8 | i32 x4 | position: steps emitted per motor at that time (wraps; the driver unwraps) |
| 24 | u16 | seq of the MOTION now playing in the running segment |
| 26 | u16 | `stale_segments` counter |
| 28 | u8 | `safe` (as STATUS) |
| 29 | u8 | `estp` (as STATUS) |
| 30 | u16 | inputs, bit (N-1) = `inN` |
| 32 | u8 | spindle direction, actual |
| 33 | u8 | g2core combined state (`stat`) |
| 34 | u16 | reserved, 0 |
| 36 | f32 | spindle speed, actual |

`result` reasons:

| Code | Meaning |
| --- | --- |
| 1 | not idle |
| 2 | E-stop active or not acknowledged |
| 3 | alarm, shutdown or panic |
| 4 | bad parameter |
| 5 | not in control (MOTION or DISABLE while USB has control) |
| 6 | already in control (ENABLE twice) |

## Firmware changes this needs

1. `stepper.cpp` exec interrupt: while the Pi has control, call `pi_link_exec_move()`
   instead of `mp_exec_move()`.
2. `pi_link.cpp`:
   - the ownership state machine;
   - a double-buffered latest command (the main loop writes it, the exec interrupt reads it);
   - the board-stop ramp generator;
   - FEEDBACK built from the step counters (`encoder_steps` + `steps_run`, read with
     interrupts briefly off);
   - a µs clock from the Cortex-M3 cycle counter.
3. Reject USB writes while the Pi has control, at the command dispatcher.
4. Keep the motors energized while the Pi has control. Normal power timeouts resume
   afterwards.
5. Re-sync g2core's position on release.

## Test plan

1. **No motion:** the ENABLE and DISABLE handshake, every rejection reason, USB writes
   refused while the Pi has control, and a watchdog trip at zero velocity.
2. **Host simulation:** the command buffer and board-stop ramp, as with the v0 receive ring.
3. **Slow motion, one motor** (you present, door closed): feedback steps against commanded
   velocity x time.
4. **Board stops during slow motion:** watchdog (the host stops sending), DISABLE, E-stop,
   and the door with the spindle on.
5. **After a Pi session:** USB G-code moves start from the right place.
6. **LinuxCNC HAL driver:** its own spec. It reads the previous FEEDBACK without blocking,
   updates `pos-fb`, computes velocities and sends MOTION, all in the servo thread.

## Open questions

1. **Segment length:** 1000 µs (simpler, 1-2 ms latency) or 500 µs (half the latency,
   about twice the prep CPU). Suggest starting at 1000.
2. **Watchdog default:** 5 ms means about 0.4 mm of travel at top speed before the stop
   begins.
3. **What USB shows while the Pi has control:**
   - `stat` unchanged plus a new read-only key for the owner, or
   - force `stat` 5 (run), so a USB app sees the machine as busy.
4. **Door with the spindle on:** the cooperative pause above (resumable), or have the board
   stop at once (simpler, but the LinuxCNC program is lost).
5. **Probe latch:** report the exact step position at the probe edge in v1, or rely on
   LinuxCNC's per-period sampling (about 2 µm error at 1 mm/s) until it's needed.
