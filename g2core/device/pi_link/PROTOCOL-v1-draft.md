# Pi link protocol, v1 (motion) - DRAFT

Status:
- **Done:** the bench build. It has the control handshake, watchdog, board stop, USB
  read-only mode and diagnostics. Built without `PI_LINK_MOTION=1`, it holds every motor at
  zero velocity, so it can't move the machine.
- **Not done:** motion, and spindle control over the link.

v0 (PROTOCOL.md: framing, PING, STATUS) stays as it is; v1 adds frame types. The frame
version stays 0 until v1 ships.

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
     |       stop finished      v  g2core alarm/shutdown, USB stop, bad frame
     +--------------------- STOPPING
```

- **USB** (power-up default): g2core as today. MOTION and DISABLE get result 5 (not in
  control).
- **PI**: the Pi drives the motors.
  - USB JSON reads and status reports keep working. Status reports show `stat` 5 (run),
    with positions following the step counters every 10 ms.
  - USB G-code, JSON writes and `$` settings get error 106 (`STAT_COMMAND_NOT_ACCEPTED`).
  - USB `!` (feed hold), `%` (queue flush) and ^D (job kill) start a board stop (reason 7).
    `~` (cycle start) is ignored. ^X still resets the board.
  - Frames from the Pi are read and answered in the exec interrupt, once per segment, so a
    slow USB reply (formatting `{"sys":n}` holds the main loop for about 5 ms) can't delay
    them. While USB has control the main loop reads them, as in v0.
  - All motors stay energized at full current, as during a USB job. g2core's idle-current
    reduction resumes once control is back with USB.
- **STOPPING**: a board stop is running (see below). It then returns to USB and records
  the reason.

ENABLE is accepted only if all of these hold:
- g2core is idle:
  - stat 1, 3 or 4;
  - no homing, probe or jog cycle and no feed hold;
  - planner and steppers empty;
  - spindle off.
- The E-stop is clear and acknowledged (`estp` 0).
- g2core is not in alarm, shutdown or panic.

An open door is allowed: stock firmware lets the machine move with the door open as long
as the spindle is off.

When control returns to USB, the board does two things:
- It folds any steps not yet counted into the step counters.
- It resets g2core's model, planner and runtime positions from the counters (forward
  kinematics + `cm_set_position_by_axis()`, as homing does). Without that, the next USB
  G-code move would start from where g2core last thought the machine was.

Homed flags are kept, except after an E-stop (g2core clears them itself).

## Board stop

The board's own stop, used whenever it ends Pi control while the motors may be moving:

- Each motor ramps linearly to zero. All motors take the same time, which is set by the
  slowest one to stop at its `stop_decel`. That keeps the machine on a straight line along
  its current direction of travel.
- At the Desktop CNC's top speed (84 mm/s) and 1000 mm/s^2 that takes about 84 ms and
  3.5 mm.
- The ramp ends with one all-zero segment. Loading that segment folds the last moving
  segment's steps into the counters.
- Spindle (once spindle control is in): stopped at the start of every board stop, except
  DISABLE, where the spindle is the host's responsibility.

| Trigger | Reason code |
| --- | --- |
| DISABLE from the host | 1 |
| Watchdog: no valid MOTION for `watchdog_ms` | 2 |
| E-stop pressed (g2core also shuts down: spindle off, axes unhomed) | 3 |
| Door opened with the spindle on, and the host didn't stop in time | 4 |
| g2core alarm, shutdown or panic | 5 |
| Bad MOTION frame (wrong length, NaN or infinite velocity, spindle request in a build without spindle control) | 6 |
| USB `!`, `%` or ^D | 7 |
| Fault: the segment chain stopped for 20 ms (something reset the steppers) | 8 |

## Door interlock while the Pi has control

Not in the bench build: it needs spindle control over the link.

- **Spindle off:** reported only (`safe` bit 0), as in stock firmware.
- **Spindle on:**
  - The board stops the spindle at once (the ESC loses power shortly after anyway) and sets
    `hold_request` in FEEDBACK.
  - The host should pause (LinuxCNC feed hold), so the machine decelerates along its path
    and can resume.
  - If the motors have not all reached zero velocity within `hold_timeout_ms` of the door
    opening, the board stops on its own (reason 4).
  - Once they have, the hold is satisfied and `hold_request` clears. Motion with the door
    open is then allowed again, as in stock firmware with the spindle off. That allows a Z
    lift off the work.
  - The spindle can't restart until the door is closed and the ESC has rebooted (5 s), as
    in stock firmware.
- **Lift and resume rules belong to the LinuxCNC config, not the board.** The config must
  include all of these:
  - **Resume only when all three hold:** door closed, ESC ready, and spindle at speed (from
    FEEDBACK `safe` and the actual spindle speed). Until then, `motion.feed-inhibit` is held
    true.
    - LinuxCNC's own wait for spindle-at-speed only covers the first feed move after a speed
      change, so it doesn't reliably cover a resume.
  - **Lift:** after the hold, raise Z by a few mm with an external offset on Z (or QtDragon's
    built-in spindle-lift pause). Returning to the paused point removes the offset.
  - Stock g2core can do the same on the USB path: `zl` (lift on feedhold) followed by the
    spindle spin-up delay on resume. The Desktop CNC settings ship `zl` = 0, i.e. no lift;
    the Othermill used -1, i.e. go to Z max.

## Timing

- The board runs segments of `segment_us` (default 500 µs) on its own clock.
  - Segments are whole DDA ticks: 500 µs becomes 31 ticks at 61 kHz, which is 508 µs.
    Steps are computed for that real length, so velocities stay exact.
  - With 1 ms frames, each command covers about two segments: the first ramps to the new
    velocity, the second holds it.
- Each segment takes the newest velocity command:
  - start velocity = the end velocity of the previous segment;
  - end velocity = the command, clamped;
  - steps = the trapezoid area.
- This gives continuous velocity, using `st_prep_line()`'s per-motor ramp form.
  - A motor whose velocity changes sign within a segment runs that segment at constant
    velocity, because the DDA can't reverse mid-segment.
  - A ramp from exactly zero is started at 1e-6 of the end rate. A zero starting rate makes
    g2core's loader treat the motor as idle and drop the segment's steps.
- g2core prepares the next segment while the current one runs, so a command reaches the
  motors 1 to 2 segments (0.5-1 ms) after it arrives. The driver's feedback loop absorbs this.
- Segment prep runs in g2core's exec interrupt. g2core's `st_prep_line()` uses software doubles
  (the Cortex-M3 has no FPU): roughly 20-30 µs per moving motor, so about 100 µs of each
  500 µs segment at worst. Since the velocities already arrive in steps/s, a leaner prep
  for this path (two multiplies per motor) can replace it if the measurement says so.
- A segment counts as stale if the newest command is more than 2 ms old. That means at least
  one 1 ms frame is missing.
- Board time comes from SysTick: milliseconds plus the count within the current millisecond.
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

The DDA runs at 61 kHz, so no motor can exceed 61 k steps/s; ENABLE refuses a `max_vel`
above 95% of that. The board also clamps each motor to `max_vel`.

## New frame types

Header, COBS and CRC are as in v0. ENABLE, MOTION, DISABLE and SAFETY are all answered with
FEEDBACK (0x91), so the driver handles one reply type. MOTION is sent every servo period
without waiting for the previous reply; the v0 one-request-in-flight rule still applies to
the other types.

### 0x10 ENABLE (host -> board), 52 bytes

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u16 | `watchdog_ms`, 2-100 (default 5) |
| 2 | u16 | `segment_us`, 500-2000 (default 500; 1000 is the fallback, no reflash needed) |
| 4 | u16 | `hold_timeout_ms`, 50-2000 (default 250) |
| 6 | u16 | reserved, 0 |
| 8 | f32 x4 | `max_vel`, steps/s per motor, > 0 and <= 0.95 x 61000 |
| 24 | f32 x4 | `stop_decel`, steps/s^2 per motor, > 0 |
| 40 | u8 x12 | reserved, 0 |

The watchdog starts at ENABLE: the first MOTION is due within `watchdog_ms`.

### 0x11 MOTION (host -> board), 24 bytes

| Offset | Type | Field |
| --- | --- | --- |
| 0 | f32 x4 | velocity, steps/s per motor |
| 16 | u8 | spindle: 0 off, 1 CW (M3), 2 CCW (M4). Must be 0 until spindle control is in. |
| 17 | u8 | reserved, 0 |
| 18 | u16 | reserved for digital outputs, 0 |
| 20 | f32 | spindle speed, RPM |

Spindle requests will go through the same ESC toolhead code and safety checks as M3/M4/M5.

### 0x12 DISABLE (host -> board), no payload

The board starts a board stop (reason 1) and replies. While a stop is already running the
reply is result 0.

### 0x13 SAFETY (host -> board), 1 byte

1 = acknowledge E-stop (as `{"estpc":n}`), 2 = clear alarm or shutdown (as `{"clear":n}`).
Only accepted while USB has control (result 6 otherwise).

### 0x03 DIAG_REQ (host -> board), no payload; reply 0x83 DIAG, 32 bytes

Counters reset at each ENABLE.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u16 | exec interrupt time, max, µs |
| 2 | u16 | exec interrupt time, last, µs |
| 4 | u32 | longest gap between main-loop passes while the Pi had control, µs |
| 8 | u32 | segments prepared |
| 12 | u16 | stale segments |
| 14 | u16 | watchdog trips (since power-up) |
| 16 | u16 | segment length, DDA ticks |
| 18 | u16 | replies dropped: both transmit buffers were still in use |
| 20 | u16 | underruns: a segment wasn't ready when the previous one ended (motors briefly ran dry) |
| 22 | u16 | frame handling part of the exec interrupt, max, µs |
| 24 | u16 | segment computation part, max, µs |
| 26 | u16 | most frames handled in one pass |
| 28 | u32 | segment number of the slowest pass |

### 0x91 FEEDBACK (board -> host), 40 bytes

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | `result` for the request answered: 0 ok, else a reason (table below) |
| 1 | u8 | owner: 0 USB, 1 PI, 2 STOPPING |
| 2 | u8 | last release reason (board stop table), 0 = none since the last ENABLE |
| 3 | u8 | flags: bit0 `hold_request`, bit1 velocity clamped, bit2 stale segment (both since the last FEEDBACK), bit3 motion locked (bench build) |
| 4 | u32 | board time, µs (wraps after about 71 minutes) |
| 8 | i32 x4 | position: steps emitted per motor at that time (wraps; the driver unwraps) |
| 24 | u16 | seq of the newest MOTION taken into a segment |
| 26 | u16 | `stale_segments` counter |
| 28 | u8 | `safe` (as STATUS) |
| 29 | u8 | `estp` (as STATUS) |
| 30 | u16 | inputs, bit (N-1) = `inN` |
| 32 | u8 | spindle direction, actual |
| 33 | u8 | g2core combined state (`stat`; 5 while the Pi has control) |
| 34 | u16 | reserved, 0 |
| 36 | f32 | spindle speed, actual |

`result` reasons:

| Code | Meaning |
| --- | --- |
| 1 | not idle (includes spindle on at ENABLE) |
| 2 | E-stop active or not acknowledged |
| 3 | alarm, shutdown or panic |
| 4 | bad parameter or bad frame |
| 5 | not in control (MOTION or DISABLE while USB has control) |
| 6 | the Pi has control (ENABLE twice, SAFETY while in control) |

## Firmware hooks (all under `#ifdef PI_LINK`)

| File | Change |
| --- | --- |
| `stepper.cpp` | Exec interrupt: `pi_link_exec_move()` instead of `mp_exec_move()` while the Pi has control. Motor power callback: skipped while the Pi has control. |
| `alarm.cpp` | `cm_is_alarmed()` returns 106 while the Pi has control. It gates every USB setting, JSON write and G-code block. |
| `controller.cpp` | `!`, `%`, ^D and `~` handling while the Pi has control. |
| `canonical_machine.cpp` | `cm_get_combined_state()` reports RUN while the Pi has control. |
| `device/pi_link/pi_link.cpp` | Everything else. |

A build without `PI_LINK` is unchanged: same size, same behaviour.

**USB interrupt priority (PI_LINK builds).** Motate's SAM3X USB driver passes the flag
`kInterruptPriorityLow` (1<<8) to `NVIC_SetPriority()`. Only 4 bits are kept, so USB runs
at level 0: the highest, shared with the step generator and above segment prep. A `$$` flood
stretched one segment prep to 390 µs. One second after boot, the link moves USB to level 3, as
intended: below the step generator (0), segment prep (1) and forward planning (2). Stock
builds are left as they are; the same fix would likely cut step jitter during USB traffic there
too.

## Test plan

1. **Bench build, no motion possible:**
   - the ENABLE and DISABLE handshake, and each rejection reason;
   - USB writes refused while the Pi has control, and `stat` 5;
   - USB `!` handing control back;
   - a watchdog trip (the host stops sending);
   - E-stop while the Pi has control, then SAFETY ack and clear over the link;
   - DIAG timing under a 10-minute 1 kHz stream.
2. **Host simulation** of the exec path (fake `st_prep_line()`): ramps, sign changes, the
   board stop and step accounting, before the motion build.
3. **Slow motion, one motor** (you present, door closed): feedback steps against commanded
   velocity x time. All four motors at a slow speed give the worst-case exec time.
4. **Board stops during slow motion:** watchdog, DISABLE, E-stop, USB `!`, and the door with
   the spindle on.
5. **After a Pi session:** USB G-code moves start from the right place.
6. **LinuxCNC HAL driver:** its own spec. It reads the previous FEEDBACK without blocking,
   updates `pos-fb`, computes velocities and sends MOTION, all in the servo thread.

## Checks before fast motion

- Worst-case exec interrupt time below 25% of the segment (about 123 µs), with all four
  motors moving. Otherwise use the leaner prep, or 1000 µs segments.
- The exec interrupt now also reads and answers frames, so its time includes that: still
  below 25% of the segment. (The main loop can stall for ~5 ms on a long USB reply; that no
  longer affects the link, which is why frame handling moved out of it.)
- Zero stale segments and zero receive errors over 10 minutes of 1 kHz MOTION frames.

## Decisions (2026-10-20)

1. **Segment length: 500 µs**, provided the checks above pass.
2. **Watchdog: 5 ms**, i.e. five missed 1 ms frames:
   - The Pi's measured real-time jitter is about 0.2 ms worst case under load, so the
     watchdog doesn't trip on normal jitter.
   - At top speed the machine travels about 0.4 mm before the stop begins.
   - LinuxCNC itself reports a realtime fault at stalls of that size.
3. **USB while the Pi has control:** `stat` 5 with live positions, and writes refused with
   error 106 rather than queued. A queued move would otherwise run, unexpectedly, after the
   Pi hands control back. (Confirmed.)
4. **Door with the spindle on: cooperative pause, resumable.**
   - LinuxCNC feed-holds, the board stops the spindle, and Z lifts.
   - The resume waits for door closed, ESC ready and spindle at speed (all three are
     required).
   - A false door signal therefore costs a pause, not the job. Only the board's 250 ms
     backstop (LinuxCNC failed to pause) ends the program.
5. **Probe: no latch in v1.**
   - One step is already 5 µm on X/Y and 2.5 µm on Z, so per-period sampling at
     slow-touch speeds (about 1-2 µm at 1 mm/s) is below the machine's resolution.
   - Most of that error is a fixed lag, which cancels when tools are touched off with the
     same probe routine.
   - LinuxCNC's motion module can't take a latched position from a driver anyway; it
     records its own position in the period the probe input changes.
   - Use a fast seek, back off, then a slow final touch.
