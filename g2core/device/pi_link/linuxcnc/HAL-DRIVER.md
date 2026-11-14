# LinuxCNC HAL driver for the Pi link (`pilink`)

Status:
- **Done:** the driver (`pilink.c`) and a slow first machine config for board 2's Desktop CNC
  (`bantam-pilink.ini`, `bantam-pilink.hal`, AXIS).
- **Bench results (board 2, 2026-10-21, firmware 101.17-13-g3a4d):** everything that works
  without moving X, Y or Z passes. Motor 4 had no motor attached then, so it carried the motion
  tests. Since 2026-10-22 the 4th axis is mounted: `bench-motor4.sh` won't take control unless
  `MOTOR4_FREE=1` says motor 4 is unplugged.
- **On the machine since:** X/Y/Z/A motion and Home All on the real switches (2026-10-21/22);
  spindle M3/M5/S, M4 refused, door stop with the spindle on (2026-10-26); M3 with the door
  open refused, and E-stop with the spindle on, with F1 recovery (2026-11-13).
- **Door hold** (pause and resume, below): on the machine 2026-11-13, in a running program:
  the door opening paused it with the spindle stopped (twice, no fault), Resume with the door
  open stayed paused, nothing moved until Resume, then the spindle came back and the program
  finished. Also passes `test/run.sh` and the firmware simulation. Not yet checked in MDI.
- **Not done:** probing through LinuxCNC; a Z lift during the door hold.

## Files

| File | What |
| --- | --- |
| `pilink.c` | Realtime HAL component, plain C, built with `halcompile` |
| `bantam-pilink.ini` | Machine config: 20 mm/s on X/Y, 10 mm/s on Z, 20 deg/s on A |
| `bantam-pilink.hal` | Wiring: joints, home switches, probe, E-stop chain, spindle, door hold |
| `tool.tbl` | One tool: the 1/4" probe |
| `test/run.sh` | Host test of the door hold: `pilink.c` against stub HAL headers and a model of the board, over a pseudo-terminal |

## Install and start (on the Pi)

```
cd ~/linuxcnc/configs/bantam-pilink
sudo halcompile --install pilink.c          # once, and again after any change to pilink.c
linuxcnc ~/linuxcnc/configs/bantam-pilink/bantam-pilink.ini
```

Then in AXIS: F1 (E-stop reset), F2 (machine on: the Pi takes control and the motors energize),
Home All (Z first, then X and Y), and jog.

- Don't run `pi_link_test.py` while LinuxCNC is running: both use `/dev/ttyAMA0`.
- Without installing: rtapi_app only loads modules from its own directory, and `halcmd`
  waits for a component named like the module argument. With a symlink `~/p` to the config
  directory, `loadrt ../../../../home/<user>/p/pilink hal_name=../../../../home/<user>/p/pilink`
  works. The bench tests used this.

## How it works

Two functions in the servo thread (1 kHz):

| Order | Function | Job |
| --- | --- | --- |
| first | `pilink.read` | Reads whatever FEEDBACK has arrived (non-blocking), unwraps the counts, updates `position-fb`, inputs and board state |
| between | `motion-command-handler`, `motion-controller` | LinuxCNC computes the new `motor-pos-cmd` |
| last | `pilink.write` | Computes one velocity per motor and sends one MOTION |

- **Polling:** while LinuxCNC has the machine off, the driver sends a zero MOTION every 10 ms.
  The board answers with result 5 (not in control) and a full FEEDBACK, so positions, inputs
  and the E-stop stay live, including while USB moves the machine.
- **Control:** machine on sends ENABLE (`segment_us` 1000, watchdog 5 ms, hold 250 ms,
  `max_vel` = `board-maxvel` or 1.5 x `maxvel`, `stop_decel`), then one MOTION per period.
  Machine off sends DISABLE; the board stops on its own (straight line, `stop-decel`).
- **Position loop:** `velocity = feed-forward + p-gain x (reference - feedback)`.
  - The feed-forward is motion's commanded velocity for this period.
  - The reference is the commanded position `latency` periods before the period in which the
    MOTION answered by the newest FEEDBACK was sent. FEEDBACK echoes that MOTION's seq, so
    a late reply is still compared with the right command.
  - A 0.5-step deadband stops hunting at rest. `maxvel` and `maxaccel` clamp the result.
  - Measured on motor 4: the counts follow the command 4.0 periods behind (2 periods of round
    trip plus the board's 2-segment pipeline), which is `latency` 2.0.
- **Feedback units:** `position-fb` = raw step count / `scale`. The counts are not g2core's
  machine coordinates (they carry an offset), so LinuxCNC homes itself every session. Inputs
  and positions come from the same FEEDBACK, so a home or probe trip is paired with the
  position at that same moment.

Driver states (`pilink.state`):

| State | Meaning |
| --- | --- |
| 0 idle | USB has control; polling |
| 1 enabling | ENABLE sent, waiting for the reply |
| 2 running | the Pi has control |
| 3 disabling | DISABLE sent, waiting for the board stop to finish |
| 4 wait-usb | the board took control back (or no reply); waiting for USB to own it again |

`pilink.fault` (wired to every joint's `amp-fault-in`, so LinuxCNC turns the machine off) is
set when:
- the board refuses ENABLE (`pilink.result` = the board's result code; 100 = no reply);
- the board takes control back: watchdog, E-stop, USB `!`, alarm, or a door hold whose motion
  wasn't at rest within `hold-ms` (`pilink.release-reason`);
- the board refuses a spindle start (door open, E-stop or alarm), outside a door hold;
- no FEEDBACK arrives for 20 ms while running (the driver then stops sending, so the board's
  watchdog stops the machine);
- `position-cmd` is more than `enable-tolerance` from `position-fb` at machine on. LinuxCNC
  holds them equal while the machine is off; if they differ, the loop would drive the motor
  there at full speed, so the driver refuses instead.
- `position-lost` is set (below).

The fault clears when the machine goes off.

**E-stop chain:** `iocontrol.0.emc-enable-in` = LinuxCNC's own chain AND `pilink.estop-ok`
(link up, `estp` 0, g2core not in alarm, shutdown or panic, positions not lost). F1 also
pulses `pilink.estop-reset`: if the board's E-stop is released but unacknowledged, the driver
sends SAFETY acknowledge, then SAFETY clear (also sent for an alarm). `estop-ok` then goes
true and LinuxCNC leaves E-stop.

**Door hold:** opening the door with the spindle running pauses the job instead of ending it.
1. The board stops the spindle and sets FEEDBACK's `hold_request` (`pilink.hold-request`).
2. The driver raises `pilink.feed-inhibit` (`motion.feed-inhibit`: G-code motion decelerates
   and stops; jogs are unaffected), pulses `pilink.pause` (`halui.program.pause`), and sends
   the spindle request off. `pilink.door-hold` = 1.
3. The board ends its hold once every motor has been commanded at rest for 10 ms. If that
   hasn't happened within `hold-ms` (250), it stops the machine instead (release reason 4).
4. Closing the door restarts nothing. Resume (`halui.program.is-paused` falling) with the door
   closed sets `pilink.door-hold` = 2 and sends the spindle request on again. The board starts
   the spindle once the ESC has rebooted (5 s after the door closes, if it was open for more
   than 0.9 s). Resume with the door open pauses the program again.
5. Once the spindle is at speed, `feed-inhibit` drops and the job goes on.

Turning the spindle off (M5, abort, machine off) ends the hold with nothing to restart. Outside a
running program (MDI, manual) there may be no pause to resume from; turning the spindle off and
on again restarts it.

**Board restart:** if FEEDBACK stops for over 500 ms and the board's clock then disagrees with
the Pi's by over 200 ms (+0.2% drift), the board has rebooted and its counts restarted.
`position-lost` latches, `estop-ok` stays false and ENABLE is refused until LinuxCNC is
restarted and re-homed.

## Pins and parameters

Per motor `N` = 0..3 (motor 1..4):

| Name | Type | Meaning |
| --- | --- | --- |
| `pilink.N.position-cmd` | float in | units |
| `pilink.N.enable` | bit in | 0 = always zero velocity |
| `pilink.N.position-fb` | float out | counts / scale |
| `pilink.N.counts` | s32 out | unwrapped steps (low 32 bits) |
| `pilink.N.velocity-cmd` | float out | units/s in the last MOTION |
| `pilink.N.error` | float out | steps, reference - feedback |
| `pilink.N.scale` | param | steps/unit (default 200, 200, 400, 17.7778) |
| `pilink.N.maxvel`, `.maxaccel` | param | the driver's clamps, units/s and units/s^2 |
| `pilink.N.board-maxvel` | param | ENABLE `max_vel`, units/s; 0 = 1.5 x `maxvel` |
| `pilink.N.stop-decel` | param | ENABLE `stop_decel`, units/s^2 |
| `pilink.N.enable-tolerance` | param | units, default 0.05 |

Global:

| Name | Type | Meaning |
| --- | --- | --- |
| `pilink.enable` | bit in | machine on (`motion.motion-enabled`) |
| `pilink.estop-reset` | bit in | F1 (`iocontrol.0.user-request-enable`) |
| `pilink.estop-ok`, `.fault`, `.in-control`, `.link-ok`, `.position-lost` | bit out | above |
| `pilink.input-01` .. `-16`, `.inputs` | out | g2core `in1`..`in16`: X home in1, E-stop in2, Y home in3, door in4, probe in5, Z home in6 |
| `pilink.door-open`, `.esc-not-ready`, `.hold-request`, `.estop-pressed` | bit out | `safe` bits 0/1, FEEDBACK flag 0, `estp` bit 0 |
| `pilink.spindle-on`, `.spindle-reverse`, `.spindle-speed-cmd` | in | `spindle.0.on`, `.reverse`, `.speed-out-abs` |
| `pilink.spindle-at-speed`, `.spindle-waiting`, `.spindle-refused`, `.spindle-speed` | out | at speed (within 2%), start waiting for the ESC, start refused, the board's ramped rpm |
| `pilink.feed-inhibit`, `.pause` | bit out | door hold: `motion.feed-inhibit`, `halui.program.pause` |
| `pilink.program-paused` | bit in | `halui.program.is-paused` |
| `pilink.door-hold` | s32 out | 0 none, 1 spindle stopped (waiting for Resume), 2 restarting |
| `pilink.owner`, `.release-reason`, `.result`, `.board-state`, `.estp`, `.safe`, `.state` | s32 out | |
| `pilink.feedback-age`, `.rx-frames`, `.rx-errors`, `.tx-errors`, `.stale-segments`, `.clamped`, `.late`, `.rx-gap-max` | out | diagnostics (`rx-gap-max` in µs, reset at ENABLE) |
| `pilink.board-time`, `.reset-board-dt`, `.reset-pi-dt` | out | board clock, and what tripped `position-lost` |
| `pilink.p-gain` (30), `.latency` (2.0), `.deadband` (0.5) | param | position loop |
| `pilink.segment-us` (1000), `.watchdog-ms` (5), `.hold-ms` (250) | param | ENABLE |

Module parameters: `port` (`/dev/ttyAMA0`), `baud` (2000000; also 115200, 1000000, 3000000),
`hal_name` (see Install).

## Bench results (2026-10-21)

All with X/Y/Z disabled and clamped to 2 steps/s by the driver and by the board; their counts
didn't change in any run.

- **Idle:** 100 Hz polls, 0 receive errors. `pilink.read` max 2-6 µs, `pilink.write` max
  42 µs (the tty `write()`), servo thread max 44 µs of 1000.
- **Motor 4, 22 s of trapezoids** (±4000 steps, 15,000 steps/s, 100,000 steps/s^2, through
  `limit3`): 22,536 segments, 0 stale, 0 underruns, 0 receive or transmit errors, longest
  FEEDBACK gap 2.0 ms, 1 late period. Tracking error 0.67 steps rms, 2.6 max; final position
  exact.
- **USB `!` mid-move:** fault, release reason 7, back to idle once USB owned the board;
  machine off and on gave a clean ENABLE.
- **Command/feedback mismatch at enable:** refused, nothing sent.
- **Board clock wrap** (71.6 min): crossed with no false `position-lost`.
- **The config, headless** (a test display instead of AXIS): loads; F1 takes LinuxCNC to
  E-stop reset with the board OK; never switched on.

## Still to test (with someone at the machine)

- Door hold in MDI (`M3`, then open the door), where halui's pause may not apply.
- Probing on in5.
- Following error and `p-gain` at higher speeds.
- One false `position-lost` was seen in an early build, before the 500 ms gap rule. It didn't
  happen again in later runs.

## Decisions for the user

1. **Hard limits:** the home switches are not wired as limit switches, as in stock firmware
   (`lim` 0); soft limits apply after homing. Wiring them would need `HOME_OFFSET` larger than
   the switch's release travel, otherwise LinuxCNC reports "on limit" right after homing.
   That travel hasn't been measured.
2. **Install:** `sudo halcompile --install` (one file into `/usr/lib/linuxcnc/modules`) or
   the path workaround.
3. **Coordinates:** g2core's convention (0 = 0.4 mm off each switch), so G-code written for
   the stock machine lines up.

## Protocol notes

- FEEDBACK doesn't say when the current segment started. Board segments (61 DDA ticks,
  999.2 µs) and the Pi's 1000 µs frames drift past each other every ~1.3 s, so the moment a
  command is picked up slides through one period. That shows as a 1-2 step sawtooth in
  `error` at 15,000 steps/s. A u16 "µs since the current segment started" in FEEDBACK's
  reserved bytes (offset 34) would let the driver model the pickup exactly. Not needed at
  these speeds.
