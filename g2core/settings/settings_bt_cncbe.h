/*
 * settings_bt_cncbe.h - Bantam Tools CNC (bt_cncbe) on a TinyG v9k board
 * This file is part of the g2core project
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * Reconstructed - Bantam never published this file (boards.mk references it).
 *
 * Values marked "$$" were read from running machines (fb 101.16,
 * fbs 101.16-1-g13daf, fbc settings_bt_cncbe.h, hp g2v9, hv k) in October 2026;
 * two machines gave identical values. A $$ dump shows the
 * live values, which may include anything the Bantam desktop software set after
 * boot, so they are not guaranteed to be Bantam's compiled defaults.
 *
 * Values marked "BIN" were read from the compiled-in defaults table (cfgArray) of
 * bt_cncbe.bin from the Bantam Tools 2.5.34 desktop app, which carries the same
 * build string (101.16-1-g13daf). Every $$ value above also matches that table,
 * except values the firmware or the desktop app set at runtime.
 *
 * Compile-time constants were checked by matching functions between our GCC 7 build
 * and bt_cncbe.bin (BIN). Values still marked "OM" are copied from settings_othermill.h
 * and are not referenced by any code, so they have no effect on this build.
 */

#ifndef SETTINGS_BT_CNCBE_H_ONCE
#define SETTINGS_BT_CNCBE_H_ONCE

/***********************************************************************/
/**** Bantam Tools CNC (bt_cncbe) profile ******************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Bantam Tools CNC (bt_cncbe) settings"

//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_INTEGRATION_TIME   0.1     // $$ jt
#define CHORDAL_TOLERANCE           0.005   // $$ ct (mm)

#define SOFT_LIMIT_ENABLE           0       // $$ sl
#define HARD_LIMIT_ENABLE           0       // $$ lim
#define SAFETY_INTERLOCK_ENABLE     1       // $$ saf

// NOTE: these are cast to ioPolarity, so 1 = IO_ACTIVE_LOW on the output pin.
// That is what produces do1po=1 and do4po=1 in $$.
#define SPINDLE_ENABLE_POLARITY     1       // BIN spep
#define SPINDLE_DIR_POLARITY        0       // BIN spdp
#define SPINDLE_PAUSE_ON_HOLD       true    // BIN spph
#define SPINDLE_SPINUP_DELAY        3.0     // BIN spde (Othermill uses 1.5)
#define SPINDLE_SPEED_CHANGE_PER_MS 7.0     // BIN (ESCSpindle constructor literal)

#define ESC_BOOT_TIME               5000    // BIN - how long the ESC takes to boot, in milliseconds
#define ESC_LOCKOUT_TIME            900     // BIN - interlock engaged time before killing power

#define COOLANT_MIST_POLARITY       1       // BIN comp
#define COOLANT_FLOOD_POLARITY      1       // BIN cofp
#define COOLANT_PAUSE_ON_HOLD       true    // BIN coph

#define FEEDHOLD_Z_LIFT             0       // $$ zl (Othermill uses -1 = go to Z max)
#define PROBE_REPORT_ENABLE         true    // BIN prbr

#define HAS_CANSTACK_Z_AXIS         false   // OM

// Switch definitions for interlock & E-stop
#define ENABLE_INTERLOCK_AND_ESTOP          // OM
#define INTERLOCK_SWITCH_INPUT          4   // BIN - DI4 (Ymax position)
#define ESTOP_SWITCH_INPUT              2   // BIN - DI2 (Xmax position)
#undef PAUSE_DWELL_TIME
#define PAUSE_DWELL_TIME                2.5 // OM

// Communications and reporting settings

#define USB_SERIAL_PORTS_EXPOSED    1                   // lsusb - valid options are 1 or 2, only!

#define COMM_MODE                   AUTO_MODE           // $$ ej=2
#define XIO_ENABLE_FLOW_CONTROL     FLOW_CONTROL_RTS    // OM

#define TEXT_VERBOSITY              TV_VERBOSE          // $$ tv=1
#define JSON_VERBOSITY              JV_MESSAGES         // $$ jv=2
#define QUEUE_REPORT_VERBOSITY      QR_OFF              // $$ qv=0

#define STATUS_REPORT_VERBOSITY     SR_FILTERED         // $$ sv=1
#define STATUS_REPORT_MIN_MS        100                 // BIN (sr_set_si identical)
#define STATUS_REPORT_INTERVAL_MS   250                 // $$ si
#define STATUS_REPORT_DEFAULTS      "mpox", "mpoy", "mpoz", \
                                    "ofsx", "ofsy", "ofsz", \
                                    "g55x", "g55y", "g55z", \
                                    "unit", "stat", "coor", "momo", "dist", \
                                    "home", "mots", "plan", "line", "path", \
                                    "frmo", "prbe", "safe", "estp", "spc", \
                                    "hold", "macs", "cycs", "sps"   // BIN

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS MILLIMETERS     // $$ gun=1
#define GCODE_DEFAULT_PLANE CANON_PLANE_XY  // $$ gpl=0
#define GCODE_DEFAULT_COORD_SYSTEM G55      // $$ gco=2
#define GCODE_DEFAULT_PATH_CONTROL PATH_CONTINUOUS          // $$ gpa=2
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE  // $$ gdi=0

// *** motor settings ************************************************************************************

#define MOTOR_POWER_LEVEL           0.3                 // $$ 1pl..4pl
#define MOTOR_POWER_LEVEL_IDLE      0.15                // BIN 1pi..4pi
#define MOTOR_POWER_LEVEL_DISABLED  0.05                // OM

#define MOTOR_POWER_MODE            MOTOR_POWER_REDUCED_WHEN_IDLE   // $$ 1pm..4pm=4
#define MOTOR_POWER_TIMEOUT         2.00                // $$ mt (seconds)

// Integer travel values on purpose: the disabled B/C axes in $$ (bvm 227520) match
// the defaults' (X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360 only with integer division.
#define M1_MOTOR_MAP                AXIS_X_EXTERNAL     // $$ 1ma
#define M1_STEP_ANGLE               1.8                 // $$ 1sa
#define M1_TRAVEL_PER_REV           8                   // $$ 1tr (mm)
#define M1_MICROSTEPS               8                   // $$ 1mi
#define M1_POLARITY                 1                   // $$ 1po
#define M1_POWER_MODE               MOTOR_POWER_MODE    // $$ 1pm
#define M1_POWER_LEVEL              MOTOR_POWER_LEVEL   // $$ 1pl
#define M1_POWER_LEVEL_IDLE         MOTOR_POWER_LEVEL_IDLE
#define M1_PWM_FREQUENCY            29000               // BIN - Vref PWM (Hz), staggered per motor

#define M2_MOTOR_MAP                AXIS_Y_EXTERNAL     // $$ 2ma
#define M2_STEP_ANGLE               1.8                 // $$ 2sa
#define M2_TRAVEL_PER_REV           8                   // $$ 2tr (mm)
#define M2_MICROSTEPS               8                   // $$ 2mi
#define M2_POLARITY                 1                   // $$ 2po
#define M2_POWER_MODE               MOTOR_POWER_MODE    // $$ 2pm
#define M2_POWER_LEVEL              MOTOR_POWER_LEVEL   // $$ 2pl
#define M2_POWER_LEVEL_IDLE         MOTOR_POWER_LEVEL_IDLE
#define M2_PWM_FREQUENCY            31000               // BIN

#define M3_MOTOR_MAP                AXIS_Z_EXTERNAL     // $$ 3ma
#define M3_STEP_ANGLE               1.8                 // $$ 3sa
#define M3_TRAVEL_PER_REV           4                   // $$ 3tr (mm)
#define M3_MICROSTEPS               8                   // $$ 3mi
#define M3_POLARITY                 0                   // $$ 3po
#define M3_POWER_MODE               MOTOR_POWER_MODE    // $$ 3pm
#define M3_POWER_LEVEL              MOTOR_POWER_LEVEL   // $$ 3pl
#define M3_POWER_LEVEL_IDLE         MOTOR_POWER_LEVEL_IDLE
#define M3_PWM_FREQUENCY            37000               // BIN

#define M4_MOTOR_MAP                AXIS_A_EXTERNAL     // $$ 4ma
#define M4_STEP_ANGLE               1.8                 // $$ 4sa
#define M4_TRAVEL_PER_REV           90                  // $$ 4tr (degrees per motor rev)
#define M4_MICROSTEPS               8                   // $$ 4mi
#define M4_POLARITY                 1                   // $$ 4po
#define M4_POWER_MODE               MOTOR_POWER_MODE    // $$ 4pm (enabled - Othermill disables M4)
#define M4_POWER_LEVEL              MOTOR_POWER_LEVEL   // $$ 4pl
#define M4_POWER_LEVEL_IDLE         MOTOR_POWER_LEVEL_IDLE
#define M4_PWM_FREQUENCY            23000               // BIN

#define M5_MOTOR_MAP                AXIS_B_EXTERNAL     // OM - not present on v9
#define M5_STEP_ANGLE               1.8
#define M5_TRAVEL_PER_REV           360
#define M5_MICROSTEPS               8
#define M5_POLARITY                 0
#define M5_POWER_MODE               MOTOR_DISABLED
#define M5_POWER_LEVEL              MOTOR_POWER_LEVEL_DISABLED
#define M5_POWER_LEVEL_IDLE         MOTOR_POWER_LEVEL_DISABLED

#define M6_MOTOR_MAP                AXIS_C_EXTERNAL     // OM - not present on v9
#define M6_STEP_ANGLE               1.8
#define M6_TRAVEL_PER_REV           360
#define M6_MICROSTEPS               8
#define M6_POLARITY                 0
#define M6_POWER_MODE               MOTOR_DISABLED
#define M6_POWER_LEVEL              MOTOR_POWER_LEVEL_DISABLED
#define M6_POWER_LEVEL_IDLE         MOTOR_POWER_LEVEL_DISABLED

// *** axis settings **********************************************************************************

#define X_AXIS_MODE                 AXIS_STANDARD       // $$ xam
#define X_VELOCITY_MAX              5060                // $$ xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              5060                // $$ xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                0                   // $$ xtn
#define X_TRAVEL_MAX                185                 // $$ xtm
#define X_JERK_MAX                  500                 // $$ xjm
#define X_JERK_HIGH_SPEED           1000                // $$ xjh
#define X_HOMING_INPUT              1                   // $$ xhi
#define X_HOMING_DIRECTION          0                   // $$ xhd
#define X_SEARCH_VELOCITY           300                 // $$ xsv
#define X_LATCH_VELOCITY            25                  // $$ xlv
#define X_LATCH_BACKOFF             2                   // $$ xlb
#define X_ZERO_BACKOFF              0.4                 // $$ xzb

#define Y_AXIS_MODE                 AXIS_STANDARD       // $$ yam
#define Y_VELOCITY_MAX              5060                // $$ yvm
#define Y_FEEDRATE_MAX              5060                // $$ yfr
#define Y_TRAVEL_MIN                0                   // $$ ytn
#define Y_TRAVEL_MAX                237                 // $$ ytm
#define Y_JERK_MAX                  500                 // $$ yjm
#define Y_JERK_HIGH_SPEED           1000                // $$ yjh
#define Y_HOMING_INPUT              3                   // $$ yhi
#define Y_HOMING_DIRECTION          0                   // $$ yhd
#define Y_SEARCH_VELOCITY           300                 // $$ ysv
#define Y_LATCH_VELOCITY            25                  // $$ ylv
#define Y_LATCH_BACKOFF             2                   // $$ ylb
#define Y_ZERO_BACKOFF              0.4                 // $$ yzb

#define Z_AXIS_MODE                 AXIS_STANDARD       // $$ zam
#define Z_VELOCITY_MAX              2000                // $$ zvm
#define Z_FEEDRATE_MAX              2000                // $$ zfr
#define Z_TRAVEL_MIN                -110                // $$ ztn
#define Z_TRAVEL_MAX                0                   // $$ ztm
#define Z_JERK_MAX                  300                 // $$ zjm
#define Z_JERK_HIGH_SPEED           1000                // $$ zjh
#define Z_HOMING_INPUT              6                   // $$ zhi
#define Z_HOMING_DIRECTION          1                   // $$ zhd
#define Z_SEARCH_VELOCITY           300                 // $$ zsv
#define Z_LATCH_VELOCITY            25                  // $$ zlv
#define Z_LATCH_BACKOFF             2                   // $$ zlb
#define Z_ZERO_BACKOFF              0.4                 // $$ zzb

#define A_AXIS_MODE                 AXIS_STANDARD       // $$ aam
#define A_RADIUS                    1.0                 // $$ ara
#define A_VELOCITY_MAX              2000                // $$ avm (deg/min)
#define A_FEEDRATE_MAX              2000                // $$ afr
#define A_TRAVEL_MIN                -1                  // $$ atn (min == max means no limit)
#define A_TRAVEL_MAX                -1                  // $$ atm
#define A_JERK_MAX                  650                 // $$ ajm
#define A_JERK_HIGH_SPEED           650                 // $$ ajh
#define A_HOMING_INPUT              0                   // $$ ahi
#define A_HOMING_DIRECTION          0                   // $$ ahd
#define A_SEARCH_VELOCITY           300                 // $$ asv
#define A_LATCH_VELOCITY            25                  // $$ alv
#define A_LATCH_BACKOFF             2                   // $$ alb
#define A_ZERO_BACKOFF              0.4                 // $$ azb

// B and C are disabled ($$ bam=0, cam=0) and left to settings_default.h

//*** Input / output settings ***
// All DI values below match both $$ and settings_othermill.h.

#define PROBING_INPUT               5                   // BIN prbin - Zmin position

// Xmin on v9 board                 // X homing
#define DI1_ENABLED                 IO_ENABLED          // $$ di1mo
#define DI1_POLARITY                IO_ACTIVE_HIGH      // $$ di1po=0 (normally closed)
#define DI1_ACTION                  INPUT_ACTION_LIMIT  // $$ di1ac=9

// Xmax                             // External ESTOP
#define DI2_ENABLED                 IO_ENABLED
#define DI2_POLARITY                IO_ACTIVE_HIGH      // $$ di2po=0
#define DI2_ACTION                  INPUT_ACTION_NONE   // $$ di2ac=0 - SHUTDOWN handled by E-Stop handler

// Ymin                             // Y homing
#define DI3_ENABLED                 IO_ENABLED
#define DI3_POLARITY                IO_ACTIVE_HIGH      // $$ di3po=0
#define DI3_ACTION                  INPUT_ACTION_LIMIT  // $$ di3ac=9

// Ymax                             // Safety interlock
#define DI4_ENABLED                 IO_ENABLED
#define DI4_POLARITY                IO_ACTIVE_HIGH      // $$ di4po=0
#define DI4_ACTION                  INPUT_ACTION_NONE   // $$ di4ac=0 - hold is performed by Interlock function

// Zmin                             // Z probe
#define DI5_ENABLED                 IO_ENABLED
#define DI5_POLARITY                IO_ACTIVE_LOW       // $$ di5po=1 (normally open)
#define DI5_ACTION                  INPUT_ACTION_NONE   // $$ di5ac=0

// Zmax                             // Z homing
#define DI6_ENABLED                 IO_ENABLED
#define DI6_POLARITY                IO_ACTIVE_HIGH      // $$ di6po=0
#define DI6_ACTION                  INPUT_ACTION_LIMIT  // $$ di6ac=9

// Amin                             // Version pin - board type
#define DI7_ENABLED                 IO_ENABLED
#define DI7_POLARITY                IO_ACTIVE_LOW       // $$ di7po=1
#define DI7_ACTION                  INPUT_ACTION_NONE

// Amax                             // Version pin - board type
#define DI8_ENABLED                 IO_ENABLED
#define DI8_POLARITY                IO_ACTIVE_LOW       // $$ di8po=1
#define DI8_ACTION                  INPUT_ACTION_NONE

// Safety line w/HW timer           // Version pin - board type
#define DI9_ENABLED                 IO_DISABLED         // $$ di9mo=0
#define DI9_POLARITY                IO_ACTIVE_LOW
#define DI9_ACTION                  INPUT_ACTION_NONE
#define DI9_EXTERNAL_NUMBER         10                  // $$ di9in

// Silk marked as "CS2" on J17      // Version pin - machine type (J17 jumper to GND)
#define DI10_ENABLED                IO_ENABLED          // $$ di10mo=1
#define DI10_POLARITY               IO_ACTIVE_LOW       // $$ di10po=1
#define DI10_ACTION                 INPUT_ACTION_NONE
#define DI10_EXTERNAL_NUMBER        9                   // $$ di10in

// DO1-DO4 (spindle enable/dir/PWM, coolant) use settings_default.h; their live
// polarities come from SPINDLE_ENABLE_POLARITY and COOLANT_*_POLARITY above.

// *** PWM SPINDLE CONTROL ***
// 100 Hz with 13.8-19.3% duty = 1.38-1.93 ms pulses: a hobby-ESC (RC servo) signal

#define P1_PWM_FREQUENCY            100     // $$ p1frq (Hz)
#define P1_CW_SPEED_LO              10000   // $$ p1csl (RPM)
#define P1_CW_SPEED_HI              28000   // $$ p1csh
#define P1_CW_PHASE_LO              0.138   // $$ p1cpl
#define P1_CW_PHASE_HI              0.193   // $$ p1cph
#define P1_CCW_SPEED_LO             0       // $$ p1wsl
#define P1_CCW_SPEED_HI             0       // $$ p1wsh
#define P1_CCW_PHASE_LO             0.1     // $$ p1wpl
#define P1_CCW_PHASE_HI             0.1     // $$ p1wph
#define P1_PWM_PHASE_OFF            0.1     // $$ p1pof

#endif  //  SETTINGS_BT_CNCBE_H_ONCE
