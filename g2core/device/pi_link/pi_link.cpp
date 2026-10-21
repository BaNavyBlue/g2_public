/*
 * pi_link.cpp - binary link to a LinuxCNC host over the v9 board's J17 header (USART2)
 * This file is part of the g2core project
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * v0: echo (PING/PONG) and STATUS - see PROTOCOL.md.
 * v1: motion (ENABLE, MOTION, DISABLE, SAFETY, DIAG) - see PROTOCOL-v1-draft.md. While the Pi
 * has control, g2core's exec interrupt calls pi_link_exec_move() instead of the planner, and
 * each segment plays the newest velocity command. Without PI_LINK_MOTION every velocity is
 * forced to zero.
 *
 * USART2 is driven directly rather than through Motate::UART: Motate's DMA layer only
 * supports USART0/1 on the SAM3X, and its UART header #undefs the CMSIS USART2 macros.
 * Both directions use the USART's PDC (DMA) and are polled from pi_link_periodic(); no
 * interrupts are enabled, so nothing here can preempt the step generator.
 *
 * Receive: the PDC fills a ring of two halves (current + next buffer). Each pass reads
 * whatever has arrived, splits frames on 0x00, answers valid frames, and re-arms the half
 * that has been fully read. Transmit: one reply at a time; a reply is skipped if the
 * previous one is still being sent, which cannot happen while the host waits for replies.
 */

#include "g2core.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "kinematics.h"
#include "spindle.h"
#include "gpio.h"
#include "report.h"
#include "pi_link.h"

#include "MotatePins.h"
#include "MotateTimers.h"

#include <math.h>
#include <string.h>

#ifdef BANTAM
#include "bantam_safety_manager.h"
extern BantamSafetyManager *bsm;
#endif

namespace {

// SAM3X8C USART2 and its PDC (addresses and peripheral ID from sam3x8c.h)
Usart *const     usart2         = reinterpret_cast<Usart *>(0x400A0000U);
Pdc *const       usart2_pdc     = reinterpret_cast<Pdc *>(0x400A0100U);
constexpr uint32_t USART2_PERIPH_ID = 19;

// Setting the pins to peripheral A (USART2) is done by Motate's pin classes
using RxPin = Motate::UARTRxPin<Motate::kSocket2_SPISlaveSelectPinNumber>;  // J17 "CS2" (PB21, RXD2)
using TxPin = Motate::UARTTxPin<Motate::kSocket1_SPISlaveSelectPinNumber>;  // J17 "CS1" (PB20, TXD2)
static_assert((RxPin::uartNum == 2) && (TxPin::uartNum == 2), "J17 CS2/CS1 must be USART2 RXD/TXD");
RxPin rx_pin;
TxPin tx_pin;

constexpr uint8_t  PROTOCOL_VERSION = 0;
constexpr uint16_t MAX_FRAME        = 128;                          // decoded bytes
constexpr uint16_t MAX_ENCODED      = MAX_FRAME + (MAX_FRAME / 254) + 2;
constexpr uint16_t MAX_PING         = 64;
constexpr uint16_t RX_HALF          = 256;
constexpr uint16_t RX_SIZE          = 2 * RX_HALF;

enum : uint8_t {
    TYPE_PING       = 0x01,
    TYPE_STATUS_REQ = 0x02,
    TYPE_DIAG_REQ   = 0x03,
    TYPE_ENABLE     = 0x10,
    TYPE_MOTION     = 0x11,
    TYPE_DISABLE    = 0x12,
    TYPE_SAFETY     = 0x13,
    TYPE_PONG       = 0x81,
    TYPE_STATUS     = 0x82,
    TYPE_DIAG       = 0x83,
    TYPE_FEEDBACK   = 0x91,
    TYPE_NAK        = 0xFF,
};

enum : uint8_t {
    NAK_UNKNOWN_TYPE = 1,
    NAK_BAD_LENGTH   = 2,
};

volatile uint8_t rx_ring[RX_SIZE];     // written by the PDC
uint16_t         rx_read       = 0;     // next ring index to read
bool             rx_stall_seen = false;

uint8_t  rx_encoded[MAX_ENCODED];
uint16_t rx_len      = 0;
bool     rx_overlong = false;

uint8_t  tx_buf[MAX_ENCODED + 1];

struct {
    uint16_t ok;
    uint16_t crc;
    uint16_t framing;
    uint16_t overflow;
} counters = {};

void put16(uint8_t *p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = v >> 8;
}

void put32(uint8_t *p, uint32_t v) {
    for (uint8_t i = 0; i < 4; i++) {
        p[i] = (v >> (8 * i)) & 0xFF;
    }
}

void putf(uint8_t *p, float v) {
    uint32_t u;
    memcpy(&u, &v, 4);
    put32(p, u);
}

uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// CRC-32 (IEEE 802.3, reflected), one nibble at a time: same result as zlib.crc32
uint32_t crc32(const uint8_t *data, uint16_t len) {
    static const uint32_t table[16] = {
        0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
        0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C, 0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C,
    };
    uint32_t crc = 0xFFFFFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 4) ^ table[(crc ^ data[i]) & 0x0F];
        crc = (crc >> 4) ^ table[(crc ^ (data[i] >> 4)) & 0x0F];
    }
    return ~crc;
}

// Returns the decoded length, or -1 if the input is not valid COBS or does not fit.
int16_t cobs_decode(const uint8_t *in, uint16_t len, uint8_t *out, uint16_t out_max) {
    uint16_t r = 0;
    uint16_t w = 0;
    while (r < len) {
        const uint8_t code = in[r++];
        if (code == 0) {
            return -1;
        }
        for (uint8_t i = 1; i < code; i++) {
            if ((r >= len) || (w >= out_max)) {
                return -1;
            }
            out[w++] = in[r++];
        }
        if ((code < 0xFF) && (r < len)) {
            if (w >= out_max) {
                return -1;
            }
            out[w++] = 0;
        }
    }
    return w;
}

uint16_t cobs_encode(const uint8_t *in, uint16_t len, uint8_t *out) {
    uint16_t code_pos = 0;
    uint16_t w        = 1;
    uint8_t  code     = 1;
    for (uint16_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_pos] = code;
            code_pos      = w++;
            code          = 1;
        } else {
            out[w++] = in[i];
            code++;
            if (code == 0xFF) {
                out[code_pos] = code;
                code_pos      = w++;
                code          = 1;
            }
        }
    }
    out[code_pos] = code;
    return w;
}

// ---- USART2 + PDC ----

void rx_reset() {
    usart2_pdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
    usart2_pdc->PERIPH_RPR  = (uint32_t)(uintptr_t)rx_ring;
    usart2_pdc->PERIPH_RCR  = RX_HALF;
    usart2_pdc->PERIPH_RNPR = (uint32_t)(uintptr_t)(rx_ring + RX_HALF);
    usart2_pdc->PERIPH_RNCR = RX_HALF;
    usart2_pdc->PERIPH_PTCR = PERIPH_PTCR_RXTEN;
    rx_read       = 0;
    rx_stall_seen = false;
    rx_len        = 0;
    rx_overlong   = false;
}

void hw_init(uint32_t baud) {
    Motate::SamCommon::enablePeripheralClock(USART2_PERIPH_ID);
    rx_pin.setOptions(Motate::kPullUp);  // keep RX idle-high with no host attached

    usart2->US_IDR = 0xFFFFFFFF;  // no interrupts
    usart2->US_CR  = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
    usart2->US_MR  = US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK | US_MR_CHRL_8_BIT | US_MR_PAR_NO |
                     US_MR_NBSTOP_1_BIT | US_MR_CHMODE_NORMAL;
    // MCK / (16 * baud) in 1/8 steps, using the fractional divider so 1, 2 and 3 Mbaud are exact
    const uint32_t eighths = (Motate::SamCommon::getPeripheralClockFreq() + baud) / (2 * baud);
    usart2->US_BRGR        = US_BRGR_CD(eighths / 8) | US_BRGR_FP(eighths % 8);

    usart2_pdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS;
    usart2_pdc->PERIPH_TCR  = 0;
    usart2_pdc->PERIPH_TNCR = 0;
    rx_reset();
    usart2_pdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;

    usart2->US_CR = US_CR_RXEN | US_CR_TXEN;
}

// Ring index the PDC will write next. While the PDC is running (RCR != 0) this is inside the
// half being written; once it has stalled it is the end of that half, which is ambiguous, so
// callers check RCR after reading this.
uint16_t rx_write_index() {
    return (uint16_t)(usart2_pdc->PERIPH_RPR - (uint32_t)(uintptr_t)rx_ring);
}

// Queue the half the writer is not in, once the reader has finished it (the reader has caught
// up into the writer's half).
void rx_rearm() {
    if (usart2_pdc->PERIPH_RNCR != 0) {
        return;  // the next buffer is still queued
    }
    const uint16_t w = rx_write_index();
    if (usart2_pdc->PERIPH_RCR == 0) {
        return;  // stalled (or swapping buffers right now); pi_link_periodic() deals with it
    }
    // If the writer finishes its half between here and the RNCR write, the half queued is still
    // the right one to follow it.
    const uint16_t writer_half = w / RX_HALF;
    if ((rx_read / RX_HALF) == writer_half) {
        usart2_pdc->PERIPH_RNPR = (uint32_t)(uintptr_t)(rx_ring + (1 - writer_half) * RX_HALF);
        usart2_pdc->PERIPH_RNCR = RX_HALF;
    }
}

bool tx_idle() {
    return (usart2_pdc->PERIPH_TCR == 0) && (usart2_pdc->PERIPH_TNCR == 0);
}

void tx_start(uint16_t len) {
    __asm__ volatile("dmb" ::: "memory");  // tx_buf is in memory before the PDC reads it
    usart2_pdc->PERIPH_TPR = (uint32_t)(uintptr_t)tx_buf;
    usart2_pdc->PERIPH_TCR = len;
}

// ---- protocol ----

void send_frame(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t payload_len) {
    if (!tx_idle() || (payload_len > MAX_FRAME - 8)) {
        return;
    }
    uint8_t frame[MAX_FRAME];
    frame[0] = PROTOCOL_VERSION;
    frame[1] = type;
    put16(frame + 2, seq);
    if (payload_len > 0) {
        memcpy(frame + 4, payload, payload_len);
    }
    put32(frame + 4 + payload_len, crc32(frame, 4 + payload_len));

    uint16_t len  = cobs_encode(frame, 8 + payload_len, tx_buf);
    tx_buf[len++] = 0;
    tx_start(len);
}

void send_nak(uint16_t seq, uint8_t reason, uint8_t type) {
    const uint8_t payload[2] = {reason, type};
    send_frame(TYPE_NAK, seq, payload, 2);
}

void send_status(uint16_t seq) {
    uint8_t p[40] = {};

    p[0] = (uint8_t)cm_get_combined_state(&cm1);
#ifdef BANTAM
    if (bsm) {
        p[1] = (bsm->get_interlock_safety_state() ? 0x1 : 0) | (bsm->get_esc_safety_state() ? 0x2 : 0);
        p[2] = bsm->get_estop_state() & 0x3;
    }
#endif
    const uint8_t axes[4] = {AXIS_X, AXIS_Y, AXIS_Z, AXIS_A};
    for (uint8_t i = 0; i < 4; i++) {
        if (cm->homed[axes[i]]) {
            p[3] |= (1 << i);
        }
    }
    uint16_t inputs = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (in_r[i]->getState()) {
            inputs |= (1 << i);
        }
    }
    put16(p + 4, inputs);
    p[6] = (uint8_t)spindle_get_direction();
    putf(p + 8, spindle_get_speed());
    for (uint8_t i = 0; i < 4; i++) {
        putf(p + 12 + 4 * i, cm_get_absolute_position(ACTIVE_MODEL, axes[i]));
    }
    put32(p + 28, Motate::SysTickTimer.getValue());
    put16(p + 32, counters.ok);
    put16(p + 34, counters.crc);
    put16(p + 36, counters.framing);
    put16(p + 38, counters.overflow);

    send_frame(TYPE_STATUS, seq, p, sizeof(p));
}

// ---- motion (v1) ----

constexpr uint8_t  PI_MOTORS         = 4;
static_assert(MOTORS >= PI_MOTORS, "the link drives motors 1-4");
constexpr float    DDA_MAX_STEP_RATE = 0.95f * FREQUENCY_DDA;  // the DDA makes at most one step per tick
constexpr uint32_t STALE_US          = 2000;    // a segment is stale if the newest command is older than this
constexpr uint32_t EXEC_LOST_US      = 20000;   // no segment prepared for this long while in control: chain lost
constexpr uint32_t SYNC_PERIOD_US    = 10000;   // how often USB-visible positions follow the step counters

enum : uint8_t {
    OWNER_USB      = 0,
    OWNER_PI       = 1,
    OWNER_STOPPING = 2,
};

enum : uint8_t {    // FEEDBACK result
    RESULT_OK             = 0,
    RESULT_NOT_IDLE       = 1,
    RESULT_ESTOP          = 2,
    RESULT_ALARM          = 3,
    RESULT_BAD_PARAM      = 4,
    RESULT_NOT_IN_CONTROL = 5,
    RESULT_IN_CONTROL     = 6,
};

enum : uint8_t {    // why the board ended the Pi's control
    REASON_NONE      = 0,
    REASON_DISABLE   = 1,
    REASON_WATCHDOG  = 2,
    REASON_ESTOP     = 3,
    REASON_DOOR      = 4,
    REASON_ALARM     = 5,
    REASON_BAD_FRAME = 6,
    REASON_USB_STOP  = 7,
    REASON_FAULT     = 8,
};

enum : uint8_t {    // FEEDBACK flags
    FB_HOLD_REQUEST  = 0x01,
    FB_CLAMPED       = 0x02,
    FB_STALE         = 0x04,
    FB_MOTION_LOCKED = 0x08,
};

// The Desktop CNC maps motors 1-4 to these axes
const uint8_t motor_axis[PI_MOTORS] = {AXIS_X, AXIS_Y, AXIS_Z, AXIS_A};

struct MotionCommand {
    float    vel[PI_MOTORS];    // steps/s, already checked and clamped
    uint16_t seq;
    uint32_t rx_us;             // board time it was taken in
};

// The main loop writes the copy the exec interrupt is not reading, then flips cmd_live. The
// interrupt can't be preempted by the main loop, so it always reads a whole command.
MotionCommand    cmd[2];
volatile uint8_t cmd_live = 0;

volatile uint8_t owner          = OWNER_USB;
volatile uint8_t release_reason = REASON_NONE;
volatile uint8_t stop_request   = REASON_NONE;  // main loop -> exec interrupt
volatile bool    stop_done      = false;        // exec interrupt -> main loop: final zero segment queued
bool             fb_clamped     = false;        // latched until the next FEEDBACK (main loop only)
volatile bool    fb_stale       = false;        // set by the exec interrupt, cleared by the next FEEDBACK

struct {
    uint32_t watchdog_us;
    uint16_t hold_timeout_ms;   // used once spindle control is in (door hold)
    uint16_t seg_ticks;
    float    seg_s;             // segment length in whole DDA ticks, seconds
    float    max_vel[PI_MOTORS];
    float    stop_decel[PI_MOTORS];
} link_cfg;

// exec interrupt state
float             v_prev[PI_MOTORS];    // end velocity of the last segment prepared, steps/s
float             stop_v0[PI_MOTORS];
float             stop_t;
float             stop_len;
volatile uint16_t queued_seq = 0;       // seq of the newest MOTION taken into a segment

struct {
    volatile uint16_t exec_max_us;
    volatile uint16_t exec_last_us;
    volatile uint32_t segments;
    volatile uint16_t stale_segments;
    volatile uint16_t watchdog_trips;
    uint32_t          loop_gap_max_us;
} diag = {};

uint32_t          last_periodic_us = 0;
volatile uint32_t last_exec_us     = 0;
uint32_t          last_sync_us     = 0;

// Signed: the exec interrupt can stamp a time after the main loop read the clock
int32_t us_since(uint32_t then, uint32_t now) {
    return (int32_t)(now - then);
}

// Microseconds since boot, from SysTick (1 ms ticks plus the count within the tick). Wraps after
// about 71 minutes. Safe from interrupts and from the main loop.
uint32_t board_us() {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t ms  = Motate::SysTickTimer.getValue();
    uint32_t val = SysTick->VAL;
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {   // the counter wrapped but the tick interrupt hasn't run yet
        ms++;
        val = SysTick->VAL;
    }
    __set_PRIMASK(primask);
    const uint32_t load = SysTick->LOAD + 1;    // core clocks per ms
    return (ms * 1000) + (((load - 1 - val) * 1000) / load);
}

// Steps each motor has made so far, including the segment now running
void read_positions(int32_t pos[MOTORS]) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t m = 0; m < MOTORS; m++) {
        pos[m] = en.en[m].encoder_steps + en.en[m].steps_run;
    }
    __set_PRIMASK(primask);
}

// Point g2core's positions at where the steps say the machine is. Live (full == false) updates
// only what USB status reports show; full also resets the planner and step bookkeeping, for
// handing control back to USB.
void sync_positions(bool full) {
    int32_t pos[MOTORS];
    read_positions(pos);
    float steps[MOTORS];
    for (uint8_t m = 0; m < MOTORS; m++) {
        steps[m] = (float)pos[m];
    }
    float travel[AXES];
    kn_forward_kinematics(steps, travel);
    for (uint8_t m = 0; m < PI_MOTORS; m++) {
        const uint8_t axis = motor_axis[m];
        if (full) {
            cm_set_position_by_axis(axis, travel[axis]);
        } else {
            cm->gmx.position[axis] = travel[axis];
            mp_set_runtime_position(axis, travel[axis]);
        }
    }
}

// Exec interrupt only. Every motor ramps linearly to zero; all reach it together, at the time the
// slowest one needs at its stop_decel, so the machine stops on a straight line.
void begin_stop(uint8_t reason) {
    owner          = OWNER_STOPPING;
    release_reason = reason;
    stop_request   = REASON_NONE;
    stop_t         = 0;
    stop_len       = 0;
    for (uint8_t m = 0; m < PI_MOTORS; m++) {
        stop_v0[m]     = v_prev[m];
        const float t  = fabsf(v_prev[m]) / link_cfg.stop_decel[m];
        stop_len       = (t > stop_len) ? t : stop_len;
    }
}

void request_stop(uint8_t reason) {
    if ((owner == OWNER_PI) && (stop_request == REASON_NONE)) {
        stop_request = reason;
    }
}

// Main loop only, once the board stop has finished and the steppers are idle
void release_to_usb() {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t m = 0; m < MOTORS; m++) {      // fold in any steps not yet accumulated
        en.en[m].encoder_steps += en.en[m].steps_run;
        en.en[m].steps_run = 0;
    }
    __set_PRIMASK(primask);

    sync_positions(true);
    stop_done = false;
    owner     = OWNER_USB;
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
}

uint8_t safe_bits() {
#ifdef BANTAM
    if (bsm) {
        return (bsm->get_interlock_safety_state() ? 0x1 : 0) | (bsm->get_esc_safety_state() ? 0x2 : 0);
    }
#endif
    return 0;
}

uint8_t estop_bits() {
#ifdef BANTAM
    if (bsm) {
        return bsm->get_estop_state();
    }
#endif
    return 0;
}

bool machine_alarmed() {
    return (cm1.machine_state == MACHINE_ALARM) || (cm1.machine_state == MACHINE_SHUTDOWN) ||
           (cm1.machine_state == MACHINE_PANIC);
}

uint16_t input_bits() {
    uint16_t inputs = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (in_r[i]->getState()) {
            inputs |= (1 << i);
        }
    }
    return inputs;
}

void send_feedback(uint16_t seq, uint8_t result) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool stale = fb_stale;
    fb_stale         = false;
    __set_PRIMASK(primask);

    uint8_t p[40] = {};
    p[0] = result;
    p[1] = owner;
    p[2] = release_reason;
    p[3] = (fb_clamped ? FB_CLAMPED : 0) | (stale ? FB_STALE : 0) | (PI_LINK_MOTION ? 0 : FB_MOTION_LOCKED);
    fb_clamped = false;
    put32(p + 4, board_us());
    int32_t pos[MOTORS];
    read_positions(pos);
    for (uint8_t m = 0; m < PI_MOTORS; m++) {
        put32(p + 8 + 4 * m, (uint32_t)pos[m]);
    }
    put16(p + 24, queued_seq);
    put16(p + 26, diag.stale_segments);
    p[28] = safe_bits();
    p[29] = estop_bits() & 0x3;
    put16(p + 30, input_bits());
    p[32] = (uint8_t)spindle_get_direction();
    p[33] = (uint8_t)cm_get_combined_state(&cm1);
    putf(p + 36, spindle_get_speed());
    send_frame(TYPE_FEEDBACK, seq, p, sizeof(p));
}

void send_diag(uint16_t seq) {
    uint8_t p[20] = {};
    put16(p + 0, diag.exec_max_us);
    put16(p + 2, diag.exec_last_us);
    put32(p + 4, diag.loop_gap_max_us);
    put32(p + 8, diag.segments);
    put16(p + 12, diag.stale_segments);
    put16(p + 14, diag.watchdog_trips);
    put16(p + 16, link_cfg.seg_ticks);
    send_frame(TYPE_DIAG, seq, p, sizeof(p));
}

float getf(const uint8_t *p) {
    const uint32_t u = get32(p);
    float v;
    memcpy(&v, &u, 4);
    return v;
}

uint16_t get16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint8_t enable_check(const uint8_t *p, uint16_t len) {
    if (owner != OWNER_USB) {
        return RESULT_IN_CONTROL;
    }
    if (estop_bits() != 0) {
        return RESULT_ESTOP;
    }
    if (machine_alarmed()) {
        return RESULT_ALARM;
    }
    const cmMachineState ms = cm1.machine_state;
    if (((ms != MACHINE_READY) && (ms != MACHINE_PROGRAM_STOP) && (ms != MACHINE_PROGRAM_END)) ||
        (cm1.cycle_type != CYCLE_NONE) || (cm1.hold_state != FEEDHOLD_OFF) || mp_has_runnable_buffer(mp) ||
        !mp_runtime_is_idle() || st_runtime_isbusy() || (spindle_get_direction() != SPINDLE_OFF)) {
        return RESULT_NOT_IDLE;
    }
    if (len != 52) {
        return RESULT_BAD_PARAM;
    }
    const uint16_t watchdog_ms = get16(p + 0);
    const uint16_t segment_us  = get16(p + 2);
    const uint16_t hold_ms     = get16(p + 4);
    if ((watchdog_ms < 2) || (watchdog_ms > 100) || (segment_us < 500) || (segment_us > 2000) || (hold_ms < 50) ||
        (hold_ms > 2000)) {
        return RESULT_BAD_PARAM;
    }
    for (uint8_t m = 0; m < PI_MOTORS; m++) {
        const float vmax  = getf(p + 8 + 4 * m);
        const float decel = getf(p + 24 + 4 * m);
        if (!isfinite(vmax) || (vmax <= 0) || (vmax > DDA_MAX_STEP_RATE) || !isfinite(decel) || (decel <= 0)) {
            return RESULT_BAD_PARAM;
        }
    }
    return RESULT_OK;
}

void handle_enable(uint16_t seq, const uint8_t *p, uint16_t len) {
    const uint8_t result = enable_check(p, len);
    if (result != RESULT_OK) {
        send_feedback(seq, result);
        return;
    }
    link_cfg.watchdog_us     = get16(p + 0) * 1000UL;
    link_cfg.hold_timeout_ms = get16(p + 4);
    // Whole DDA ticks: 500 us is 31 ticks at 61 kHz (508 us). Rates are per second, so the
    // rounding changes segment length, not speed.
    link_cfg.seg_ticks = (uint16_t)((get16(p + 2) * (uint32_t)FREQUENCY_DDA + 500000UL) / 1000000UL);
    link_cfg.seg_s     = (float)link_cfg.seg_ticks / FREQUENCY_DDA;
    for (uint8_t m = 0; m < PI_MOTORS; m++) {
        link_cfg.max_vel[m]    = getf(p + 8 + 4 * m);
        link_cfg.stop_decel[m] = getf(p + 24 + 4 * m);
        v_prev[m]         = 0;
    }

    const uint32_t now = board_us();
    for (uint8_t i = 0; i < 2; i++) {
        memset(cmd[i].vel, 0, sizeof(cmd[i].vel));
        cmd[i].seq   = seq;
        cmd[i].rx_us = now;     // the watchdog starts now: the first MOTION is due within watchdog_ms
    }
    diag.exec_max_us     = 0;
    diag.exec_last_us    = 0;
    diag.segments        = 0;
    diag.stale_segments  = 0;
    diag.loop_gap_max_us = 0;
    queued_seq           = seq;
    release_reason       = REASON_NONE;
    stop_request         = REASON_NONE;
    stop_done            = false;
    fb_clamped           = false;
    fb_stale             = false;
    last_exec_us         = now;
    last_sync_us         = now;

    for (uint8_t m = 0; m < MOTORS; m++) {
        Motors[m]->enable();    // energized for as long as the Pi has control
    }
    __asm__ volatile("dmb" ::: "memory");
    owner = OWNER_PI;
    st_request_exec_move();     // start the segment chain
    send_feedback(seq, RESULT_OK);
}

void handle_motion(uint16_t seq, const uint8_t *p, uint16_t len) {
    if (owner != OWNER_PI) {
        send_feedback(seq, RESULT_NOT_IN_CONTROL);
        return;
    }
    bool valid = (len == 24) && (p[16] == 0);   // no spindle control over the link yet
    float vel[PI_MOTORS];
    for (uint8_t m = 0; m < PI_MOTORS; m++) {
        vel[m] = getf(p + 4 * m);
        if (!isfinite(vel[m])) {
            valid = false;
        }
    }
    if (!valid) {
        request_stop(REASON_BAD_FRAME);
        send_feedback(seq, RESULT_BAD_PARAM);
        return;
    }

    MotionCommand &c = cmd[cmd_live ^ 1];
    for (uint8_t m = 0; m < PI_MOTORS; m++) {
        float v = PI_LINK_MOTION ? vel[m] : 0;
        if (v > link_cfg.max_vel[m]) {
            v          = link_cfg.max_vel[m];
            fb_clamped = true;
        } else if (v < -link_cfg.max_vel[m]) {
            v          = -link_cfg.max_vel[m];
            fb_clamped = true;
        }
        c.vel[m] = v;
    }
    c.seq   = seq;
    c.rx_us = board_us();
    __asm__ volatile("dmb" ::: "memory");
    cmd_live ^= 1;
    send_feedback(seq, RESULT_OK);
}

void handle_disable(uint16_t seq, uint16_t len) {
    if (len != 0) {
        send_feedback(seq, RESULT_BAD_PARAM);
        return;
    }
    if (owner != OWNER_PI) {
        send_feedback(seq, (owner == OWNER_STOPPING) ? RESULT_OK : RESULT_NOT_IN_CONTROL);
        return;
    }
    request_stop(REASON_DISABLE);
    send_feedback(seq, RESULT_OK);
}

void handle_safety(uint16_t seq, const uint8_t *p, uint16_t len) {
    if (owner != OWNER_USB) {
        send_feedback(seq, RESULT_IN_CONTROL);
        return;
    }
    if (len != 1) {
        send_feedback(seq, RESULT_BAD_PARAM);
        return;
    }
    if (p[0] == 1) {
#ifdef BANTAM
        if (bsm) {
            bsm->ack_estop();
        }
#endif
    } else if (p[0] == 2) {
        cm_clear();
    } else {
        send_feedback(seq, RESULT_BAD_PARAM);
        return;
    }
    send_feedback(seq, RESULT_OK);
}

// Main loop housekeeping while the Pi has control
void motion_periodic() {
    const uint32_t now = board_us();
    const uint32_t gap = now - last_periodic_us;
    last_periodic_us   = now;
    if (owner == OWNER_USB) {
        return;
    }
    if (gap > diag.loop_gap_max_us) {
        diag.loop_gap_max_us = gap;
    }

    if (estop_bits() != 0) {
        request_stop(REASON_ESTOP);
    } else if (machine_alarmed()) {
        request_stop(REASON_ALARM);
    }

    // Something reset the steppers (a g2core halt) and the segment chain stopped: hand back now
    if (us_since(last_exec_us, now) > (int32_t)EXEC_LOST_US) {
        if (owner == OWNER_PI) {
            release_reason = REASON_FAULT;
        }
        stop_done = true;
    }

    if (stop_done && !st_runtime_isbusy()) {
        release_to_usb();
        return;
    }

    if (us_since(last_sync_us, now) > (int32_t)SYNC_PERIOD_US) {
        last_sync_us = now;
        sync_positions(false);
    }
}

void handle_frame(const uint8_t *f, uint16_t len) {
    if ((len < 8) || (f[0] != PROTOCOL_VERSION)) {
        counters.framing++;
        return;
    }
    if (crc32(f, len - 4) != get32(f + len - 4)) {
        counters.crc++;
        return;
    }
    counters.ok++;

    const uint8_t   type        = f[1];
    const uint16_t  seq         = (uint16_t)f[2] | ((uint16_t)f[3] << 8);
    const uint8_t  *payload     = f + 4;
    const uint16_t  payload_len = len - 8;

    switch (type) {
        case TYPE_PING:
            if (payload_len > MAX_PING) {
                send_nak(seq, NAK_BAD_LENGTH, type);
            } else {
                send_frame(TYPE_PONG, seq, payload, payload_len);
            }
            break;
        case TYPE_STATUS_REQ:
            if (payload_len != 0) {
                send_nak(seq, NAK_BAD_LENGTH, type);
            } else {
                send_status(seq);
            }
            break;
        case TYPE_DIAG_REQ:
            if (payload_len != 0) {
                send_nak(seq, NAK_BAD_LENGTH, type);
            } else {
                send_diag(seq);
            }
            break;
        case TYPE_ENABLE:
            handle_enable(seq, payload, payload_len);
            break;
        case TYPE_MOTION:
            handle_motion(seq, payload, payload_len);
            break;
        case TYPE_DISABLE:
            handle_disable(seq, payload_len);
            break;
        case TYPE_SAFETY:
            handle_safety(seq, payload, payload_len);
            break;
        default:
            send_nak(seq, NAK_UNKNOWN_TYPE, type);
            break;
    }
}

void handle_byte(uint8_t c) {
    if (c != 0) {
        if (rx_len < MAX_ENCODED) {
            rx_encoded[rx_len++] = c;
        } else {
            rx_overlong = true;
        }
        return;
    }
    // 0x00 ends a frame
    if (rx_overlong) {
        counters.framing++;
    } else if (rx_len > 0) {
        uint8_t       frame[MAX_FRAME];
        const int16_t len = cobs_decode(rx_encoded, rx_len, frame, sizeof(frame));
        if (len < 0) {
            counters.framing++;
        } else {
            handle_frame(frame, len);
        }
    }
    rx_len      = 0;
    rx_overlong = false;
}

void rx_poll() {
    if (usart2->US_CSR & (US_CSR_OVRE | US_CSR_FRAME)) {
        usart2->US_CR = US_CR_RSTSTA;  // line errors; the CRC catches any damaged frame
    }

    // Both PDC buffers filled before we read them: bytes are being dropped. RCR is also 0 for an
    // instant while the PDC swaps buffers, so it has to be seen on two passes in a row.
    if (usart2_pdc->PERIPH_RCR == 0) {
        if (rx_stall_seen) {
            counters.overflow++;
            rx_reset();
        } else {
            rx_stall_seen = true;
        }
        return;
    }
    rx_stall_seen = false;

    // w == RX_SIZE: stalled at the end of the ring since the check above; next pass handles it
    uint16_t w = rx_write_index();
    while ((w < RX_SIZE) && (rx_read != w)) {
        handle_byte(rx_ring[rx_read]);
        rx_read = (rx_read + 1) % RX_SIZE;
        if (rx_read % RX_HALF == 0) {
            rx_rearm();     // a half was just finished; let the PDC have it back
        }
        w = rx_write_index();
    }
    rx_rearm();
}

}  // namespace

void pi_link_init() {
    hw_init(PI_LINK_BAUD);
}

stat_t pi_link_periodic() {
    rx_poll();
    motion_periodic();
    return (STAT_OK);
}

bool pi_link_owns_machine() {
    return (owner != OWNER_USB);
}

void pi_link_usb_stop() {
    request_stop(REASON_USB_STOP);
}

stat_t pi_link_exec_move() {
    const uint32_t t0 = board_us();
    last_exec_us      = t0;
    if (stop_done) {
        return (STAT_NOOP);     // the final zero segment is queued; the main loop hands back to USB
    }

    const MotionCommand &c = cmd[cmd_live];
    if (owner == OWNER_PI) {
        uint8_t reason = stop_request;
        if ((reason == REASON_NONE) && (us_since(c.rx_us, t0) > (int32_t)link_cfg.watchdog_us)) {
            reason = REASON_WATCHDOG;
            diag.watchdog_trips++;
        }
        if (reason != REASON_NONE) {
            begin_stop(reason);
        }
    }

    float v1[PI_MOTORS];
    if (owner == OWNER_PI) {
        for (uint8_t m = 0; m < PI_MOTORS; m++) {
            v1[m] = c.vel[m];
        }
        queued_seq = c.seq;
        if (us_since(c.rx_us, t0) > (int32_t)STALE_US) {
            diag.stale_segments++;
            fb_stale = true;
        }
    } else {
        stop_t += link_cfg.seg_s;
        const float frac = (stop_t < stop_len) ? (1.0f - (stop_t / stop_len)) : 0.0f;
        for (uint8_t m = 0; m < PI_MOTORS; m++) {
            v1[m] = stop_v0[m] * frac;
        }
    }

    // Each motor ramps linearly from the previous segment's end rate to the new one
    float from[MOTORS] = {};
    float to[MOTORS]   = {};
    bool  moving       = false;
    for (uint8_t m = 0; m < PI_MOTORS; m++) {
        const float v0 = v_prev[m];
        const float vv = v1[m];
        if (((v0 > 0) && (vv < 0)) || ((v0 < 0) && (vv > 0))) {
            // the DDA can't reverse within a segment: run this one at the average rate
            from[m] = 0.5f * (v0 + vv);
            to[m]   = from[m];
        } else {
            from[m] = v0;
            to[m]   = vv;
        }
        if ((fabsf(v0) > 0) || (fabsf(vv) > 0)) {
            moving = true;
        }
        v_prev[m] = vv;
    }
    st_prep_velocity_segment(from, to, link_cfg.seg_ticks);
    diag.segments++;
    if ((owner == OWNER_STOPPING) && !moving) {
        stop_done = true;       // this all-zero segment folds the last steps into the step counters
    }

    const uint32_t dt = board_us() - t0;
    diag.exec_last_us = (uint16_t)dt;
    if (dt > diag.exec_max_us) {
        diag.exec_max_us = (uint16_t)dt;
    }
    return (STAT_OK);
}
