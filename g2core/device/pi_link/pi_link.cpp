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
 * v0: echo (PING/PONG) and STATUS only - see PROTOCOL.md. Nothing here commands motion.
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
#include "spindle.h"
#include "gpio.h"
#include "pi_link.h"

#include "MotatePins.h"
#include "MotateTimers.h"

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
    TYPE_PONG       = 0x81,
    TYPE_STATUS     = 0x82,
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

}  // namespace

void pi_link_init() {
    hw_init(PI_LINK_BAUD);
}

stat_t pi_link_periodic() {
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
        return (STAT_OK);
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
    return (STAT_OK);
}
