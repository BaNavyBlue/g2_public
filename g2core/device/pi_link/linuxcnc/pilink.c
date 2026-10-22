/*
 * pilink.c - LinuxCNC HAL driver for the g2core Pi link v1 (see HAL-DRIVER.md and
 * ../PROTOCOL-v1-draft.md).
 *
 * Every servo period:
 *   pilink.read   (first in the thread) reads whatever FEEDBACK has arrived, without blocking,
 *                 and updates position-fb, the inputs and the board state.
 *   pilink.write  (last in the thread) turns motion's new position-cmd into one velocity per
 *                 motor and sends one MOTION frame.
 *
 * The position loop runs here: velocity = feed-forward + P x (reference - feedback), where the
 * reference is the commanded position `latency` periods before the FEEDBACK was sampled. Each
 * FEEDBACK echoes the seq of the MOTION it answers, so a reply that arrives late is still
 * compared with the right command.
 *
 * Build on the Pi:  halcompile --compile pilink.c
 * Load:             loadrt /full/path/pilink.so [port=/dev/ttyAMA0] [baud=2000000]
 */

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

MODULE_AUTHOR("BaNavyBlue");
MODULE_DESCRIPTION("g2core Pi link v1 driver (velocity commands, step-count feedback)");
MODULE_LICENSE("GPL");

static char *port = "/dev/ttyAMA0";
RTAPI_MP_STRING(port, "serial device wired to the board's J17");
static int baud = 2000000;
RTAPI_MP_INT(baud, "link baud rate: 115200, 1000000, 2000000 or 3000000");
/* Only for loading an uninstalled copy: halcmd waits for a component named like the module
 * argument, so "loadrt ../../../../home/u/p/pilink hal_name=../../../../home/u/p/pilink" works
 * without installing. Pin names are always pilink.* */
static char *hal_name = "pilink";
RTAPI_MP_STRING(hal_name, "HAL component name (default pilink)");

#define MOTORS      4
#define INPUTS      16
#define HIST        64          /* commands remembered for FEEDBACK alignment (power of 2) */
#define MAX_FRAME   128         /* decoded frame limit (PROTOCOL.md) */

enum { T_ENABLE = 0x10, T_MOTION = 0x11, T_DISABLE = 0x12, T_SAFETY = 0x13, T_FEEDBACK = 0x91 };
enum { OWNER_USB = 0, OWNER_PI = 1, OWNER_STOPPING = 2 };
enum { RESULT_OK = 0, RESULT_TIMEOUT = 100 };   /* 1-6 come from the board */
enum { KIND_NONE = 0, KIND_MOTION, KIND_OTHER };
enum { ST_IDLE = 0, ST_ENABLING, ST_RUNNING, ST_DISABLING, ST_WAIT_USB };

#define POLL_PERIODS        10      /* zero MOTION every 10 ms while not in control */
#define REPLY_TIMEOUT       50      /* periods to wait for an ENABLE/DISABLE/SAFETY reply */
#define LINK_LOST_PERIODS   50      /* no FEEDBACK for this long: link down */
#define RUN_LOST_PERIODS    20      /* ...while running: stop sending, let the watchdog stop it */
#define STOP_TIMEOUT        3000    /* periods to wait for a board stop to finish */
#define RESET_GAP_US        500000  /* a board restart takes longer than this without replies */
#define RESET_MISMATCH_US   200000  /* ...and its clock then disagrees with the Pi's by this much */
#define DDA_MAX_RATE        57950.0 /* ENABLE limit: 0.95 x 61 kHz */

typedef struct {
    /* pins */
    hal_float_t *position_cmd;      /* in, machine units */
    hal_bit_t   *enable;            /* in */
    hal_float_t *position_fb;       /* out, machine units */
    hal_s32_t   *counts;            /* out, unwrapped steps (low 32 bits) */
    hal_float_t *velocity_cmd;      /* out, units/s sent in the last MOTION */
    hal_float_t *error;             /* out, steps: reference - feedback */
    /* parameters */
    hal_float_t scale;              /* steps per unit */
    hal_float_t maxvel;             /* units/s, the driver's clamp */
    hal_float_t maxaccel;           /* units/s^2, the driver's limit */
    hal_float_t board_maxvel;       /* units/s, ENABLE max_vel; 0 = 1.5 x maxvel */
    hal_float_t stop_decel;         /* units/s^2, the board stop */
    hal_float_t enable_tolerance;   /* units: ENABLE refused if position-cmd is further from fb */
    /* state */
    int64_t count_acc;              /* unwrapped steps */
    int32_t count_raw;
    int64_t fb_count;               /* unwrapped steps at the newest MOTION-aligned FEEDBACK */
    double  cmd_prev;               /* steps */
    double  v_prev;                 /* steps/s */
    double  hist[HIST];             /* commanded steps, indexed by MOTION seq */
} motor_t;

typedef struct {
    motor_t m[MOTORS];

    hal_bit_t   *enable;            /* in: machine on (motion.motion-enabled) */
    hal_bit_t   *estop_reset;       /* in: E-stop reset request (iocontrol.0.user-request-enable) */
    hal_bit_t   *estop_ok;          /* out: link up, board E-stop clear, no alarm */
    hal_bit_t   *fault;             /* out: ENABLE refused or control lost; clears with enable */
    hal_bit_t   *in_control;        /* out */
    hal_bit_t   *link_ok;           /* out */
    hal_bit_t   *position_lost;     /* out: the board reset; positions are no longer valid */
    hal_bit_t   *door_open;         /* out: safe bit 0 */
    hal_bit_t   *esc_not_ready;     /* out: safe bit 1 */
    hal_bit_t   *hold_request;      /* out: FEEDBACK flags bit 0 */
    hal_bit_t   *estop_pressed;     /* out: estp bit 0 */
    hal_bit_t   *input[INPUTS];     /* out: input-01 .. input-16 = in1 .. in16 */
    hal_u32_t   *inputs;            /* out */
    hal_s32_t   *owner;             /* out: 0 USB, 1 PI, 2 STOPPING */
    hal_s32_t   *release_reason;    /* out: board stop table */
    hal_s32_t   *result;            /* out: last refusal (board result code, 100 = timeout) */
    hal_s32_t   *board_state;       /* out: g2core stat */
    hal_s32_t   *estp;              /* out */
    hal_s32_t   *safe;              /* out */
    hal_s32_t   *state;             /* out: driver state */
    hal_float_t *spindle_speed;     /* out */
    hal_u32_t   *feedback_age;      /* out: periods since the last FEEDBACK */
    hal_u32_t   *rx_frames;         /* out */
    hal_u32_t   *rx_errors;         /* out: bad COBS, CRC, length or version */
    hal_u32_t   *tx_errors;         /* out: short or failed writes */
    hal_u32_t   *stale_segments;    /* out: board counter */
    hal_u32_t   *clamped;           /* out: FEEDBACKs that reported a board velocity clamp */
    hal_u32_t   *late;              /* out: periods with no new FEEDBACK while running */
    hal_u32_t   *board_time;        /* out: board clock of the newest FEEDBACK, us */
    hal_u32_t   *rx_gap_max;        /* out: longest gap between FEEDBACKs while running, us */
    hal_s32_t   *reset_board_dt;    /* out: what tripped position-lost: board clock step, us */
    hal_s32_t   *reset_pi_dt;       /* out: ...and the Pi's, us */

    hal_float_t p_gain;             /* 1/s */
    hal_float_t latency;            /* periods between a command and its effect on the counts */
    hal_float_t deadband;           /* steps */
    hal_u32_t   watchdog_ms;
    hal_u32_t   segment_us;
    hal_u32_t   hold_ms;

    /* state */
    int      fd;
    int      st;
    uint16_t seq;
    uint16_t req_seq;               /* the ENABLE/DISABLE/SAFETY in flight */
    int      req_type;              /* 0 = none */
    int      req_age;
    int      reply_ready;           /* a reply to req_seq arrived */
    int      reply_result;
    int      safety_queue[2];       /* SAFETY actions still to send */
    int      safety_n;
    uint8_t  kind[HIST];            /* what each seq was */
    int      have_fb;               /* any FEEDBACK yet */
    uint16_t fb_seq;                /* seq of the newest MOTION-aligned FEEDBACK */
    int      fb_aligned;            /* fb_seq belongs to this RUNNING session */
    uint8_t  fb_owner;
    uint32_t fb_time;
    int64_t  pi_us;                 /* Pi time, monotonic */
    int64_t  fb_pi_us;              /* Pi time of the newest FEEDBACK */
    uint32_t age;
    int      poll;
    int      stop_wait;
    int      prev_enable;
    int      prev_reset;
    double   period_s;

    uint8_t  rx[2048];
    int      rx_len;
} pilink_t;

static int comp_id;
static pilink_t *pl;
static uint32_t crc_table[256];

/* ---- framing ------------------------------------------------------------------------- */

static void crc_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        crc_table[i] = c;
    }
}

static uint32_t crc32(const uint8_t *p, int n) {
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c = crc_table[(c ^ *p++) & 0xFF] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

static int cobs_encode(const uint8_t *in, int n, uint8_t *out) {
    int code_pos = 0, o = 1;
    uint8_t code = 1;
    for (int i = 0; i < n; i++) {
        if (in[i] == 0) {
            out[code_pos] = code;
            code_pos = o++;
            code = 1;
        } else {
            out[o++] = in[i];
            if (++code == 0xFF) {
                out[code_pos] = code;
                code_pos = o++;
                code = 1;
            }
        }
    }
    out[code_pos] = code;
    return o;
}

static int cobs_decode(const uint8_t *in, int n, uint8_t *out, int max) {
    int i = 0, o = 0;
    while (i < n) {
        const int code = in[i++];
        if ((code == 0) || (i + code - 1 > n) || (o + code - 1 > max)) {
            return -1;
        }
        for (int k = 1; k < code; k++) {
            out[o++] = in[i++];
        }
        if ((code < 0xFF) && (i < n)) {
            if (o >= max) {
                return -1;
            }
            out[o++] = 0;
        }
    }
    return o;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void putf(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); put32(p, u); }
static uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t get32(const uint8_t *p) {
    return p[0] | (p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float getf(const uint8_t *p) { uint32_t u = get32(p); float f; memcpy(&f, &u, 4); return f; }

static uint16_t send_frame(pilink_t *d, int type, const uint8_t *payload, int n, int kind) {
    uint8_t body[8 + 64], enc[sizeof(body) + 4];
    const uint16_t seq = ++d->seq;
    body[0] = 0;
    body[1] = type;
    put16(body + 2, seq);
    if (n > 0) {
        memcpy(body + 4, payload, n);
    }
    put32(body + 4 + n, crc32(body, 4 + n));
    int len = cobs_encode(body, 8 + n, enc);
    enc[len++] = 0;
    const ssize_t w = write(d->fd, enc, len);     /* O_NONBLOCK: never waits */
    if (w != len) {
        (*d->tx_errors)++;
    }
    d->kind[seq & (HIST - 1)] = kind;
    return seq;
}

static void send_request(pilink_t *d, int type, const uint8_t *payload, int n) {
    d->req_seq     = send_frame(d, type, payload, n, KIND_OTHER);
    d->req_type    = type;
    d->req_age     = 0;
    d->reply_ready = 0;
}

static void send_motion(pilink_t *d, const float v[MOTORS], int kind) {
    uint8_t p[24] = {0};
    for (int i = 0; i < MOTORS; i++) {
        putf(p + 4 * i, v[i]);
    }
    send_frame(d, T_MOTION, p, sizeof(p), kind);
}

static void send_poll(pilink_t *d) {
    const float zero[MOTORS] = {0};
    send_motion(d, zero, KIND_OTHER);
}

static void send_enable(pilink_t *d) {
    uint8_t p[52] = {0};
    put16(p + 0, d->watchdog_ms);
    put16(p + 2, d->segment_us);
    put16(p + 4, d->hold_ms);
    for (int i = 0; i < MOTORS; i++) {
        motor_t *m = &d->m[i];
        double vmax = (m->board_maxvel > 0) ? m->board_maxvel : 1.5 * m->maxvel;
        vmax *= (m->scale < 0) ? -m->scale : m->scale;
        if (vmax > DDA_MAX_RATE) {
            vmax = DDA_MAX_RATE;
        }
        double decel = m->stop_decel * ((m->scale < 0) ? -m->scale : m->scale);
        putf(p + 8 + 4 * i, (float)vmax);
        putf(p + 24 + 4 * i, (float)decel);
    }
    send_request(d, T_ENABLE, p, sizeof(p));
}

/* ---- receive ------------------------------------------------------------------------- */

static void handle_feedback(pilink_t *d, uint16_t seq, const uint8_t *p) {
    const uint32_t t = get32(p + 4);
    if (d->have_fb) {
        /* A board restart: a gap without replies, after which its clock disagrees with ours.
         * (A reply held up in the tty layer is a gap too, but the clocks then agree.) */
        const int64_t board_dt = (uint32_t)(t - d->fb_time);
        const int64_t pi_dt    = d->pi_us - d->fb_pi_us;
        const int64_t allowed  = RESET_MISMATCH_US + pi_dt / 500;     /* + 0.2% clock drift */
        if ((d->st == ST_RUNNING) && (pi_dt > (int64_t)*d->rx_gap_max)) {
            *d->rx_gap_max = (uint32_t)pi_dt;
        }
        if ((pi_dt > RESET_GAP_US) && ((board_dt - pi_dt > allowed) || (pi_dt - board_dt > allowed))) {
            if (!*d->position_lost) {
                rtapi_print_msg(RTAPI_MSG_ERR, "pilink: the board restarted: positions are lost, re-home\n");
                *d->reset_board_dt = (int32_t)board_dt;
                *d->reset_pi_dt    = (int32_t)pi_dt;
            }
            *d->position_lost = 1;
        }
    }
    d->fb_time  = t;
    d->fb_pi_us = d->pi_us;
    *d->board_time = t;

    const int idx = seq & (HIST - 1);
    const int aligned = (d->kind[idx] == KIND_MOTION) && (d->st == ST_RUNNING) &&
                        ((uint16_t)(d->seq - seq) < HIST - 4);
    for (int i = 0; i < MOTORS; i++) {
        motor_t *m = &d->m[i];
        const int32_t raw = (int32_t)get32(p + 8 + 4 * i);
        if (d->have_fb) {
            m->count_acc += (int32_t)(raw - m->count_raw);
        } else {
            m->count_acc = raw;
        }
        m->count_raw = raw;
        if (aligned) {
            m->fb_count = m->count_acc;
        }
    }
    if (aligned) {
        d->fb_seq     = seq;
        d->fb_aligned = 1;
    }
    d->have_fb  = 1;
    d->fb_owner = p[1];
    *d->owner          = p[1];
    *d->release_reason = p[2];
    *d->hold_request   = (p[3] & 1) != 0;
    if (p[3] & 2) {
        (*d->clamped)++;
    }
    *d->stale_segments = get16(p + 26);
    *d->safe           = p[28];
    *d->door_open      = (p[28] & 1) != 0;
    *d->esc_not_ready  = (p[28] & 2) != 0;
    *d->estp           = p[29];
    *d->estop_pressed  = (p[29] & 1) != 0;
    const uint16_t in  = get16(p + 30);
    *d->inputs         = in;
    for (int i = 0; i < INPUTS; i++) {
        *d->input[i] = (in >> i) & 1;
    }
    *d->board_state   = p[33];
    *d->spindle_speed = getf(p + 36);

    if (d->req_type && (seq == d->req_seq)) {
        d->reply_ready  = 1;
        d->reply_result = p[0];
    }
    d->age = 0;
}

static void receive(pilink_t *d) {
    const ssize_t r = read(d->fd, d->rx + d->rx_len, sizeof(d->rx) - d->rx_len);   /* O_NONBLOCK */
    if (r > 0) {
        d->rx_len += r;
    }

    int start = 0;
    for (int i = 0; i < d->rx_len; i++) {
        if (d->rx[i] != 0) {
            continue;
        }
        uint8_t f[MAX_FRAME];
        const int n = cobs_decode(d->rx + start, i - start, f, sizeof(f));
        start = i + 1;
        if (n < 8) {
            if (n != 0) {           /* an empty frame is just a stray delimiter */
                (*d->rx_errors)++;
            }
            continue;
        }
        if ((f[0] != 0) || (crc32(f, n - 4) != get32(f + n - 4))) {
            (*d->rx_errors)++;
            continue;
        }
        (*d->rx_frames)++;
        if ((f[1] == T_FEEDBACK) && (n == 4 + 40 + 4)) {
            handle_feedback(d, get16(f + 2), f + 4);
        }
    }
    if (start > 0) {
        memmove(d->rx, d->rx + start, d->rx_len - start);
        d->rx_len -= start;
    }
    if (d->rx_len == (int)sizeof(d->rx)) {      /* full and no delimiter: resync on the next 0x00 */
        d->rx_len = 0;
        (*d->rx_errors)++;
    }
}

/* ---- HAL functions ------------------------------------------------------------------- */

static int alarmed(int stat) { return (stat == 2) || (stat == 12) || (stat == 13); }

static void pilink_read(void *arg, long period) {
    pilink_t *d = arg;
    d->period_s = period * 1e-9;
    d->pi_us    = rtapi_get_time() / 1000;
    if (d->age < 0xFFFFFFFF) {
        d->age++;
    }
    receive(d);

    *d->feedback_age = d->age;
    *d->link_ok      = d->have_fb && (d->age < LINK_LOST_PERIODS);
    *d->estop_ok     = *d->link_ok && (*d->estp == 0) && !alarmed(*d->board_state) && !*d->position_lost;
    for (int i = 0; i < MOTORS; i++) {
        motor_t *m = &d->m[i];
        *m->counts      = (int32_t)m->count_acc;
        *m->position_fb = (m->scale != 0) ? (double)m->count_acc / m->scale : 0;
    }
}

static void set_fault(pilink_t *d, int result, const char *why) {
    if (!*d->fault) {
        rtapi_print_msg(RTAPI_MSG_ERR, "pilink: %s (result %d, owner %d, release reason %d)\n",
                        why, result, *d->owner, *d->release_reason);
    }
    *d->fault = 1;
    if (result) {
        *d->result = result;
    }
}

/* LinuxCNC holds position-cmd at position-fb while the machine is off. If they disagree at
 * enable, the loop would drive the motor there at maxvel: refuse instead. */
static int command_matches_feedback(pilink_t *d) {
    for (int i = 0; i < MOTORS; i++) {
        const motor_t *m = &d->m[i];
        if (!*m->enable || (m->scale == 0)) {
            continue;
        }
        const double diff = *m->position_cmd - (double)m->count_acc / m->scale;
        if ((diff > m->enable_tolerance) || (diff < -m->enable_tolerance)) {
            rtapi_print_msg(RTAPI_MSG_ERR, "pilink: motor %d position-cmd is %.4f from position-fb\n", i, diff);
            return 0;
        }
    }
    return 1;
}

static void start_running(pilink_t *d) {
    for (int i = 0; i < MOTORS; i++) {
        motor_t *m = &d->m[i];
        const double c = *m->position_cmd * m->scale;
        for (int k = 0; k < HIST; k++) {
            m->hist[k] = c;
        }
        m->cmd_prev = c;
        m->v_prev   = 0;
        m->fb_count = m->count_acc;
    }
    d->fb_aligned  = 0;
    *d->rx_gap_max = 0;
    d->st = ST_RUNNING;
    *d->in_control = 1;
}

/* Reference: the commanded position `latency` periods before the start of MOTION `seq`'s
 * period, interpolated. hist[s] is where motion wanted the motor at the end of period s. */
static double reference(const motor_t *m, uint16_t seq, double latency) {
    if (latency < 0) {
        latency = 0;
    }
    if (latency > HIST - 8) {
        latency = HIST - 8;
    }
    const int    j = (int)latency;
    const double f = latency - j;
    const double a = m->hist[(uint16_t)(seq - 1 - j) & (HIST - 1)];
    const double b = m->hist[(uint16_t)(seq - 2 - j) & (HIST - 1)];
    return a - f * (a - b);
}

static void run_motion(pilink_t *d) {
    const double T = d->period_s;
    float v[MOTORS];
    const uint16_t seq = d->seq + 1;        /* the MOTION about to go out */
    for (int i = 0; i < MOTORS; i++) {
        motor_t *m = &d->m[i];
        const double s     = m->scale;
        const double abs_s = (s < 0) ? -s : s;
        const double c     = *m->position_cmd * s;
        double vel         = (c - m->cmd_prev) / T;
        m->cmd_prev        = c;
        m->hist[seq & (HIST - 1)] = c;

        double err = 0;
        if (d->fb_aligned) {
            err = reference(m, d->fb_seq, d->latency) - (double)m->fb_count;
            if (err > d->deadband) {
                vel += d->p_gain * (err - d->deadband);
            } else if (err < -d->deadband) {
                vel += d->p_gain * (err + d->deadband);
            }
        }
        *m->error = err;
        if (!*m->enable) {
            vel = 0;
        }
        const double vmax = m->maxvel * abs_s;
        if (vel > vmax) { vel = vmax; }
        if (vel < -vmax) { vel = -vmax; }
        const double dv = m->maxaccel * abs_s * T;
        if (vel > m->v_prev + dv) { vel = m->v_prev + dv; }
        if (vel < m->v_prev - dv) { vel = m->v_prev - dv; }
        m->v_prev        = vel;
        v[i]             = (float)vel;
        *m->velocity_cmd = (s != 0) ? vel / s : 0;
    }
    send_motion(d, v, KIND_MOTION);
}

static void zero_outputs(pilink_t *d) {
    for (int i = 0; i < MOTORS; i++) {
        *d->m[i].velocity_cmd = 0;
        d->m[i].v_prev        = 0;
    }
}

static void pilink_write(void *arg, long period) {
    pilink_t *d = arg;
    (void)period;
    const int enable = *d->enable;
    const int reset  = *d->estop_reset;

    if (!enable) {
        *d->fault = 0;      /* machine off clears a fault, as an amp fault does */
    }
    if (reset && !d->prev_reset && (d->st == ST_IDLE) && (d->fb_owner == OWNER_USB) && *d->link_ok &&
        !*d->estop_pressed && (d->safety_n == 0)) {
        if (*d->estp & 2) {
            d->safety_queue[d->safety_n++] = 1;     /* acknowledge */
        }
        if ((*d->estp & 2) || alarmed(*d->board_state)) {
            d->safety_queue[d->safety_n++] = 2;     /* clear */
        }
    }
    d->prev_reset  = reset;
    d->prev_enable = enable;

    if (d->req_type) {
        d->req_age++;
    }

    switch (d->st) {
    case ST_IDLE:
        *d->in_control = 0;
        zero_outputs(d);
        if (d->req_type == T_SAFETY) {
            if (d->reply_ready || (d->req_age > REPLY_TIMEOUT)) {
                d->req_type = 0;
            }
        } else if (d->safety_n > 0) {
            uint8_t action = d->safety_queue[0];
            d->safety_queue[0] = d->safety_queue[1];
            d->safety_n--;
            send_request(d, T_SAFETY, &action, 1);
        } else if (enable && !*d->fault && *d->link_ok && (d->fb_owner == OWNER_USB)) {
            if (*d->position_lost) {
                set_fault(d, 0, "positions lost after a board restart: restart LinuxCNC and re-home");
            } else if (!command_matches_feedback(d)) {
                set_fault(d, 0, "position-cmd doesn't match position-fb at enable");
            } else {
                send_enable(d);
                d->st = ST_ENABLING;
                break;
            }
        }
        if (++d->poll >= POLL_PERIODS) {
            d->poll = 0;
            send_poll(d);
        }
        break;

    case ST_ENABLING:
        if (d->reply_ready) {
            d->req_type = 0;
            if (d->reply_result == RESULT_OK) {
                start_running(d);
                if (!enable) {      /* switched off while we waited */
                    send_request(d, T_DISABLE, NULL, 0);
                    d->st = ST_DISABLING;
                    d->stop_wait = 0;
                } else {
                    run_motion(d);
                }
            } else {
                set_fault(d, d->reply_result, "the board refused ENABLE");
                d->st = ST_IDLE;
            }
        } else if (d->req_age > REPLY_TIMEOUT) {
            d->req_type = 0;
            set_fault(d, RESULT_TIMEOUT, "no reply to ENABLE");
            d->st = ST_WAIT_USB;
        }
        break;

    case ST_RUNNING:
        if (d->age >= 1) {
            (*d->late)++;
        }
        if (d->fb_aligned && (d->fb_owner != OWNER_PI)) {
            set_fault(d, 0, "the board took control back");
            *d->in_control = 0;
            zero_outputs(d);
            d->st = ST_WAIT_USB;
            send_poll(d);
            break;
        }
        if (d->age > RUN_LOST_PERIODS) {
            /* No replies: stop sending, so the board's watchdog stops the machine */
            set_fault(d, RESULT_TIMEOUT, "no FEEDBACK from the board");
            *d->in_control = 0;
            zero_outputs(d);
            d->st = ST_WAIT_USB;
            break;
        }
        if (!enable) {
            send_request(d, T_DISABLE, NULL, 0);
            *d->in_control = 0;
            zero_outputs(d);
            d->st = ST_DISABLING;
            d->stop_wait = 0;
            break;
        }
        run_motion(d);
        break;

    case ST_DISABLING:
    case ST_WAIT_USB:
        if (d->req_type && (d->reply_ready || (d->req_age > REPLY_TIMEOUT))) {
            d->req_type = 0;
        }
        if (d->fb_owner == OWNER_USB && d->age == 0 && !d->req_type) {
            d->st = ST_IDLE;
            d->poll = 0;
            break;
        }
        if (++d->stop_wait > STOP_TIMEOUT) {
            set_fault(d, RESULT_TIMEOUT, "the board did not hand control back");
            d->stop_wait = 0;
        }
        if (++d->poll >= POLL_PERIODS) {
            d->poll = 0;
            send_poll(d);
        }
        break;
    }
    *d->state = d->st;
}

/* ---- setup --------------------------------------------------------------------------- */

static speed_t baud_code(int b) {
    switch (b) {
    case 115200:  return B115200;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    default:      return 0;
    }
}

static int open_port(void) {
    const speed_t code = baud_code(baud);
    if (!code) {
        rtapi_print_msg(RTAPI_MSG_ERR, "pilink: unsupported baud %d\n", baud);
        return -1;
    }
    const int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "pilink: can't open %s: %s\n", port, strerror(errno));
        return -1;
    }
    struct termios t;
    if (tcgetattr(fd, &t) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "pilink: tcgetattr %s: %s\n", port, strerror(errno));
        close(fd);
        return -1;
    }
    cfmakeraw(&t);
    t.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB);
    t.c_cflag |= CLOCAL | CREAD | CS8;
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 0;
    cfsetispeed(&t, code);
    cfsetospeed(&t, code);
    if (tcsetattr(fd, TCSANOW, &t) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "pilink: tcsetattr %s: %s\n", port, strerror(errno));
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

#define PIN(call) do { if ((r = (call)) < 0) goto fail; } while (0)

static const double def_scale[MOTORS]  = {200, 200, 400, 17.7778};
static const double def_vel[MOTORS]    = {10, 10, 5, 10};
static const double def_accel[MOTORS]  = {200, 200, 100, 100};
static const double def_decel[MOTORS]  = {500, 500, 250, 500};

int rtapi_app_main(void) {
    int r;
    comp_id = hal_init(hal_name);
    if (comp_id < 0) {
        return comp_id;
    }
    pl = hal_malloc(sizeof(pilink_t));
    if (!pl) {
        hal_exit(comp_id);
        return -ENOMEM;
    }
    memset(pl, 0, sizeof(*pl));
    pilink_t *d = pl;
    d->fd = -1;
    crc_init();

    for (int i = 0; i < MOTORS; i++) {
        motor_t *m = &d->m[i];
        PIN(hal_pin_float_newf(HAL_IN,  &m->position_cmd, comp_id, "pilink.%d.position-cmd", i));
        PIN(hal_pin_bit_newf  (HAL_IN,  &m->enable,       comp_id, "pilink.%d.enable", i));
        PIN(hal_pin_float_newf(HAL_OUT, &m->position_fb,  comp_id, "pilink.%d.position-fb", i));
        PIN(hal_pin_s32_newf  (HAL_OUT, &m->counts,       comp_id, "pilink.%d.counts", i));
        PIN(hal_pin_float_newf(HAL_OUT, &m->velocity_cmd, comp_id, "pilink.%d.velocity-cmd", i));
        PIN(hal_pin_float_newf(HAL_OUT, &m->error,        comp_id, "pilink.%d.error", i));
        PIN(hal_param_float_newf(HAL_RW, &m->scale,        comp_id, "pilink.%d.scale", i));
        PIN(hal_param_float_newf(HAL_RW, &m->maxvel,       comp_id, "pilink.%d.maxvel", i));
        PIN(hal_param_float_newf(HAL_RW, &m->maxaccel,     comp_id, "pilink.%d.maxaccel", i));
        PIN(hal_param_float_newf(HAL_RW, &m->board_maxvel, comp_id, "pilink.%d.board-maxvel", i));
        PIN(hal_param_float_newf(HAL_RW, &m->stop_decel,   comp_id, "pilink.%d.stop-decel", i));
        PIN(hal_param_float_newf(HAL_RW, &m->enable_tolerance, comp_id, "pilink.%d.enable-tolerance", i));
        m->scale    = def_scale[i];
        m->maxvel   = def_vel[i];
        m->maxaccel = def_accel[i];
        m->stop_decel = def_decel[i];
        m->enable_tolerance = 0.05;
    }
    PIN(hal_pin_bit_newf(HAL_IN,  &d->enable,        comp_id, "pilink.enable"));
    PIN(hal_pin_bit_newf(HAL_IN,  &d->estop_reset,   comp_id, "pilink.estop-reset"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->estop_ok,      comp_id, "pilink.estop-ok"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->fault,         comp_id, "pilink.fault"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->in_control,    comp_id, "pilink.in-control"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->link_ok,       comp_id, "pilink.link-ok"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->position_lost, comp_id, "pilink.position-lost"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->door_open,     comp_id, "pilink.door-open"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->esc_not_ready, comp_id, "pilink.esc-not-ready"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->hold_request,  comp_id, "pilink.hold-request"));
    PIN(hal_pin_bit_newf(HAL_OUT, &d->estop_pressed, comp_id, "pilink.estop-pressed"));
    for (int i = 0; i < INPUTS; i++) {
        PIN(hal_pin_bit_newf(HAL_OUT, &d->input[i], comp_id, "pilink.input-%02d", i + 1));
    }
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->inputs,         comp_id, "pilink.inputs"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->owner,          comp_id, "pilink.owner"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->release_reason, comp_id, "pilink.release-reason"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->result,         comp_id, "pilink.result"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->board_state,    comp_id, "pilink.board-state"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->estp,           comp_id, "pilink.estp"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->safe,           comp_id, "pilink.safe"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->state,          comp_id, "pilink.state"));
    PIN(hal_pin_float_newf(HAL_OUT, &d->spindle_speed,  comp_id, "pilink.spindle-speed"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->feedback_age,   comp_id, "pilink.feedback-age"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->rx_frames,      comp_id, "pilink.rx-frames"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->rx_errors,      comp_id, "pilink.rx-errors"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->tx_errors,      comp_id, "pilink.tx-errors"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->stale_segments, comp_id, "pilink.stale-segments"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->clamped,        comp_id, "pilink.clamped"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->late,           comp_id, "pilink.late"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->board_time,     comp_id, "pilink.board-time"));
    PIN(hal_pin_u32_newf  (HAL_OUT, &d->rx_gap_max,     comp_id, "pilink.rx-gap-max"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->reset_board_dt, comp_id, "pilink.reset-board-dt"));
    PIN(hal_pin_s32_newf  (HAL_OUT, &d->reset_pi_dt,    comp_id, "pilink.reset-pi-dt"));
    PIN(hal_param_float_newf(HAL_RW, &d->p_gain,      comp_id, "pilink.p-gain"));
    PIN(hal_param_float_newf(HAL_RW, &d->latency,     comp_id, "pilink.latency"));
    PIN(hal_param_float_newf(HAL_RW, &d->deadband,    comp_id, "pilink.deadband"));
    PIN(hal_param_u32_newf  (HAL_RW, &d->watchdog_ms, comp_id, "pilink.watchdog-ms"));
    PIN(hal_param_u32_newf  (HAL_RW, &d->segment_us,  comp_id, "pilink.segment-us"));
    PIN(hal_param_u32_newf  (HAL_RW, &d->hold_ms,     comp_id, "pilink.hold-ms"));
    d->p_gain      = 30;
    d->latency     = 2.0;
    d->deadband    = 0.5;
    d->watchdog_ms = 5;
    d->segment_us  = 1000;
    d->hold_ms     = 250;
    d->fb_owner    = OWNER_USB;

    d->fd = open_port();
    if (d->fd < 0) {
        hal_exit(comp_id);
        return -ENODEV;
    }
    PIN(hal_export_funct("pilink.read",  pilink_read,  d, 1, 0, comp_id));
    PIN(hal_export_funct("pilink.write", pilink_write, d, 1, 0, comp_id));
    rtapi_print_msg(RTAPI_MSG_INFO, "pilink: %s at %d baud\n", port, baud);
    hal_ready(comp_id);
    return 0;

fail:
    rtapi_print_msg(RTAPI_MSG_ERR, "pilink: HAL setup failed (%d)\n", r);
    if (d->fd >= 0) {
        close(d->fd);
    }
    hal_exit(comp_id);
    return r;
}

void rtapi_app_exit(void) {
    if (pl && (pl->fd >= 0)) {
        if (pl->st == ST_RUNNING) {     /* stop now rather than on the watchdog */
            send_frame(pl, T_DISABLE, NULL, 0, KIND_OTHER);
            tcdrain(pl->fd);
        }
        close(pl->fd);
    }
    hal_exit(comp_id);
}
