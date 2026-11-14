/* Host test of pilink.c's door hold: the real driver, built against hal_stub.h, talks over a
 * pseudo-terminal to a small model of the board (spindle start rules, door hold flag, ESC boot).
 * Each period runs pilink.read, sets LinuxCNC's side of the pins, runs pilink.write, then lets
 * the model answer. Run with run.sh. */
#define _GNU_SOURCE
#include "hal_stub.h"

#include <poll.h>
#include <pty.h>

#include "pilink.c"

static int master = -1;

/* ---- the board model ---- */
static struct {
    int      owner;             /* 0 USB, 1 PI */
    int      door, esc_booting;
    int      sp_req, sp_dir, waiting, refused;
    float    sp_rpm, sp_speed;
    int      hold;              /* MOTIONs left with hold_request set */
    int      motions;
} B;

static void board_open_door(void) {
    B.door = 1;
    if (B.sp_dir) {             /* the firmware's hold: spindle stopped, motion to rest */
        B.sp_dir   = 0;
        B.sp_speed = 0;
        B.hold     = 20;
    }
}

static void board_reply(uint16_t seq) {
    uint8_t p[40] = {0};
    p[1] = B.owner;
    p[3] = (B.hold > 0 ? 1 : 0) | (B.refused ? 0x10 : 0) | (B.waiting ? 0x20 : 0);
    B.refused = 0;
    put32(p + 4, (uint32_t)(rtapi_get_time() / 1000));
    put16(p + 24, seq);
    p[28] = (B.door ? 1 : 0) | (B.esc_booting ? 2 : 0);
    p[32] = B.sp_dir;
    p[33] = B.owner ? 5 : 1;
    putf(p + 36, B.sp_speed);

    uint8_t body[48], enc[64];
    body[0] = 0;
    body[1] = T_FEEDBACK;
    put16(body + 2, seq);
    memcpy(body + 4, p, 40);
    put32(body + 44, crc32(body, 44));
    int n = cobs_encode(body, 48, enc);
    enc[n++] = 0;
    if (write(master, enc, n) != n) {
        abort();
    }
}

static void board_frame(const uint8_t *f, int n) {
    const uint16_t seq = get16(f + 2);
    const uint8_t *p   = f + 4;
    if (f[1] == T_ENABLE) {
        B.owner = 1;
    } else if (f[1] == T_DISABLE) {
        B.owner = 0;
        B.sp_dir = 0;
        B.sp_speed = 0;
    } else if ((f[1] == T_MOTION) && (n == 4 + 24 + 4) && B.owner) {
        const int sp = p[16];
        B.sp_rpm     = getf(p + 20);
        B.motions++;
        if (sp && !B.sp_req) {              /* a start is an off-to-on change */
            if (B.door) {
                B.refused = 1;
            } else if (B.esc_booting) {
                B.waiting = 1;
            } else {
                B.sp_dir = 1;
            }
        }
        if (!sp) {
            B.sp_dir   = 0;
            B.sp_speed = 0;
            B.waiting  = 0;
        }
        if (B.waiting && B.door) {
            B.waiting = 0;
            B.refused = 1;
        } else if (B.waiting && !B.esc_booting) {
            B.waiting = 0;
            B.sp_dir  = 1;
        }
        if (B.sp_dir) {
            B.sp_speed = (B.sp_speed + 500 < B.sp_rpm) ? B.sp_speed + 500 : B.sp_rpm;
        }
        B.sp_req = sp;
        if (B.hold > 0) {
            B.hold--;
        }
    }
    board_reply(seq);
}

static void board_service(void) {
    static uint8_t buf[4096];
    static int     len = 0;
    struct pollfd  pf = {master, POLLIN, 0};
    if (poll(&pf, 1, 2) > 0) {
        const ssize_t r = read(master, buf + len, sizeof(buf) - len);
        if (r > 0) {
            len += r;
        }
    }
    int start = 0;
    for (int i = 0; i < len; i++) {
        if (buf[i] != 0) {
            continue;
        }
        uint8_t f[MAX_FRAME];
        const int n = cobs_decode(buf + start, i - start, f, sizeof(f));
        start = i + 1;
        if ((n >= 8) && (crc32(f, n - 4) == get32(f + n - 4))) {
            board_frame(f, n);
        }
    }
    memmove(buf, buf + start, len - start);
    len -= start;
    struct pollfd ps = {pl->fd, POLLIN, 0};
    poll(&ps, 1, 2);        /* let the reply reach the driver's side before the next read */
}

/* ---- LinuxCNC's side ---- */
static void *pin(const char *name) {
    for (int i = 0; i < stub_npins; i++) {
        if (!strcmp(stub_pins[i].name, name)) {
            return stub_pins[i].p;
        }
    }
    fprintf(stderr, "no pin %s\n", name);
    abort();
}
#define BIT(n)   (*(hal_bit_t *)pin(n))
#define S32(n)   (*(hal_s32_t *)pin(n))
#define FLOAT(n) (*(hal_float_t *)pin(n))

static void period(void) {
    stub_functs[0].f(stub_functs[0].arg, 1000000);     /* pilink.read */
    stub_functs[1].f(stub_functs[1].arg, 1000000);     /* pilink.write */
    board_service();
}
static void run(int n) {
    for (int i = 0; i < n; i++) {
        period();
    }
}

static int failures = 0;
static void check(int ok, const char *what) {
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) {
        failures++;
    }
}

int main(void) {
    char slave[256];
    int  sfd;
    if (openpty(&master, &sfd, slave, NULL, NULL) < 0) {
        perror("openpty");
        return 2;
    }
    port = slave;
    if (rtapi_app_main() != 0) {
        return 2;
    }
    if (strcmp(stub_functs[0].name, "pilink.read") || strcmp(stub_functs[1].name, "pilink.write")) {
        return 2;
    }
    for (int i = 0; i < MOTORS; i++) {
        char n[48];
        snprintf(n, sizeof(n), "pilink.%d.enable", i);
        BIT(n) = 1;
    }

    puts("1. machine on, spindle on");
    run(20);
    BIT("pilink.enable") = 1;
    run(20);
    check(BIT("pilink.in-control") && !BIT("pilink.fault"), "Pi in control");
    BIT("pilink.spindle-on")         = 1;
    FLOAT("pilink.spindle-speed-cmd") = 12000;
    run(40);
    check(BIT("pilink.spindle-at-speed") && !BIT("pilink.feed-inhibit") && S32("pilink.door-hold") == 0,
          "at speed, motion free");

    puts("2. the door opens: hold, pause, spindle request off");
    const int msgs0 = stub_msgs;
    board_open_door();
    run(2);
    check(S32("pilink.door-hold") == 1 && BIT("pilink.feed-inhibit") && BIT("pilink.pause"), "held and paused");
    check(B.sp_req == 0, "the board sees the spindle request drop");
    check(stub_msgs > msgs0 && strstr(stub_last_msg, "Close the door"), "message tells the operator what to do");
    run(25);
    BIT("pilink.program-paused") = 1;           /* halui paused the program */
    run(PAUSE_PULSE);
    check(!BIT("pilink.hold-request") && !BIT("pilink.pause") && BIT("pilink.feed-inhibit"),
          "board hold over, pause pulse over, motion still inhibited");

    puts("3. Resume with the door still open: paused again");
    BIT("pilink.program-paused") = 0;
    run(1);
    check(S32("pilink.door-hold") == 1 && BIT("pilink.pause") && B.sp_req == 0, "pause pulsed again, spindle off");
    BIT("pilink.program-paused") = 1;
    run(PAUSE_PULSE);

    puts("4. the door closes (ESC rebooting): nothing restarts by itself");
    B.door        = 0;
    B.esc_booting = 1;
    run(500);
    check(S32("pilink.door-hold") == 1 && B.sp_req == 0 && BIT("pilink.feed-inhibit"), "still held, spindle off");

    puts("5. Resume: the spindle restarts, motion waits for speed");
    BIT("pilink.program-paused") = 0;
    run(3);
    check(S32("pilink.door-hold") == 2 && B.sp_req == 1 && B.waiting, "restarting, the board waits for the ESC");
    run(50);
    check(BIT("pilink.feed-inhibit"), "motion still inhibited while the ESC boots");
    B.esc_booting = 0;
    run(40);
    check(S32("pilink.door-hold") == 0 && !BIT("pilink.feed-inhibit") && BIT("pilink.spindle-at-speed"),
          "at speed: hold over, motion free");
    check(strstr(stub_last_msg, "resuming") != NULL, "says it's resuming");
    check(!BIT("pilink.fault") && BIT("pilink.in-control"), "no fault through the whole hold");

    puts("6. the door opens again while the spindle restarts");
    board_open_door();
    run(30);
    BIT("pilink.program-paused") = 1;
    run(PAUSE_PULSE);
    B.door        = 0;
    B.esc_booting = 1;
    run(10);
    BIT("pilink.program-paused") = 0;
    run(10);
    check(S32("pilink.door-hold") == 2 && B.waiting, "restarting, waiting for the ESC");
    B.door = 1;
    run(3);
    check(S32("pilink.door-hold") == 1 && B.sp_req == 0 && !BIT("pilink.fault"), "back to held, no fault");

    puts("7. spindle turned off during a hold: nothing to restart");
    BIT("pilink.spindle-on") = 0;
    run(3);
    check(S32("pilink.door-hold") == 0 && !BIT("pilink.feed-inhibit"), "hold over, motion free");

    puts("8. M3 with the door open (no hold): refused, machine off");
    BIT("pilink.program-paused") = 0;
    BIT("pilink.spindle-on")     = 1;
    run(3);
    check(BIT("pilink.fault") && strstr(stub_last_msg, "refused") != NULL, "fault: the board refused the start");
    check(B.sp_req == 0, "the request drops after the refusal");

    printf("\n%s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
