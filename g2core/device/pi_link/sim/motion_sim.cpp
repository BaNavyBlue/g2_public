// Motion simulation for the Pi link: the real pi_link.cpp (built with PI_LINK_MOTION=1) and the
// real st_prep_velocity_segment() from stepper.cpp, driven against a model of g2core's step
// generator (DDA ticks, segment load, direction flips, step counting), on a virtual clock.
#include "sim_env.h"
#include "pi_link.h"
#include <sys/mman.h>
#include <zlib.h>
#include <cstdio>
#include <vector>
#include <random>
#include <functional>

cmMachine_t  cm1{};
cmMachine_t *cm = &cm1;
mpPlanner_t  mp_storage;
mpPlanner_t *mp = &mp_storage;
Stepper      motor_objs[MOTORS];
Stepper     *Motors[MOTORS] = {&motor_objs[0], &motor_objs[1], &motor_objs[2], &motor_objs[3]};
enEncoders_t en{};
Reader       readers[16];
Reader      *in_r[16];
SysTick_t    sim_systick{0, 83999};
SCB_t        sim_scb{0};
stPrep_t     st_pre{PREP_BUFFER_OWNED_BY_EXEC, 0, nullptr, 0, {}};
stConfig_t   st_cfg{};

static uint64_t sim_tick = 0;                 // DDA ticks since start
static double   now_us() { return sim_tick * 1e6 / 61000.0; }
namespace Motate {
SysTick_ms SysTickTimer;
uint32_t SysTick_ms::getValue() { return (uint32_t)(now_us() / 1000.0); }
}
static void update_systick() {
    const double frac = fmod(now_us(), 1000.0);
    sim_systick.VAL   = 83999 - (uint32_t)(frac * 84.0);
}

// ---- interrupt bookkeeping ----
static bool exec_pending = false, usart_pending = false, usart_enabled = false;
void NVIC_SetPriority(IRQn_Type, uint32_t) {}
void NVIC_EnableIRQ(IRQn_Type irq) { if (irq == USART2_IRQn) usart_enabled = true; }
void NVIC_DisableIRQ(IRQn_Type irq) { if (irq == USART2_IRQn) usart_enabled = false; }
void NVIC_ClearPendingIRQ(IRQn_Type irq) { if (irq == USART2_IRQn) usart_pending = false; }
void NVIC_SetPendingIRQ(IRQn_Type irq) { if (irq == USART2_IRQn) usart_pending = true; }
void st_request_exec_move() { exec_pending = true; }
void st_request_forward_plan() {}

// ---- g2core position hooks ----
static float synced_pos[AXES];
static int   full_syncs = 0;
void cm_set_position_by_axis(const uint8_t axis, const float position) { synced_pos[axis] = position; full_syncs++; }
void mp_set_runtime_position(uint8_t, const float) {}
static const float steps_per_unit[4] = {200.0f, 200.0f, 400.0f, 1600.0f / 90.0f};
static const uint8_t axis_of[4]      = {AXIS_X, AXIS_Y, AXIS_Z, AXIS_A};
static float kn_offset[4] = {};             // the kinematics' step offset, which g2core re-bases on position resets
void kn_forward_kinematics(const float steps[], float travel[]) {
    for (int a = 0; a < AXES; a++) travel[a] = 0;
    for (int m = 0; m < 4; m++) travel[axis_of[m]] = (steps[m] - kn_offset[m]) / steps_per_unit[m];
}
static bool sim_estop = false;
bool gpio_read_input(const uint8_t input_num) { return (input_num == ESTOP_SWITCH_INPUT) && sim_estop; }

// ---- the real lean prep, wrapped to record what each segment intends ----
#define st_prep_velocity_segment st_prep_velocity_segment_real
#include "prep_extract.inc"
#undef st_prep_velocity_segment
static double pending_travel[MOTORS], running_travel[MOTORS], completed_travel[MOTORS];
static std::function<void(const float *, const float *)> on_prep;
stat_t st_prep_velocity_segment(const float v0[], const float v1[], const uint16_t ticks) {
    for (int m = 0; m < MOTORS; m++) pending_travel[m] = 0.5 * ((double)v0[m] + v1[m]) * ticks / 61000.0;
    if (on_prep) on_prep(v0, v1);
    return st_prep_velocity_segment_real(v0, v1, ticks);
}

// ---- step generator model (stepper.cpp: dda ISR, _load_move, exec ISR) ----
struct { int32_t downcount; int64_t acc[MOTORS]; int64_t inc[MOTORS]; int64_t inc_inc[MOTORS]; uint8_t prev_dir[MOTORS]; } run = {};
static double max_phase_err[MOTORS];
bool st_runtime_isbusy() { return run.downcount != 0; }
static int64_t actual_steps(int m) { return (int64_t)en.en[m].encoder_steps + en.en[m].steps_run; }

static void load_move() {
    if (run.downcount != 0) return;
    // the segment that just ended is complete: compare what it intended with what the DDA made
    for (int m = 0; m < MOTORS; m++) {
        completed_travel[m] += running_travel[m];
        running_travel[m] = 0;
        const double err = fabs((double)actual_steps(m) - completed_travel[m]);
        if (err > max_phase_err[m]) max_phase_err[m] = err;
    }
    if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) return;     // ran dry (motionStopped)
    if (st_pre.block_type == BLOCK_TYPE_ALINE) {
        for (int m = 0; m < MOTORS; m++) {
            if ((run.inc[m] = st_pre.mot[m].substep_increment) != 0) {
                run.inc_inc[m] = st_pre.mot[m].substep_increment_increment;
                if (st_pre.mot[m].direction != run.prev_dir[m]) {
                    run.prev_dir[m] = st_pre.mot[m].direction;
                    run.acc[m]      = -(DDA_SUBSTEPS + run.acc[m]);
                }
                en.en[m].step_sign = st_pre.mot[m].step_sign;
            } else {
                run.inc_inc[m] = 0;
            }
            en.en[m].encoder_steps += en.en[m].steps_run;
            en.en[m].steps_run = 0;
            running_travel[m]  = pending_travel[m];
        }
        run.downcount = st_pre.dda_ticks;
    }
    st_pre.block_type   = BLOCK_TYPE_NULL;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;
    exec_pending        = true;
}

static void dda_tick() {
    if (run.downcount == 0) return;
    for (int m = 0; m < MOTORS; m++) {
        if ((run.acc[m] += run.inc[m]) > 0) {
            en.en[m].steps_run += en.en[m].step_sign;
            run.acc[m] -= DDA_SUBSTEPS;
        }
        run.inc[m] += run.inc_inc[m];
    }
    if (--run.downcount == 0) load_move();
}

static void exec_isr() {
    exec_pending = false;
    if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_EXEC) {
        const stat_t status = pi_link_owns_machine() ? pi_link_exec_move() : STAT_NOOP;
        if (status != STAT_NOOP) {
            st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;
            if (!st_runtime_isbusy()) load_move();
        }
    }
}

#include "pi_link_sim.cpp"

// ---- host side: frames in through the RX PDC, replies out through the TX PDC ----
static Pdc *pdc() { return reinterpret_cast<Pdc *>(0x400A0100U); }
static uint32_t lost_rx = 0;
static void pdc_rx_byte(uint8_t b) {
    Pdc *p = pdc();
    if (p->PERIPH_RCR == 0) { lost_rx++; return; }
    *(volatile uint8_t *)(uintptr_t)p->PERIPH_RPR = b;
    const uint32_t rcr = p->PERIPH_RCR - 1;
    p->PERIPH_RCR = rcr;
    p->PERIPH_RPR = p->PERIPH_RPR + 1;
    if ((rcr == 0) && (p->PERIPH_RNCR != 0)) {
        p->PERIPH_RPR = p->PERIPH_RNPR; p->PERIPH_RCR = p->PERIPH_RNCR; p->PERIPH_RNCR = 0; p->PERIPH_RNPR = 0;
    }
}
static std::vector<uint8_t> cobs(const std::vector<uint8_t> &in) {
    std::vector<uint8_t> out{0};
    size_t cp = 0; uint8_t code = 1;
    for (uint8_t b : in) {
        if (b == 0) { out[cp] = code; cp = out.size(); out.push_back(0); code = 1; }
        else { out.push_back(b); if (++code == 0xFF) { out[cp] = code; cp = out.size(); out.push_back(0); code = 1; } }
    }
    out[cp] = code;
    return out;
}
static bool uncobs(const std::vector<uint8_t> &in, std::vector<uint8_t> &out) {
    out.clear(); size_t i = 0;
    while (i < in.size()) {
        uint8_t code = in[i++];
        if (code == 0 || i + code - 1 > in.size()) return false;
        out.insert(out.end(), in.begin() + i, in.begin() + i + code - 1);
        i += code - 1;
        if (code < 0xFF && i < in.size()) out.push_back(0);
    }
    return true;
}
static uint16_t host_seq = 0;
static void host_send(uint8_t type, const std::vector<uint8_t> &payload) {
    std::vector<uint8_t> f{0, type, (uint8_t)(++host_seq & 0xFF), (uint8_t)(host_seq >> 8)};
    f.insert(f.end(), payload.begin(), payload.end());
    uint32_t crc = crc32(0, f.data(), f.size());
    for (int k = 0; k < 4; k++) f.push_back(crc >> (8 * k));
    auto enc = cobs(f);
    enc.push_back(0);
    for (uint8_t b : enc) pdc_rx_byte(b);
    usart_pending = true;           // receiver time-out after the frame
}
template <typename T> static void put(std::vector<uint8_t> &v, T x) { uint8_t b[sizeof(T)]; memcpy(b, &x, sizeof(T)); v.insert(v.end(), b, b + sizeof(T)); }
static void host_enable(float decel_scale = 1.0f) {
    std::vector<uint8_t> p;
    put<uint16_t>(p, 5); put<uint16_t>(p, 500); put<uint16_t>(p, 250); put<uint16_t>(p, 0);
    const float vmax[4] = {17000, 17000, 13400, 600}, decel[4] = {200000, 200000, 400000, 17780};
    for (float v : vmax) put<float>(p, v);
    for (float d : decel) put<float>(p, d * decel_scale);
    p.resize(52, 0);
    host_send(0x10, p);
}
static void host_motion(const float v[4]) {
    std::vector<uint8_t> p;
    for (int m = 0; m < 4; m++) put<float>(p, v[m]);
    put<uint8_t>(p, 0); put<uint8_t>(p, 0); put<uint16_t>(p, 0); put<float>(p, 0.0f);
    host_send(0x11, p);
}
struct Feedback { uint8_t result, owner, reason, flags; };
static Feedback last_fb{};
static int bad_tx_frames = 0, feedbacks = 0;
static std::vector<uint8_t> tx_acc;
static void host_drain_tx() {
    Pdc *p = pdc();
    for (int pass = 0; pass < 2; pass++) {
        if (p->PERIPH_TCR == 0) break;
        const uint8_t *tx = (const uint8_t *)(uintptr_t)p->PERIPH_TPR;
        for (uint32_t i = 0; i < p->PERIPH_TCR; i++) {
            if (tx[i] != 0) { tx_acc.push_back(tx[i]); continue; }
            std::vector<uint8_t> fr;
            bool ok = uncobs(tx_acc, fr) && fr.size() >= 8;
            tx_acc.clear();
            if (ok) {
                uint32_t crc; memcpy(&crc, &fr[fr.size() - 4], 4);
                ok = crc == crc32(0, fr.data(), fr.size() - 4);
            }
            if (!ok) { bad_tx_frames++; continue; }
            if (fr[1] == 0x91) { last_fb = {fr[4], fr[5], fr[6], fr[7]}; feedbacks++; }
        }
        p->PERIPH_TPR += p->PERIPH_TCR;
        p->PERIPH_TCR = 0;
        if (p->PERIPH_TNCR != 0) { p->PERIPH_TPR = p->PERIPH_TNPR; p->PERIPH_TCR = p->PERIPH_TNCR; p->PERIPH_TNCR = 0; }
    }
}

// ---- run the machine for a while; profile(t_s, v) fills velocities or returns false to stay silent ----
static bool main_stalled = false;           // the main loop is busy elsewhere (a long USB reply)
static void run_for(double seconds, std::function<bool(double, float *)> profile) {
    const uint64_t end = sim_tick + (uint64_t)(seconds * 61000.0);
    double next_frame_us = now_us();
    uint64_t next_main = sim_tick;
    while (sim_tick < end) {
        sim_tick++;
        update_systick();
        dda_tick();
        while (exec_pending) exec_isr();
        if (now_us() >= next_frame_us) {
            next_frame_us += 1000.0;
            float v[4];
            if (profile(now_us() / 1e6, v)) host_motion(v);
        }
        if (usart_pending && usart_enabled) { usart_pending = false; USART2_Handler(); }
        if (!main_stalled && (sim_tick >= next_main)) { next_main = sim_tick + 6; pi_link_periodic(); }   // main loop every ~100 us
        host_drain_tx();
    }
}

static int failures = 0;
static void check(bool ok, const char *what) { printf("  %s %s\n", ok ? "ok  " : "FAIL", what); if (!ok) failures++; }

int main() {
    void *m = mmap((void *)0x400A0000UL, 4096, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (m != (void *)0x400A0000UL) { perror("mmap"); return 2; }
    for (int i = 0; i < 16; i++) in_r[i] = &readers[i];
    for (int k = 0; k < 4; k++) st_cfg.mot[k].steps_per_unit = steps_per_unit[k];
    cm1.machine_state = MACHINE_READY;
    pi_link_init();
    run_for(0.01, [](double, float *) { return false; });

    // 1. Accuracy: smooth random profiles with reversals, all four motors, 8 s
    puts("1. step accuracy under smooth random profiles with reversals (8 s)");
    host_enable();
    std::mt19937 rng(7);
    double amp[4], freq[4], ph[4];
    const float vmax[4] = {17000, 17000, 13400, 600};
    for (int k = 0; k < 4; k++) { amp[k] = vmax[k] * (0.5 + 0.4 * (rng() % 100) / 100.0); freq[k] = 0.3 + 0.2 * k; ph[k] = k; }
    run_for(8.0, [&](double t, float *v) {
        for (int k = 0; k < 4; k++) v[k] = (float)(amp[k] * sin(2 * M_PI * freq[k] * t + ph[k]));
        return true;
    });
    for (int k = 0; k < 4; k++) printf("  motor %d: %lld steps made, worst |made - intended| at a segment boundary %.2f steps\n",
                                         k + 1, (long long)actual_steps(k), max_phase_err[k]);
    check(max_phase_err[0] < 1.5 && max_phase_err[1] < 1.5 && max_phase_err[2] < 1.5 && max_phase_err[3] < 1.5,
          "the DDA makes what each segment intends, within 1.5 steps, through reversals");
    check(diag.underruns == 0 && diag.stale_segments == 0, "no underruns, no stale segments");
    check(bad_tx_frames == 0 && last_fb.owner == 1, "replies intact, Pi still in control");

    // 2. Step change from rest (zero-start floor) and an instant reversal at speed
    puts("2. start from rest and instant reversal at 12000 steps/s");
    for (int k = 0; k < 4; k++) max_phase_err[k] = 0;
    run_for(0.2, [](double, float *v) { for (int k = 0; k < 4; k++) v[k] = 0; return true; });
    const int64_t x0 = actual_steps(0);
    run_for(0.5, [](double, float *v) { v[0] = 12000; v[1] = v[2] = v[3] = 0; return true; });
    const int64_t x1 = actual_steps(0);
    run_for(0.5, [](double, float *v) { v[0] = -12000; v[1] = v[2] = v[3] = 0; return true; });
    run_for(0.2, [](double, float *v) { for (int k = 0; k < 4; k++) v[k] = 0; return true; });
    const int64_t x2 = actual_steps(0);
    printf("  X: +%lld steps forward, then back to %+lld from the start; worst phase error %.2f\n",
           (long long)(x1 - x0), (long long)(x2 - x0), max_phase_err[0]);
    check(llabs((x1 - x0) - 6000) < 40, "0.5 s at 12000 steps/s from rest is ~6000 steps (pipeline delay aside)");
    check(llabs(x2 - x0) <= 2, "forward then back by the same command returns to the start");
    check(max_phase_err[0] < 1.5, "no steps lost at the reversal");

    // 3. Watchdog stop at speed: straight line, linear ramp, release, position sync
    puts("3. watchdog stop from speed (host goes silent)");
    const float cruise[4] = {16000, -8000, 4000, 500};
    run_for(0.5, [&](double, float *v) { for (int k = 0; k < 4; k++) v[k] = cruise[k]; return true; });
    std::vector<std::array<float, 4>> stop_rates;
    on_prep = [&](const float *, const float *to) { if (owner == OWNER_STOPPING) stop_rates.push_back({to[0], to[1], to[2], to[3]}); };
    const int64_t before[4] = {actual_steps(0), actual_steps(1), actual_steps(2), actual_steps(3)};
    full_syncs = 0;
    run_for(0.3, [](double, float *) { return false; });
    on_prep = nullptr;
    { const float z[4] = {0, 0, 0, 0}; host_motion(z); }   // ask once more: the reply carries owner and reason
    run_for(0.01, [](double, float *) { return false; });
    double worst_ratio = 0, ramp_s = stop_rates.size() * 31 / 61000.0;
    for (auto &r : stop_rates) if (fabsf(r[0]) > 100) for (int k = 1; k < 4; k++)
        worst_ratio = fmax(worst_ratio, fabs(r[k] / r[0] - cruise[k] / cruise[0]));
    printf("  stop took %zu segments (%.1f ms; X at 16000 steps/s and 200000 steps/s^2 needs 80 ms)\n", stop_rates.size(), ramp_s * 1e3);
    printf("  coasting distance X %lld steps (v^2/2a + 5 ms watchdog ~ 640 + 80), worst ratio drift %.4f\n",
           (long long)(actual_steps(0) - before[0]), worst_ratio);
    check(last_fb.owner == 0 && last_fb.reason == 2, "control back with USB, reason watchdog");
    check(worst_ratio < 0.01, "all motors ramp down together: the machine stops on a straight line");
    check(fabs(ramp_s - 0.080) < 0.003, "ramp length matches the slowest motor's stop_decel");
    check(full_syncs == 4 && fabsf(synced_pos[AXIS_X] - actual_steps(0) / 200.0f) < 1e-3f, "g2core positions reset from the step counts on release");
    check(diag.underruns == 0, "no underruns during the stop");

    // 4. DISABLE at speed, with a reversal in progress
    puts("4. DISABLE while decelerating through a reversal");
    host_enable();
    run_for(0.3, [](double t, float *v) { v[0] = (float)(12000 * cos(2 * M_PI * 2 * t)); v[1] = 6000; v[2] = -3000; v[3] = 0; return true; });
    host_send(0x12, {});
    run_for(0.3, [](double, float *v) { v[0] = v[1] = v[2] = v[3] = 0; return true; });
    check(last_fb.owner == 0 && last_fb.reason == 1, "control back with USB, reason DISABLE");

    // 5. Clamp
    puts("5. velocity above max_vel is clamped and flagged");
    host_enable();
    const int64_t c0 = actual_steps(3);
    run_for(1.0, [](double, float *v) { v[0] = v[1] = v[2] = 0; v[3] = 5000; return true; });
    printf("  A made %lld steps in 1 s at a requested 5000 steps/s (max_vel 600)\n", (long long)(actual_steps(3) - c0));
    check(llabs((actual_steps(3) - c0) - 600) < 10, "motor 4 held to max_vel");
    host_send(0x12, {});
    run_for(0.2, [](double, float *v) { v[0] = v[1] = v[2] = v[3] = 0; return true; });

    // 6. g2core re-bases the kinematics' step offset mid-session (a shutdown's queue flush does,
    //    from a runtime position up to 10 ms old): the released position must not move with it
    puts("6. step offset re-based 3 steps off while the Pi has control");
    auto zero = [](double, float *v) { v[0] = v[1] = v[2] = v[3] = 0; return true; };
    auto slow = [](double, float *v) { v[0] = -400; v[1] = v[2] = v[3] = 0; return true; };
    kn_offset[0] = 4;                       // what g2core had when the Pi took control
    host_enable();
    run_for(0.3, slow);
    kn_offset[0] = 7;
    run_for(0.1, slow);
    host_send(0x12, {});
    run_for(0.2, zero);
    const float x_expect = (actual_steps(0) - 4) / 200.0f;
    printf("  released X %.4f, ENABLE position plus steps made %.4f\n", synced_pos[AXIS_X], x_expect);
    check(fabsf(synced_pos[AXIS_X] - x_expect) < 1e-3f, "released position is the ENABLE position plus the steps made since");
    kn_offset[0] = 0;

    // 7. E-stop input while the main loop is stalled: the exec interrupt stops the Pi by itself
    puts("7. E-stop input with the main loop stalled");
    auto cruise_x = [](double, float *v) { v[0] = 8000; v[1] = v[2] = v[3] = 0; return true; };
    host_enable();
    run_for(0.2, cruise_x);
    main_stalled = true;
    sim_estop    = true;                    // 300 us: at most one segment's check sees it
    run_for(0.0003, cruise_x);
    sim_estop = false;
    run_for(0.01, cruise_x);
    check(owner == OWNER_PI, "a 300 us glitch on the E-stop input doesn't stop the Pi");
    int segs = 0, stop_seg = -1;
    on_prep = [&](const float *, const float *) { segs++; if ((stop_seg < 0) && (owner == OWNER_STOPPING)) stop_seg = segs; };
    sim_estop = true;
    run_for(0.1, cruise_x);
    on_prep = nullptr;
    printf("  board stop began at the %d%s segment after the input went active\n", stop_seg, stop_seg == 2 ? "nd" : "th");
    check((stop_seg > 0) && (stop_seg <= ESTOP_SEGMENTS), "a held E-stop starts the board stop within 2 segments, main loop or not");
    main_stalled = false;
    sim_estop    = false;
    run_for(0.05, zero);
    { const float z[4] = {0, 0, 0, 0}; host_motion(z); }
    run_for(0.01, zero);
    check(last_fb.owner == 0 && last_fb.reason == 3, "control back with USB once the main loop runs, reason E-stop");

    printf("\nrx bytes lost %u, bad reply frames %d, feedbacks %d, exec max %u us (host clock, not meaningful)\n",
           lost_rx, bad_tx_frames, feedbacks, diag.exec_max_us);
    puts(failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
