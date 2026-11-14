/* Just enough of rtapi.h, rtapi_app.h and hal.h to build pilink.c on a PC for driver_test.c.
 * Pins are allocated by name; exported functions are recorded so the test can call them. */
#ifndef HAL_STUB_H
#define HAL_STUB_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef volatile bool     hal_bit_t;
typedef volatile double   hal_float_t;
typedef volatile int32_t  hal_s32_t;
typedef volatile uint32_t hal_u32_t;
enum { HAL_IN = 16, HAL_OUT = 32, HAL_IO = 48, HAL_RO = 64, HAL_RW = 192 };
enum { RTAPI_MSG_NONE = 0, RTAPI_MSG_ERR, RTAPI_MSG_WARN, RTAPI_MSG_INFO, RTAPI_MSG_DBG };

#define MODULE_AUTHOR(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_LICENSE(x)
#define RTAPI_MP_STRING(var, desc)
#define RTAPI_MP_INT(var, desc)

#define STUB_MAX_PINS 128
static struct { char name[64]; void *p; } stub_pins[STUB_MAX_PINS];
static int stub_npins = 0;
static struct { char name[64]; void (*f)(void *, long); void *arg; } stub_functs[4];
static int stub_nfuncts = 0;
static char stub_last_msg[512];
static int  stub_msgs = 0;

static void *stub_pin_new(size_t size, const char *fmt, va_list ap) {
    if (stub_npins >= STUB_MAX_PINS) {
        abort();
    }
    vsnprintf(stub_pins[stub_npins].name, sizeof(stub_pins[0].name), fmt, ap);
    stub_pins[stub_npins].p = calloc(1, size);
    return stub_pins[stub_npins++].p;
}
#define STUB_PIN(type, fn)                                                    \
    static int fn(int dir, type **ptr, int comp, const char *fmt, ...) {     \
        (void)dir; (void)comp;                                                \
        va_list ap;                                                           \
        va_start(ap, fmt);                                                    \
        *ptr = stub_pin_new(sizeof(type), fmt, ap);                           \
        va_end(ap);                                                           \
        return 0;                                                             \
    }
STUB_PIN(hal_bit_t, hal_pin_bit_newf)
STUB_PIN(hal_float_t, hal_pin_float_newf)
STUB_PIN(hal_s32_t, hal_pin_s32_newf)
STUB_PIN(hal_u32_t, hal_pin_u32_newf)
static int hal_param_float_newf(int dir, hal_float_t *p, int comp, const char *fmt, ...) {
    (void)dir; (void)p; (void)comp; (void)fmt;
    return 0;
}
static int hal_param_u32_newf(int dir, hal_u32_t *p, int comp, const char *fmt, ...) {
    (void)dir; (void)p; (void)comp; (void)fmt;
    return 0;
}
static int hal_export_funct(const char *name, void (*f)(void *, long), void *arg, int fp, int re, int comp) {
    (void)fp; (void)re; (void)comp;
    snprintf(stub_functs[stub_nfuncts].name, sizeof(stub_functs[0].name), "%s", name);
    stub_functs[stub_nfuncts].f   = f;
    stub_functs[stub_nfuncts].arg = arg;
    stub_nfuncts++;
    return 0;
}
static int   hal_init(const char *name) { (void)name; return 1; }
static void *hal_malloc(long size) { return calloc(1, size); }
static int   hal_ready(int comp) { (void)comp; return 0; }
static int   hal_exit(int comp) { (void)comp; return 0; }

static long long rtapi_get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void rtapi_print_msg(int level, const char *fmt, ...) {
    (void)level;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(stub_last_msg, sizeof(stub_last_msg), fmt, ap);
    va_end(ap);
    stub_msgs++;
}

#endif
