// Minimal stand-ins for g2core / Motate / CMSIS so pi_link.cpp compiles on the host.
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdlib>

typedef uint8_t stat_t;
#define STAT_OK 0
#define STAT_NOOP 1
#define STAT_INTERNAL_ERROR 2
#define MOTORS 4
#define AXES 9
#define FREQUENCY_DDA 61000UL
#define DDA_SUBSTEPS (INT64_MAX-100)
enum { AXIS_X = 0, AXIS_Y, AXIS_Z, AXIS_U, AXIS_V, AXIS_W, AXIS_A, AXIS_B, AXIS_C };
enum { DIRECTION_CW = 0, DIRECTION_CCW = 1 };

// canonical machine
typedef enum { MACHINE_INITIALIZING = 0, MACHINE_READY, MACHINE_ALARM, MACHINE_PROGRAM_STOP, MACHINE_PROGRAM_END,
               MACHINE_CYCLE, MACHINE_INTERLOCK, MACHINE_SHUTDOWN, MACHINE_PANIC } cmMachineState;
enum { CYCLE_NONE = 0 };
enum { FEEDHOLD_OFF = 0 };
enum { COMBINED_READY = 1, COMBINED_RUN = 5 };
struct GCodeStateX { float position[AXES]; };
struct cmMachine_t { cmMachineState machine_state; int cycle_type; int hold_state; bool homed[AXES]; GCodeStateX gmx; };
extern cmMachine_t cm1;
extern cmMachine_t *cm;
#define ACTIVE_MODEL 0
inline int cm_get_combined_state(cmMachine_t *) { return 1; }
inline float cm_get_absolute_position(int, uint8_t axis) { return cm->gmx.position[axis]; }
void cm_set_position_by_axis(const uint8_t axis, const float position);
inline void cm_clear() {}
inline stat_t cm_panic(stat_t s, const char *) { std::abort(); return s; }

// planner
struct mpPlanner_t { int dummy; };
extern mpPlanner_t *mp;
inline bool mp_has_runnable_buffer(const mpPlanner_t *) { return false; }
inline bool mp_runtime_is_idle() { return true; }
void mp_set_runtime_position(uint8_t axis, const float position);

// stepper
struct Stepper { void enable() {} };
extern Stepper *Motors[MOTORS];
stat_t st_prep_velocity_segment(const float v0[], const float v1[], const uint16_t ticks);
void st_request_exec_move();
bool st_runtime_isbusy();
void st_request_forward_plan();

// encoder
struct enEncoder_t { int16_t steps_run; int32_t encoder_steps; int8_t step_sign; };
struct enEncoders_t { enEncoder_t en[MOTORS]; };
extern enEncoders_t en;

// kinematics: Desktop CNC steps per unit, motors 1-4 -> X Y Z A
void kn_forward_kinematics(const float steps[], float travel[]);

// spindle, gpio, report
enum spDirection { SPINDLE_OFF = 0 };
inline spDirection spindle_get_direction() { return SPINDLE_OFF; }
inline float spindle_get_speed() { return 0; }
struct Reader { bool getState() { return false; } };
extern Reader *in_r[16];
enum { SR_REQUEST_IMMEDIATE = 0 };
inline stat_t sr_request_status_report(int) { return STAT_OK; }

// Motate / CMSIS
typedef struct { volatile uint32_t US_CR, US_MR, US_IER, US_IDR, US_IMR, US_CSR, US_RHR, US_THR, US_BRGR, US_RTOR; } Usart;
typedef struct { volatile uint32_t PERIPH_RPR, PERIPH_RCR, PERIPH_TPR, PERIPH_TCR, PERIPH_RNPR, PERIPH_RNCR, PERIPH_TNPR, PERIPH_TNCR, PERIPH_PTCR, PERIPH_PTSR; } Pdc;
#define US_CR_RSTRX (1u<<2)
#define US_CR_RSTTX (1u<<3)
#define US_CR_RXEN (1u<<4)
#define US_CR_RXDIS (1u<<5)
#define US_CR_TXEN (1u<<6)
#define US_CR_TXDIS (1u<<7)
#define US_CR_RSTSTA (1u<<8)
#define US_CR_STTTO (1u<<11)
#define US_MR_USART_MODE_NORMAL 0u
#define US_MR_USCLKS_MCK 0u
#define US_MR_CHRL_8_BIT (3u<<6)
#define US_MR_PAR_NO (4u<<9)
#define US_MR_NBSTOP_1_BIT 0u
#define US_MR_CHMODE_NORMAL 0u
#define US_BRGR_CD(v) ((uint32_t)(v) & 0xFFFF)
#define US_BRGR_FP(v) (((uint32_t)(v) & 7) << 16)
#define US_CSR_OVRE (1u<<5)
#define US_CSR_FRAME (1u<<6)
#define US_CSR_TIMEOUT (1u<<8)
#define US_IER_TIMEOUT (1u<<8)
#define US_IDR_TIMEOUT (1u<<8)
#define US_RTOR_TO(v) ((uint32_t)(v))
#define PERIPH_PTCR_RXTEN (1u<<0)
#define PERIPH_PTCR_RXTDIS (1u<<1)
#define PERIPH_PTCR_TXTEN (1u<<8)
#define PERIPH_PTCR_TXTDIS (1u<<9)
enum IRQn_Type { USART2_IRQn = 19, UOTGHS_IRQn = 40 };
void NVIC_SetPriority(IRQn_Type, uint32_t);
void NVIC_EnableIRQ(IRQn_Type);
void NVIC_DisableIRQ(IRQn_Type);
void NVIC_ClearPendingIRQ(IRQn_Type);
void NVIC_SetPendingIRQ(IRQn_Type);
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
struct SysTick_t { uint32_t VAL; uint32_t LOAD; };
extern SysTick_t sim_systick;
#define SysTick (&sim_systick)
struct SCB_t { uint32_t ICSR; };
extern SCB_t sim_scb;
#define SCB (&sim_scb)
#define SCB_ICSR_PENDSTSET_Msk (1u<<26)
namespace Motate {
enum { kSocket1_SPISlaveSelectPinNumber = 10, kSocket2_SPISlaveSelectPinNumber = 20 };
enum { kPullUp = 1 };
template <int N> struct UARTRxPin { static const uint8_t uartNum = 2; void setOptions(int) {} };
template <int N> struct UARTTxPin { static const uint8_t uartNum = 2; };
struct SamCommon { static void enablePeripheralClock(uint32_t) {} static uint32_t getPeripheralClockFreq() { return 84000000; } };
struct SysTick_ms { uint32_t getValue(); };
extern SysTick_ms SysTickTimer;
}
// st_pre / st_cfg for the extracted st_prep_velocity_segment()
enum { PREP_BUFFER_OWNED_BY_LOADER = 0, PREP_BUFFER_OWNED_BY_EXEC = 1 };
enum { BLOCK_TYPE_NULL = 0, BLOCK_TYPE_ALINE = 1 };
struct stPrepMotor_t { int64_t substep_increment; int64_t substep_increment_increment; uint8_t direction; int8_t step_sign; };
struct stPrep_t { volatile int buffer_state; int block_type; void *bf; int32_t dda_ticks; stPrepMotor_t mot[MOTORS]; };
struct stCfgMotor_t { uint8_t polarity; };
struct stConfig_t { stCfgMotor_t mot[MOTORS]; };
extern stPrep_t st_pre;
extern stConfig_t st_cfg;
