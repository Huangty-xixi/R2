/**
 * @file retry.c
 * @brief R2 retry ¡ª CH5 low->mid pulse count selects retry type, CH7 max confirms.
 *
 *   CH7 min ¡ú CH5 low->mid N times ¡ú 2s window ¡ú buzzer N%5 beeps/s ¡ú CH7 max execute
 *   N%5=1: zone1 (reset), 2: zone2 (no reset), 3: zone3_prep (reset), 4: zone3 (no reset), 0: cancel
 */
#include "retry.h"
#include "buzzer.h"
#include "app_zone1.h"
#include "app_zone2.h"
#include "app_zone3.h"
#include "app_zone3_prep.h"
#include "Motion_Task.h"
#include "Process_Flow.h"
#include "odom_nav_goto.h"
#include "clamp_head_ctrl.h"
#include "kfs.h"
#include "weapon.h"
#include "r1_link.h"

#include "cmsis_os.h"

#include <string.h>

/* ---- state machine ---- */
typedef enum {
    RETRY_ST_IDLE = 0,
    RETRY_ST_COUNTING,
    RETRY_ST_FEEDBACK,
    RETRY_ST_EXECUTING,
} RetryState;

/* ---- timing ¡ª Keil Watch live tuning via g_retry_tune ---- */
volatile RetryTune g_retry_tune = {
    .count_window_ms     = 1000U,
    .feedback_timeout_ms = 5000U,
    .beep_on_ms          = 150U,
    .beep_gap_ms         = 50U,
    .beep_cycle_ms       = 1500U,
};

static RetryState s_state = RETRY_ST_IDLE;
static uint32_t s_enter_ms = 0U;
static uint32_t s_last_count_ms = 0U;
static uint8_t  s_count = 0U;
static uint8_t  s_ch7_is_min = 0U;
static uint8_t  s_exec_phase = 0U;
static uint32_t s_phase_ms = 0U;

/* ---- buzzer rhythm (cycle-based, 1000ms per cycle) ---- */
static uint32_t s_cycle_ms = 0U;
static uint32_t s_tick = 0U;
static uint8_t  s_phase = 0U;
static uint8_t  s_beep_i = 0U;
static uint8_t  s_long = 0U;

/* ---- saved mission for zone2 retry ---- */
static app_zone2_mission_t s_saved_mission;
static uint8_t s_has_saved_mission = 0U;

static uint8_t mod5(void) { return s_count % 5U; }

static void buzz_stop(void)
{
    Buzzer_Off();
    s_phase = 0U; s_beep_i = 0U; s_long = 0U;
}

static void buzz_start(uint32_t now)
{
    buzz_stop();
    s_cycle_ms = now; s_tick = now;
    if (mod5() == 0U) { s_long = 1U; Buzzer_Beep(g_retry_tune.beep_cycle_ms); }
    else             { s_phase = 0U; s_beep_i = 0U; }
}

static void buzz_tick(uint32_t now)
{
    uint8_t m = mod5();

    if (s_long) {
        if (now - s_tick >= g_retry_tune.beep_cycle_ms) { s_tick = now; Buzzer_Beep(g_retry_tune.beep_cycle_ms); }
        return;
    }
    if (m == 0U) return;

    switch (s_phase) {
    case 0U:
        Buzzer_Beep(g_retry_tune.beep_on_ms); s_phase = 1U; s_tick = now; break;
    case 1U:
        if (now - s_tick >= g_retry_tune.beep_on_ms) {
            s_beep_i++;
            s_phase = (s_beep_i >= m) ? 3U : 2U;
            s_tick = now;
        }
        break;
    case 2U:
        if (now - s_tick >= g_retry_tune.beep_gap_ms) s_phase = 0U;
        break;
    case 3U:
        if (now - s_cycle_ms >= g_retry_tune.beep_cycle_ms) { s_cycle_ms = now; s_beep_i = 0U; s_phase = 0U; }
        break;
    default: s_phase = 3U; break;
    }
}

/* ---- init ---- */
void Retry_Init(void)
{
    s_state = RETRY_ST_IDLE;
    s_enter_ms = 0U; s_last_count_ms = 0U; s_count = 0U;
    s_ch7_is_min = 0U; s_exec_phase = 0U; s_phase_ms = 0U;
    buzz_stop();
    s_has_saved_mission = 0U;
    (void)memset(&s_saved_mission, 0, sizeof(s_saved_mission));
}

uint8_t Retry_IsActive(void)
{
    return (uint8_t)(s_state != RETRY_ST_IDLE);
}

/* ---- CH5 low->mid edge ---- */
void Retry_OnCH5Edge(void)
{
    uint32_t now = osKernelGetTickCount();

    if (s_state == RETRY_ST_IDLE) {
        if (s_ch7_is_min == 0U) return;
        s_state = RETRY_ST_COUNTING; s_enter_ms = now; s_count = 1U; s_last_count_ms = now;
        Buzzer_Beep(g_retry_tune.beep_on_ms);
        return;
    }
    if (s_state == RETRY_ST_COUNTING) {
        if (s_ch7_is_min == 0U) { s_state = RETRY_ST_IDLE; s_count = 0U; buzz_stop(); return; }
        s_count++; s_last_count_ms = now; s_enter_ms = now;
        Buzzer_Beep(g_retry_tune.beep_on_ms);
        return;
    }
}

/* ---- CH7 level ---- */
void Retry_OnCH7Level(uint8_t ch7_bit)
{
    s_ch7_is_min = (uint8_t)(ch7_bit == 0U);

    if (ch7_bit == 0U && s_state == RETRY_ST_IDLE) {
        Process_Flow_ResetAll();
        odom_nav_goto_disarm();
    }

    if (s_state == RETRY_ST_FEEDBACK && ch7_bit == 1U) {
        uint8_t m = mod5();
        if (m == 0U || m == 1U) { buzz_stop(); s_state = RETRY_ST_IDLE; s_count = 0U; return; }
        s_state = RETRY_ST_EXECUTING; s_exec_phase = 0U; s_phase_ms = osKernelGetTickCount();
        buzz_start(s_phase_ms);
        return;
    }
    if (ch7_bit == 0U && (s_state == RETRY_ST_COUNTING || s_state == RETRY_ST_FEEDBACK)) {
        buzz_stop(); s_state = RETRY_ST_IDLE; s_count = 0U; return;
    }
}

/* ---- exec helpers ---- */
static void exec_zone2_targets(void)
{
    main_lift_position = main_lift_p4;
    kfs_spin_position = kfs_spin_p2;
    kfs_above_position = kfs_above_cmd_p1;
    kfs_below_position = kfs_below_cmd_p1;
    three_kfs_position = three_kfs_p1;
    sucker1_state = 0U; sucker2_state = 0U; sucker3_state = 0U; sucker4_state = 0U;
}

static void exec_zone2_apply(void)
{
    Process_Flow_ResetAll();
    odom_nav_goto_disarm();
    ClampHeadCtrl_Init();
    if (s_has_saved_mission != 0U) app_zone2_mission_apply(&s_saved_mission);
    app_flow_mode = app_flow_zone2;
}

static void exec_zone3_prep(void)
{
    main_lift_position = main_lift_p2;
    kfs_spin_position = kfs_spin_p2;
    kfs_above_position = kfs_above_cmd_p1;
    kfs_below_position = kfs_below_cmd_p1;
    three_kfs_position = three_kfs_p1;
    sucker1_state = 0U; sucker2_state = 0U; sucker3_state = 0U; sucker4_state = 0U;
    Process_Flow_ResetAll();
    odom_nav_goto_disarm();
    AppZone3Prep_StartFromP1();
    app_flow_mode = app_flow_zone3_prep;
}

static void exec_zone3(void)
{
    Process_Flow_ResetAll();
    odom_nav_goto_disarm();
    flow_mode = flow_none;
    app_flow_mode = app_flow_none;
    AppZone3_Start();
    app_flow_mode = app_flow_zone3;
}

/* ---- main tick ---- */
void Retry_Service(uint32_t now_ms)
{
    if (s_state == RETRY_ST_COUNTING && (now_ms - s_enter_ms) >= g_retry_tune.count_window_ms) {
        s_state = RETRY_ST_FEEDBACK; s_enter_ms = now_ms; buzz_start(now_ms);
    }
    if (s_state == RETRY_ST_FEEDBACK) {
        if ((now_ms - s_enter_ms) >= g_retry_tune.feedback_timeout_ms) { buzz_stop(); s_state = RETRY_ST_IDLE; s_count = 0U; }
        else buzz_tick(now_ms);
    }
    if (s_state == RETRY_ST_EXECUTING) buzz_tick(now_ms);
    Buzzer_Service(now_ms);

    if (s_state != RETRY_ST_EXECUTING) return;
    uint8_t m = mod5();

    switch (s_exec_phase) {
    case 0U:
        if (m == 2U)          { exec_zone2_targets(); s_exec_phase = 1U; s_phase_ms = now_ms; }
        else if (m == 3U)     { exec_zone3_prep(); buzz_stop(); s_state = RETRY_ST_IDLE; }
        else                  { exec_zone3(); buzz_stop(); s_state = RETRY_ST_IDLE; }
        break;
    case 1U:
        if ((now_ms - s_phase_ms) >= 1000U) {
            if (m == 2U) exec_zone2_apply();
            buzz_stop(); s_state = RETRY_ST_IDLE;
        }
        break;
    default: break;
    }
}

/* ---- save mission ---- */
void Retry_SaveMission(const void *data, uint8_t size)
{
    if (data == NULL || size > sizeof(s_saved_mission)) return;
    (void)memcpy(&s_saved_mission, data, size);
    s_has_saved_mission = (size > 0U) ? 1U : 0U;
}
