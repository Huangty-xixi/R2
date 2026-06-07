#ifndef APP_ZONE1_H
#define APP_ZONE1_H

#include <stdint.h>

#include "clamp_head_ctrl.h"
#include "odom_nav_goto.h"

/* 0=竞技赛(夹1枪)  1=技能赛(夹2枪) */
#ifndef APP_ZONE1_SKILL_MODE
#define APP_ZONE1_SKILL_MODE  (1U)
#endif

/**
 * 一区流程状态（与状态机 case 顺序一致，Keil Watch 看 state 数值）
 * 0 idle  1 nav+90开局  2 右移搜料  3 夹爪等待
 * 4 转180+前进  5 慢进抵限位  6 等R1  7 done  8 abort
 */
typedef enum
{
    app_zone1_state_idle = 0,
    app_zone1_state_nav_turn90_to_open,
    app_zone1_state_shift_right_monitor,
    app_zone1_state_shift_right_clamp_wait,
    app_zone1_state_advance_turn180,
    app_zone1_state_forward_slow_to_limit,
    app_zone1_state_wait_r1_release,
    app_zone1_state_done,
    app_zone1_state_abort,
} AppZone1State;

typedef AppZone1State app_zone1_state_t;

/** shift_right_monitor 内部相位（Watch: grab_phase） */
typedef enum
{
    app_zone1_grab_phase_pull_back = 0,
    app_zone1_grab_phase_sweep,
} AppZone1GrabPhase;

typedef struct
{
    /* 开局：odom 到点 + 同步转 90° */
    float open_target_x_m;
    float open_target_y_m;
    uint32_t action_timeout_ms;
    uint32_t nav_odom_max_age_ms;

    /* 夹取 Y 工作区（仅 Y 限制；越界反拉） */
    float grab_work_y_min_m;
    float grab_work_y_max_m;
    float grab_work_y_margin_m;

    /* shift_right_monitor 扫掠 */
    float shift_right_slow_cmd;
    float shift_right_vy_comp_cmd;

    /* shift_right_clamp_wait */
    uint32_t clamp_timeout_ms;
    uint32_t clamp_upright_hold_dwell_ms;

    /* advance_turn180：转 180 同时 vy 前进（无 vw） */
    float post_grab_forward_vy_cmd;

    /* forward_slow_to_limit：rpm 抵限位 */
    float forward_slow_cmd;
    float limit_meas_rpm_thr;
    float limit_cmd_thr;
    uint32_t limit_debounce_ms;
    uint32_t limit_timeout_ms;

    /* wait_r1_release */
    uint32_t r1_wait_timeout_ms;
} AppZone1Config;

/** Keil Watch：一区流程实时快照 */
typedef struct
{
    AppZone1State state;
    uint32_t state_enter_ms;
    uint32_t state_dwell_ms;

    uint8_t active;
    uint8_t done;
    uint8_t failed;

    uint8_t grab_latched;
    uint8_t grab_was_active_in_clamp_wait;
    uint32_t grab_retry_count;

    ClampHeadState clamp_prev_state;

    uint8_t r1_pending;
    uint8_t yaw_cmd_issued;
    uint8_t skill_lap;

    AppZone1GrabPhase grab_phase;
    int8_t grab_sweep_dir;
    float center_y_m;
    uint8_t center_y_valid;
    uint8_t in_grab_work_y;

    uint32_t limit_detect_start_ms;
    float chassis_rpm_abs_avg;

    uint8_t last_apply_ok;
    uint8_t last_apply_axis_mask;
    float last_apply_vy;
    float last_apply_vw;

    uint8_t pf_axis_mask;
    float pf_override_vy;
    float pf_override_vw;

    odom_nav_goto_err_t last_nav_rc;
    float nav_target_x_m;
    float nav_target_y_m;
} app_zone1_dbg_t;

void AppZone1_Init(void);
void AppZone1_Start(void);
void AppZone1_Run(void);
void AppZone1_Reset(void);

void AppZone1_NotifyR1Release(void);

uint8_t AppZone1_IsBusy(void);
uint8_t AppZone1_IsDone(void);
uint8_t AppZone1_IsFailed(void);

uint8_t AppZone1_GetConfig(AppZone1Config *out);
uint8_t AppZone1_SetConfig(const AppZone1Config *cfg);
uint8_t AppZone1_SetPostGrabForwardVy(float vy_cmd);

extern volatile AppZone1Config g_app_zone1_cfg;
extern volatile app_zone1_dbg_t g_app_zone1_dbg;

#endif /* APP_ZONE1_H */
