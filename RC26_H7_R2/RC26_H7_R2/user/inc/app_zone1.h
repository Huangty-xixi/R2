#ifndef APP_ZONE1_H
#define APP_ZONE1_H


#include <stdint.h>

#include "app_init.h"
#include "clamp_head_ctrl.h"
#include "odom_nav_goto.h"

/* 0=竞技赛(夹1枪)  1=技能赛(夹2枪) */
#ifndef APP_ZONE1_SKILL_MODE
#define APP_ZONE1_SKILL_MODE  (0U)
#endif

/**
 * 0=正常比赛逻辑
 * 1=夹取不可完成时（搜料超时/夹爪超时/失料重试耗尽）放行跑通下游（仅台架联调）
 */
#ifndef APP_ZONE1_FLOW_THROUGH_ENABLE
#define APP_ZONE1_FLOW_THROUGH_ENABLE  (1U)
#endif

/** 等 R1 超时后放行；FLOW_THROUGH 开启时默认一并启用 */
#ifndef APP_ZONE1_WAIT_R1_TIMEOUT_ENABLE
#if APP_ZONE1_FLOW_THROUGH_ENABLE
#define APP_ZONE1_WAIT_R1_TIMEOUT_ENABLE  (1U)
#else
#define APP_ZONE1_WAIT_R1_TIMEOUT_ENABLE  (0U)
#endif
#endif

/** 右移搜料 Y 锚点数量：竞技赛 6 点全用；技能赛红 [0..2]、蓝 [3..5] */
#define APP_ZONE1_SWEEP_ANCHOR_COUNT           (6U)
#define APP_ZONE1_SWEEP_ANCHOR_SKILL_PER_SIDE  (3U)

/**
 * 开局开口点导航目标（米，与 odom 车心坐标一致）
 * - 竞技赛红/蓝 + 技能赛红方：APP_ZONE1_OPEN_TARGET_SHARED_*
 * - 技能赛红蓝共用：APP_ZONE1_OPEN_TARGET_SKILL_*
 * Keil -D 可覆盖各默认值
 */
#ifndef APP_ZONE1_OPEN_TARGET_SHARED_X_M
#define APP_ZONE1_OPEN_TARGET_SHARED_X_M       (0.61f)   /* 竞技红蓝、技能红；0.58-0.08=0.50 */
#endif
#ifndef APP_ZONE1_OPEN_TARGET_SHARED_Y_M
#define APP_ZONE1_OPEN_TARGET_SHARED_Y_M       (0.39f)//0.55-0.13=0.42
#endif
#ifndef APP_ZONE1_OPEN_TARGET_SKILL_X_M
#define APP_ZONE1_OPEN_TARGET_SKILL_X_M   (0.61f)   /* 技能赛红蓝共用；0.58-0.08=0.50 */
#endif
#ifndef APP_ZONE1_OPEN_TARGET_SKILL_Y_M
#define APP_ZONE1_OPEN_TARGET_SKILL_Y_M   (0.96f)//1.09-0.13=0.96
#endif

#if APP_ZONE1_SKILL_MODE
#define APP_ZONE1_OPEN_TARGET_X_M              APP_ZONE1_OPEN_TARGET_SKILL_X_M
#define APP_ZONE1_OPEN_TARGET_Y_M              APP_ZONE1_OPEN_TARGET_SKILL_Y_M
#else
#define APP_ZONE1_OPEN_TARGET_X_M              APP_ZONE1_OPEN_TARGET_SHARED_X_M
#define APP_ZONE1_OPEN_TARGET_Y_M              APP_ZONE1_OPEN_TARGET_SHARED_Y_M
#endif

/**
 * 一区流程状态（与状态机 case 顺序一致，Keil Watch 看 state 数值）
 * 0 idle  1 边导航开口边转90  2 倒退靠限位  3 右移搜料  4 夹爪等待
 * 5 转180+前进  6 等R1(底盘锁死，放行后解锁)  7 done  8 abort
 */
typedef enum
{
    app_zone1_state_idle = 0,
    app_zone1_state_nav_turn_open,
    app_zone1_state_reverse_slow_to_limit,
    app_zone1_state_shift_right_monitor,
    app_zone1_state_shift_right_clamp_wait,
    app_zone1_state_advance_turn180,
    app_zone1_state_wait_r1_release,
    app_zone1_state_done,
    app_zone1_state_abort,
    app_zone1_state_nav_turn_lap2,  /* 第二圈:转180+导航到lap2点 */
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
    float sweep_anchor_y_m[APP_ZONE1_SWEEP_ANCHOR_COUNT];
    float sweep_anchor_slow_radius_m;
    uint8_t grab_ticks_thr_boundary;
    uint8_t grab_ticks_thr_center;

    /* shift_right_clamp_wait */
    uint32_t clamp_timeout_ms;
    uint32_t clamp_upright_hold_dwell_ms;


    /* 抓取后返回导航目标（米），与旋转180°并行 */
    float return_target_x_m;
    float return_target_y_m;

    /* reverse_slow_to_limit / 调试：抵限位（单轮 |rpm|<=limit_meas_rpm_thr，>=limit_stall_wheel_min 判堵转） */
    float forward_slow_cmd;
    float limit_meas_rpm_thr;
    uint8_t limit_stall_wheel_min;
    float limit_cmd_thr;
    uint32_t limit_debounce_ms;
    uint32_t limit_timeout_ms;

    /* wait_r1_release */
    uint32_t r1_wait_timeout_ms;

    /* 第二圈起始目标(米):转180+导航并行 */
    float lap2_x_m;
    float lap2_y_m;
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
    uint32_t grab_retry_count;

    ClampHeadState clamp_prev_state;

    uint8_t r1_pending;
    uint8_t yaw_cmd_issued;
    uint8_t skill_lap;

    AppZone1GrabPhase grab_phase;
    int8_t grab_sweep_dir;
    uint8_t grab_y_zone;
    float center_y_m;
    uint8_t center_y_valid;
    uint8_t in_grab_work_y;
    float grab_sweep_vw_scale;

    uint32_t limit_detect_start_ms;
    float chassis_rpm_abs_avg;
    uint8_t limit_stall_wheel_count;

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

/** 清除任务与机内状态；急停/遥控/CH7 离开高位时调用，需再次满足全自动+CH7 高才 Start */
void app_zone1_mission_clear(void);

/** 一区状态机周期推进（对标 app_zone2_poll；Motion 在 app_flow_zone1 时调用） */
void app_zone1_poll(void);

uint8_t app_zone1_is_done(void);
uint8_t app_zone1_is_failed(void);

void AppZone1_Init(void);
void AppZone1_Start(void);
void AppZone1_Run(void);
void AppZone1_Reset(void);

void AppZone1_NotifyR1Release(void);

uint8_t AppZone1_IsDone(void);
uint8_t AppZone1_IsFailed(void);
uint8_t AppZone1_IsBusy(void);
uint8_t AppZone1_ShouldAllowAutoGrab(void);

/** 状态 6 wait_r1_release：请求底盘锁死（ChassisLockHold）；离开后自动解锁 */
uint8_t AppZone1_ChassisLockHoldActive(void);

uint8_t AppZone1_GetConfig(AppZone1Config *out);
uint8_t AppZone1_SetConfig(const AppZone1Config *cfg);

extern volatile AppZone1Config g_app_zone1_cfg;
extern volatile app_zone1_dbg_t g_app_zone1_dbg;

#endif /* APP_ZONE1_H */
