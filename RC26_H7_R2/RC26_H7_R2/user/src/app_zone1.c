#include "app_zone1.h"
#include "app_init.h"
#include "r1_link.h"
#include "r1_link_sig.h"

#include "Process_Flow.h"
#include "clamp_head_ctrl.h"
#include "weapon.h"
#include "yaw_heading_ctrl.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "odom_nav_goto.h"
#include "odom_center_offset.h"
#include "upper_pc_protocol.h"

#include <math.h>
#include <string.h>

    
#define APP_ZONE1_NAV_ODOM_MAX_AGE_MS_DEFAULT  (500U)   // 500ms
#define APP_ZONE1_GRAB_RETRY_MAX               (8U)      // 8次
#define APP_ZONE1_WAIT_R1_TIMEOUT_ENABLE       (0U)      // 0:不启用

#define APP_ZONE1_CHASSIS_AXES_NAV             ((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY | \
                                                         PROCESS_FLOW_CHASSIS_OVERRIDE_VW))  // VY和VW
#define APP_ZONE1_CHASSIS_PRIO_MOTION          PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH  // 高优先级

#define APP_ZONE1_GRAB_SWEEP_DIR_BLUE          (1)  // 蓝方扫掠方向
#define APP_ZONE1_GRAB_SWEEP_DIR_RED           (-1)  // 红方扫掠方向

typedef enum
{
    app_zone1_nav_turn_none = 0,
    app_zone1_nav_turn_90,
    app_zone1_nav_turn_180,
} app_zone1_nav_turn_t;

volatile AppZone1Config g_app_zone1_cfg = {
    .open_target_x_m = 0.54f,  //  开局目标X
    .open_target_y_m = 0.54f, //  开局目标Y
    .action_timeout_ms = 15000U, // 15s   动作超时时间
    .nav_odom_max_age_ms = APP_ZONE1_NAV_ODOM_MAX_AGE_MS_DEFAULT, // 500ms 导航odom最大年龄
    .grab_work_y_min_m = 0.54f, //    夹取Y工作区最小值
    .grab_work_y_max_m = 1.1f, //  夹取Y工作区最大值
    .grab_work_y_margin_m = 0.02f, // 0.02m   夹取Y工作区边距
    .shift_right_slow_cmd = 40.0f, // 40.0f 扫掠慢速速度        
    .shift_right_vy_comp_cmd = -8.0f, // -8.0f 扫掠补偿速度
    .clamp_timeout_ms = 10000U, // 10s   夹爪超时时间
    .clamp_upright_hold_dwell_ms = 2000U, // 2s   夹爪直立保持时间
    .post_grab_forward_vy_cmd = 10.0f, // 10.0f 后拉前进速度
    .forward_slow_cmd = 15.0f, // 15.0f 慢进速度
    .limit_meas_rpm_thr = 10.0f, // 10.0f 限位测量阈值
    .limit_cmd_thr = 2.0f, // 2.0f 限位命令阈值
    .limit_debounce_ms = 180U, // 180ms 限位消抖时间
    .limit_timeout_ms = 6000U, // 6s   限位超时时间
    .r1_wait_timeout_ms = 20000U, // 20s   等R1超时时间
};

typedef struct
{
    app_zone1_state_t state; // 状态
    uint32_t state_enter_ms; // 状态进入时间
    uint32_t limit_detect_start_ms; // 限位检测开始时间
    uint32_t r1_wait_start_ms; // R1等待开始时间
    volatile uint8_t r1_pending; // R1等待标志
    uint8_t yaw_cmd_issued; // 转向命令已发出标志
    uint8_t grab_latched; // 夹爪锁定标志
    uint32_t grab_retry_count; // 夹爪重试次数
    uint8_t grab_was_active_in_clamp_wait; // 夹爪在夹爪等待阶段是否活动标志
    uint8_t skill_lap; // 技能圈数标志
    uint8_t advance_turn180_cmd_failed; // 转180命令失败标志
    AppZone1GrabPhase grab_phase; // 夹爪扫掠阶段
    int8_t grab_sweep_dir; // 夹爪扫掠方向
    float center_y_m; // 中心Y坐标
    uint8_t center_y_valid; // 中心Y坐标有效标志
    uint8_t active; // 活动标志
    uint8_t done; // 完成标志
    uint8_t failed; // 失败标志
    uint8_t clamp_upright_hold_dwell_started; // 夹爪直立保持开始标志
    uint32_t clamp_upright_hold_dwell_start_ms; // 夹爪直立保持开始时间
    ClampHeadState clamp_prev_state; // 夹爪前状态
    odom_nav_goto_target_t target; // 导航目标
    odom_nav_goto_err_t last_nav_rc; // 导航错误码
} app_zone1_ctx_t;

static app_zone1_ctx_t g_app_zone1_ctx; // 一区上下文

volatile app_zone1_dbg_t g_app_zone1_dbg; // 一区调试信息

static uint8_t app_zone1_cfg_validate(const AppZone1Config *cfg)
{
    if (cfg == 0)
    {
        return 0U;
    }
    if (!isfinite(cfg->open_target_x_m) || !isfinite(cfg->open_target_y_m))
    {
        return 0U;
    }
    if (!isfinite(cfg->grab_work_y_min_m) || !isfinite(cfg->grab_work_y_max_m) ||
        !isfinite(cfg->grab_work_y_margin_m))
    {
        return 0U;
    }
    if (cfg->grab_work_y_max_m <= cfg->grab_work_y_min_m)
    {
        return 0U;
    }
    if (cfg->grab_work_y_margin_m < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->shift_right_slow_cmd) || !isfinite(cfg->shift_right_vy_comp_cmd) ||
        !isfinite(cfg->post_grab_forward_vy_cmd) ||
        !isfinite(cfg->forward_slow_cmd))
    {
        return 0U;
    }
    if (fabsf(cfg->shift_right_vy_comp_cmd) > (0.5f * fabsf(cfg->shift_right_slow_cmd)))
    {
        return 0U;
    }
    if (!isfinite(cfg->limit_meas_rpm_thr) || cfg->limit_meas_rpm_thr < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->limit_cmd_thr) || cfg->limit_cmd_thr < 0.0f)
    {
        return 0U;
    }
    if (cfg->limit_debounce_ms == 0U || cfg->limit_timeout_ms == 0U ||
        cfg->clamp_timeout_ms == 0U ||
        cfg->r1_wait_timeout_ms == 0U || cfg->action_timeout_ms == 0U)
    {
        return 0U;
    }
    return 1U;
}

static void app_zone1_dbg_refresh(uint32_t now_ms, float meas_rpm_abs)
{
    g_app_zone1_dbg.state = (AppZone1State)g_app_zone1_ctx.state;
    g_app_zone1_dbg.state_enter_ms = g_app_zone1_ctx.state_enter_ms;
    if (now_ms >= g_app_zone1_ctx.state_enter_ms)
    {
        g_app_zone1_dbg.state_dwell_ms = now_ms - g_app_zone1_ctx.state_enter_ms;
    }
    else
    {
        g_app_zone1_dbg.state_dwell_ms = 0U;
    }

    g_app_zone1_dbg.active = g_app_zone1_ctx.active;
    g_app_zone1_dbg.done = g_app_zone1_ctx.done;
    g_app_zone1_dbg.failed = g_app_zone1_ctx.failed;

    g_app_zone1_dbg.grab_latched = g_app_zone1_ctx.grab_latched;
    g_app_zone1_dbg.grab_was_active_in_clamp_wait = g_app_zone1_ctx.grab_was_active_in_clamp_wait;
    g_app_zone1_dbg.grab_retry_count = g_app_zone1_ctx.grab_retry_count;

    g_app_zone1_dbg.clamp_prev_state = g_app_zone1_ctx.clamp_prev_state;

    g_app_zone1_dbg.r1_pending = g_app_zone1_ctx.r1_pending;
    g_app_zone1_dbg.yaw_cmd_issued = g_app_zone1_ctx.yaw_cmd_issued;
    g_app_zone1_dbg.skill_lap = g_app_zone1_ctx.skill_lap;

    g_app_zone1_dbg.grab_phase = g_app_zone1_ctx.grab_phase;
    g_app_zone1_dbg.grab_sweep_dir = g_app_zone1_ctx.grab_sweep_dir;
    g_app_zone1_dbg.center_y_m = g_app_zone1_ctx.center_y_m;
    g_app_zone1_dbg.center_y_valid = g_app_zone1_ctx.center_y_valid;
    if (g_app_zone1_ctx.center_y_valid != 0U)
    {
        const float y_lo = g_app_zone1_cfg.grab_work_y_min_m;
        const float y_hi = g_app_zone1_cfg.grab_work_y_max_m;
        g_app_zone1_dbg.in_grab_work_y = (uint8_t)((g_app_zone1_ctx.center_y_m >= y_lo) &&
                                                   (g_app_zone1_ctx.center_y_m <= y_hi));
    }
    else
    {
        g_app_zone1_dbg.in_grab_work_y = 0U;
    }

    g_app_zone1_dbg.limit_detect_start_ms = g_app_zone1_ctx.limit_detect_start_ms;
    g_app_zone1_dbg.chassis_rpm_abs_avg = meas_rpm_abs;

    g_app_zone1_dbg.last_nav_rc = g_app_zone1_ctx.last_nav_rc;
    g_app_zone1_dbg.nav_target_x_m = g_app_zone1_ctx.target.x_m;
    g_app_zone1_dbg.nav_target_y_m = g_app_zone1_ctx.target.y_m;

    g_app_zone1_dbg.pf_axis_mask = (uint8_t)process_flow_chassis_override.axis_mask;
    g_app_zone1_dbg.pf_override_vy = process_flow_chassis_override.vy;
    g_app_zone1_dbg.pf_override_vw = process_flow_chassis_override.vw;
}

static uint8_t app_zone1_flow_state_depends_on_nav_odom(app_zone1_state_t st)
{
    return (uint8_t)((st == app_zone1_state_nav_turn90_to_open) ||
                     (st == app_zone1_state_shift_right_monitor));
}

static uint8_t app_zone1_flow_nav_odom_trustworthy(void)
{
    if (rc_odom_is_valid() == 0U)
    {
        return 0U;
    }
    if (g_app_zone1_cfg.nav_odom_max_age_ms > 0U)
    {
        if (rc_get_odom_age_ms() > g_app_zone1_cfg.nav_odom_max_age_ms)
        {
            return 0U;
        }
    }
    return 1U;
}

static void app_zone1_flow_release_for_nav(void)
{
    Process_Flow_ClearChassisOverrideAxes(APP_ZONE1_CHASSIS_AXES_NAV);
}

static void app_zone1_flow_clear_motion_override(void)
{
    Process_Flow_ClearChassisOverrideAxesByPriority(
        (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY | PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
        APP_ZONE1_CHASSIS_PRIO_MOTION);
}

static void app_zone1_flow_apply_chassis_axes(uint8_t axis_mask, float vx_cmd, float vy_cmd, float vw_cmd)
{
    g_app_zone1_dbg.last_apply_ok = 0U;
    g_app_zone1_dbg.last_apply_axis_mask = axis_mask;
    g_app_zone1_dbg.last_apply_vy = vy_cmd;
    g_app_zone1_dbg.last_apply_vw = vw_cmd;

    if (Process_Flow_ChassisOverrideCanWrite(axis_mask, APP_ZONE1_CHASSIS_PRIO_MOTION) == 0U)
    {
        return;
    }
    Process_Flow_SetChassisOverrideAxes(axis_mask, APP_ZONE1_CHASSIS_PRIO_MOTION, vx_cmd, vy_cmd, vw_cmd);
    g_app_zone1_dbg.last_apply_ok = 1U;
}

static uint8_t app_zone1_flow_post_nav_turn(app_zone1_nav_turn_t turn)
{
    if (turn == app_zone1_nav_turn_90)
    {
#if APP_ZONE2_RED_SIDE
        return YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_right_90);
#else
        return YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_left_90);
#endif
    }
    if (turn == app_zone1_nav_turn_180)
    {
        return YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_180);
    }
    return 1U;
}

static uint8_t app_zone1_flow_nav_leg_complete(odom_nav_goto_err_t nav_rc)
{
    return (uint8_t)((nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) && (YawHeadingCtrl_IsBusy() == 0U));
}

static float app_zone1_flow_get_chassis_rpm_abs_avg(void)
{
    float rpm_sum = 0.0f;

    rpm_sum += fabsf((float)chassis_motor1.speed_rpm);
    rpm_sum += fabsf((float)chassis_motor2.speed_rpm);
    rpm_sum += fabsf((float)chassis_motor3.speed_rpm);
    rpm_sum += fabsf((float)chassis_motor4.speed_rpm);
    return rpm_sum * 0.25f;
}

static uint8_t app_zone1_flow_limit_hit_detect(float cmd_abs, float meas_abs, uint32_t now_ms)
{
    if ((cmd_abs >= g_app_zone1_cfg.limit_cmd_thr) &&
        (meas_abs <= g_app_zone1_cfg.limit_meas_rpm_thr))
    {
        if (g_app_zone1_ctx.limit_detect_start_ms == 0U)
        {
            g_app_zone1_ctx.limit_detect_start_ms = now_ms;
        }
        if ((now_ms - g_app_zone1_ctx.limit_detect_start_ms) >= g_app_zone1_cfg.limit_debounce_ms)
        {
            return 1U;
        }
        return 0U;
    }
    g_app_zone1_ctx.limit_detect_start_ms = 0U;
    return 0U;
}

static void app_zone1_set_nav_target(float x_m, float y_m)
{
    odom_nav_goto_set_target(x_m, y_m);
    g_app_zone1_ctx.target.x_m = x_m;
    g_app_zone1_ctx.target.y_m = y_m;
    g_app_zone1_ctx.target.session_id = odom_nav_target.session_id;
}

static uint8_t app_zone1_flow_yaw_turn_begin(app_zone1_nav_turn_t turn)
{
    g_app_zone1_ctx.yaw_cmd_issued = 0U;
    if (turn == app_zone1_nav_turn_none)
    {
        return 1U;
    }
    if (app_zone1_flow_post_nav_turn(turn) == 0U)
    {
        return 0U;
    }
    g_app_zone1_ctx.yaw_cmd_issued = 1U;
    return 1U;
}

static void app_zone1_flow_nav_leg_begin(float x_m, float y_m, app_zone1_nav_turn_t turn)
{
    app_zone1_flow_release_for_nav();
    odom_nav_goto_disarm();
    app_zone1_set_nav_target(x_m, y_m);
    g_app_zone1_ctx.yaw_cmd_issued = 0U;
    if (turn != app_zone1_nav_turn_none)
    {
        if (app_zone1_flow_post_nav_turn(turn) == 0U)
        {
            g_app_zone1_ctx.yaw_cmd_issued = 0U;
            return;
        }
        g_app_zone1_ctx.yaw_cmd_issued = 1U;
    }
}

static odom_nav_goto_err_t app_zone1_flow_nav_peek(void)
{
    odom_nav_goto_err_t nav_rc = odom_nav_goto_peek_last_run_result();

    if (g_app_zone1_ctx.target.session_id != odom_nav_target.session_id)
    {
        return ODOM_NAV_GOTO_ERR_DISARMED;
    }
    return nav_rc;
}

static int8_t app_zone1_flow_grab_default_sweep_dir(void)
{
#if APP_ZONE2_RED_SIDE
    return (int8_t)APP_ZONE1_GRAB_SWEEP_DIR_RED;
#else
    return (int8_t)APP_ZONE1_GRAB_SWEEP_DIR_BLUE;
#endif
}

static uint8_t app_zone1_flow_read_center_y(float *y_out)
{
    float cx = 0.0f;
    float cy = 0.0f;

    if (y_out == 0)
    {
        return 0U;
    }
    if (odom_center_offset_latest_center(&cx, &cy) == 0U)
    {
        g_app_zone1_ctx.center_y_valid = 0U;
        return 0U;
    }
    g_app_zone1_ctx.center_y_m = cy;
    g_app_zone1_ctx.center_y_valid = 1U;
    *y_out = cy;
    return 1U;
}

static float app_zone1_flow_grab_sweep_vw_cmd(int8_t sweep_dir)
{
    return (float)sweep_dir * fabsf(g_app_zone1_cfg.shift_right_slow_cmd);
}

static void app_zone1_flow_enter_state(app_zone1_state_t state, uint32_t now_ms);

static void app_zone1_flow_grab_monitor_begin(uint32_t now_ms)
{
    g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
    g_app_zone1_ctx.grab_sweep_dir = app_zone1_flow_grab_default_sweep_dir();
    g_app_zone1_ctx.grab_retry_count = 0U;
    g_app_zone1_ctx.grab_latched = 0U;
    g_app_zone1_ctx.grab_was_active_in_clamp_wait = 0U;
    g_app_zone1_ctx.clamp_prev_state = ClampHeadCtrl_GetState();
    app_zone1_flow_enter_state(app_zone1_state_shift_right_monitor, now_ms);
}

static void app_zone1_flow_enter_advance_turn180(uint32_t now_ms)
{
    g_app_zone1_ctx.advance_turn180_cmd_failed = 0U;
    if (app_zone1_flow_yaw_turn_begin(app_zone1_nav_turn_180) == 0U)
    {
        g_app_zone1_ctx.advance_turn180_cmd_failed = 1U;
    }
    app_zone1_flow_clear_motion_override();
    odom_nav_goto_disarm();
    app_zone1_flow_enter_state(app_zone1_state_advance_turn180, now_ms);
}

static void app_zone1_flow_enter_done(uint32_t now_ms)
{
    app_zone1_flow_enter_state(app_zone1_state_done, now_ms);
}

static void app_zone1_flow_clamp_wait_exit_success(uint32_t now_ms)
{
    app_zone1_flow_clear_motion_override();
    odom_nav_goto_disarm();

#if APP_ZONE1_SKILL_MODE
    if (g_app_zone1_ctx.skill_lap != 0U)
    {
        app_zone1_flow_enter_done(now_ms);
        return;
    }
#endif
    app_zone1_flow_enter_advance_turn180(now_ms);
}

static uint8_t app_zone1_flow_shift_right_retry_to_monitor(uint32_t now_ms)
{
    if (g_app_zone1_ctx.grab_retry_count >= APP_ZONE1_GRAB_RETRY_MAX)
    {
        app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
        return 1U;
    }

    g_app_zone1_ctx.grab_retry_count++;
    ClampHeadCtrl_Init();
    g_app_zone1_ctx.grab_latched = 0U;
    g_app_zone1_ctx.grab_was_active_in_clamp_wait = 0U;
    g_app_zone1_ctx.clamp_prev_state = ClampHeadCtrl_GetState();
    g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
    app_zone1_flow_enter_state(app_zone1_state_shift_right_monitor, now_ms);
    return 1U;
}

static void app_zone1_flow_wait_r1_exit(uint32_t now_ms, uint8_t notify_dock_ok)
{
    if (notify_dock_ok != 0U)
    {
        ClampHeadCtrl_NotifyDockOk();
    }

#if APP_ZONE1_SKILL_MODE
    if (g_app_zone1_ctx.skill_lap == 0U)
    {
        g_app_zone1_ctx.skill_lap = 1U;
        ClampHeadCtrl_Init();
        g_app_zone1_ctx.grab_latched = 0U;
        g_app_zone1_ctx.grab_retry_count = 0U;
        g_app_zone1_ctx.grab_was_active_in_clamp_wait = 0U;
        g_app_zone1_ctx.clamp_prev_state = ClampHeadCtrl_GetState();
        app_zone1_flow_enter_advance_turn180(now_ms);
        return;
    }
#endif
    app_zone1_flow_enter_done(now_ms);
}

static void app_zone1_flow_enter_state(app_zone1_state_t state, uint32_t now_ms)
{
    g_app_zone1_ctx.state = state;
    g_app_zone1_ctx.state_enter_ms = now_ms;
    g_app_zone1_ctx.limit_detect_start_ms = 0U;
    if (state == app_zone1_state_shift_right_clamp_wait)
    {
        g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U;
        g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms = 0U;
        g_app_zone1_ctx.grab_was_active_in_clamp_wait = 0U;
    }
}

static void app_zone1_flow_run_grab_monitor(uint32_t now_ms,
                                            ClampHeadState prev_s,
                                            ClampHeadState *cur_s_out)
{
    float center_y = 0.0f;
    float y_lo;
    float y_hi;
    float y_margin;
    float vw_cmd;
    ClampHeadState cur_s;

    if (cur_s_out == 0)
    {
        return;
    }

    cur_s = ClampHeadCtrl_GetState();
    *cur_s_out = cur_s;

    if ((prev_s == clamp_head_state_idle) &&
        ((cur_s == clamp_head_state_closing) || (cur_s == clamp_head_state_wait_close_delay)))
    {
        app_zone1_flow_clear_motion_override();
        g_app_zone1_ctx.grab_latched = 1U;
        app_zone1_flow_enter_state(app_zone1_state_shift_right_clamp_wait, now_ms);
        g_app_zone1_ctx.clamp_prev_state = cur_s;
        return;
    }

    if (app_zone1_flow_read_center_y(&center_y) == 0U)
    {
        g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
        vw_cmd = app_zone1_flow_grab_sweep_vw_cmd(g_app_zone1_ctx.grab_sweep_dir);
        app_zone1_flow_apply_chassis_axes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                    PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
                                          0.0f,
                                          g_app_zone1_cfg.shift_right_vy_comp_cmd,
                                          vw_cmd);
        g_app_zone1_ctx.clamp_prev_state = cur_s;
        return;
    }

    y_lo = g_app_zone1_cfg.grab_work_y_min_m;
    y_hi = g_app_zone1_cfg.grab_work_y_max_m;
    y_margin = g_app_zone1_cfg.grab_work_y_margin_m;

    if (center_y > y_hi)
    {
        g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_pull_back;
        vw_cmd = -app_zone1_flow_grab_sweep_vw_cmd(g_app_zone1_ctx.grab_sweep_dir);
        app_zone1_flow_apply_chassis_axes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                    PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
                                          0.0f,
                                          g_app_zone1_cfg.shift_right_vy_comp_cmd,
                                          vw_cmd);
        if (center_y <= (y_hi - y_margin))
        {
            g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
        }
        g_app_zone1_ctx.clamp_prev_state = cur_s;
        return;
    }

    if (center_y < y_lo)
    {
        g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_pull_back;
        vw_cmd = app_zone1_flow_grab_sweep_vw_cmd(g_app_zone1_ctx.grab_sweep_dir);
        app_zone1_flow_apply_chassis_axes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                    PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
                                          0.0f,
                                          g_app_zone1_cfg.shift_right_vy_comp_cmd,
                                          vw_cmd);
        if (center_y >= (y_lo + y_margin))
        {
            g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
        }
        g_app_zone1_ctx.clamp_prev_state = cur_s;
        return;
    }

    g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
    vw_cmd = app_zone1_flow_grab_sweep_vw_cmd(g_app_zone1_ctx.grab_sweep_dir);
    app_zone1_flow_apply_chassis_axes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
                                      0.0f,
                                      g_app_zone1_cfg.shift_right_vy_comp_cmd,
                                      vw_cmd);

    if (center_y >= (y_hi - y_margin))
    {
        g_app_zone1_ctx.grab_sweep_dir = (int8_t)(-g_app_zone1_ctx.grab_sweep_dir);
    }
    else if (center_y <= (y_lo + y_margin))
    {
        g_app_zone1_ctx.grab_sweep_dir = (int8_t)(-g_app_zone1_ctx.grab_sweep_dir);
    }

    g_app_zone1_ctx.clamp_prev_state = cur_s;
}

void AppZone1_Reset(void)
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_disarm();
    g_app_zone1_ctx.state = app_zone1_state_idle;
    g_app_zone1_ctx.state_enter_ms = 0U;
    g_app_zone1_ctx.limit_detect_start_ms = 0U;
    g_app_zone1_ctx.r1_wait_start_ms = 0U;
    g_app_zone1_ctx.r1_pending = 0U;
    g_app_zone1_ctx.yaw_cmd_issued = 0U;
    g_app_zone1_ctx.grab_latched = 0U;
    g_app_zone1_ctx.grab_retry_count = 0U;
    g_app_zone1_ctx.grab_was_active_in_clamp_wait = 0U;
    g_app_zone1_ctx.skill_lap = 0U;
    g_app_zone1_ctx.advance_turn180_cmd_failed = 0U;
    g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
    g_app_zone1_ctx.grab_sweep_dir = app_zone1_flow_grab_default_sweep_dir();
    g_app_zone1_ctx.center_y_m = 0.0f;
    g_app_zone1_ctx.center_y_valid = 0U;
    g_app_zone1_ctx.active = 0U;
    g_app_zone1_ctx.done = 0U;
    g_app_zone1_ctx.failed = 0U;
    g_app_zone1_ctx.clamp_prev_state = clamp_head_state_idle;
    g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U;
    g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms = 0U;
    g_app_zone1_ctx.last_nav_rc = ODOM_NAV_GOTO_ERR_OK_ARRIVED;
    g_app_zone1_ctx.target.x_m = 0.0f;
    g_app_zone1_ctx.target.y_m = 0.0f;
    g_app_zone1_ctx.target.session_id = 0U;

    (void)memset((void *)&g_app_zone1_dbg, 0, sizeof(g_app_zone1_dbg));
}

void AppZone1_Init(void)
{
    AppZone1_Reset();
}

void AppZone1_Start(void)
{
    uint32_t now_ms = osKernelGetTickCount();

    AppZone1_Reset();
    ClampHeadCtrl_Init();

    g_app_zone1_ctx.active = 1U;
    g_app_zone1_ctx.done = 0U;
    g_app_zone1_ctx.failed = 0U;
    g_app_zone1_ctx.grab_latched = 0U;
    g_app_zone1_ctx.r1_pending = 0U;
    g_app_zone1_ctx.skill_lap = 0U;

    app_zone1_flow_nav_leg_begin(g_app_zone1_cfg.open_target_x_m,
                                 g_app_zone1_cfg.open_target_y_m,
                                 app_zone1_nav_turn_90);
    if (g_app_zone1_ctx.yaw_cmd_issued == 0U)
    {
        app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
        return;
    }
    app_zone1_flow_enter_state(app_zone1_state_nav_turn90_to_open, now_ms);
}

uint8_t AppZone1_GetConfig(AppZone1Config *out)
{
    if (out == 0)
    {
        return 0U;
    }
    *out = g_app_zone1_cfg;
    return 1U;
}

uint8_t AppZone1_SetConfig(const AppZone1Config *cfg)
{
    if (app_zone1_cfg_validate(cfg) == 0U)
    {
        return 0U;
    }
    g_app_zone1_cfg = *cfg;
    return 1U;
}

uint8_t AppZone1_SetPostGrabForwardVy(float vy_cmd)
{
    AppZone1Config cfg = g_app_zone1_cfg;

    if (!isfinite(vy_cmd))
    {
        return 0U;
    }
    cfg.post_grab_forward_vy_cmd = vy_cmd;
    return AppZone1_SetConfig(&cfg);
}

void AppZone1_NotifyR1Release(void)
{
    g_app_zone1_ctx.r1_pending = 1U;
}

static void app_zone1_poll_r1_release_sig(void)
{
    r1_link_sig_cmd_t sig;

    if (R1Link_TakeSig(&sig) == 0U)
    {
        return;
    }
    if (sig == r1_link_sig_release)
    {
        AppZone1_NotifyR1Release();
    }
}

uint8_t AppZone1_IsBusy(void)
{
    return g_app_zone1_ctx.active;
}

uint8_t AppZone1_IsDone(void)
{
    return g_app_zone1_ctx.done;
}

uint8_t AppZone1_IsFailed(void)
{
    return g_app_zone1_ctx.failed;
}

void AppZone1_Run(void)
{
    uint32_t now_ms;
    float meas_rpm_abs;
    odom_nav_goto_err_t nav_rc;
    ClampHeadState prev_s;
    ClampHeadState cur_s;

    if (g_app_zone1_ctx.active == 0U)
    {
        app_zone1_dbg_refresh(osKernelGetTickCount(), 0.0f);
        return;
    }

    app_zone1_poll_r1_release_sig();

    now_ms = osKernelGetTickCount();

    if (app_zone1_flow_state_depends_on_nav_odom(g_app_zone1_ctx.state) != 0U)
    {
        if (app_zone1_flow_nav_odom_trustworthy() == 0U)
        {
            Process_Flow_ClearChassisOverride();
            app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            meas_rpm_abs = app_zone1_flow_get_chassis_rpm_abs_avg();
            app_zone1_dbg_refresh(now_ms, meas_rpm_abs);
            return;
        }
    }

    meas_rpm_abs = app_zone1_flow_get_chassis_rpm_abs_avg();

    switch (g_app_zone1_ctx.state)
    {
        case app_zone1_state_nav_turn90_to_open:
            nav_rc = app_zone1_flow_nav_peek();
            g_app_zone1_ctx.last_nav_rc = nav_rc;
            if (app_zone1_flow_nav_leg_complete(nav_rc) != 0U)
            {
                app_zone1_flow_release_for_nav();
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
                g_app_zone1_ctx.yaw_cmd_issued = 0U;
                app_zone1_flow_grab_monitor_begin(now_ms);
                break;
            }
            if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) ||
                (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED))
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) &&
                     ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms))
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            else if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_shift_right_monitor:
            prev_s = g_app_zone1_ctx.clamp_prev_state;
            app_zone1_flow_run_grab_monitor(now_ms, prev_s, &cur_s);
            break;

        case app_zone1_state_shift_right_clamp_wait:
        {
            ClampHeadState clamp_cs;

            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.clamp_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
                break;
            }

            clamp_cs = ClampHeadCtrl_GetState();
            if (clamp_cs != clamp_head_state_idle)
            {
                g_app_zone1_ctx.grab_was_active_in_clamp_wait = 1U;
            }

            if (clamp_cs == clamp_head_state_dock_ok)
            {
                app_zone1_flow_clamp_wait_exit_success(now_ms);
                break;
            }

            if (clamp_cs == clamp_head_state_upright_hold)
            {
                if (ClampHeadCtrl_IsObjectPresentRaw() == 0U)
                {
                    g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U;
                }
                else
                {
                    if (g_app_zone1_ctx.clamp_upright_hold_dwell_started == 0U)
                    {
                        g_app_zone1_ctx.clamp_upright_hold_dwell_started = 1U;
                        g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms = now_ms;
                    }
                    if ((now_ms - g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms) >=
                        g_app_zone1_cfg.clamp_upright_hold_dwell_ms)
                    {
                        app_zone1_flow_clamp_wait_exit_success(now_ms);
                    }
                }
                break;
            }

            if ((clamp_cs == clamp_head_state_idle) &&
                (g_clamp_head_ctrl_dbg.pe9_absent_filt != 0U) &&
                (ClampHeadCtrl_ReachedCloseLimit() == 0U) &&
                (Weapon_ClampMotor_IsBusy() == 0U))
            {
                if (app_zone1_flow_shift_right_retry_to_monitor(now_ms) != 0U)
                {
                    break;
                }
            }

            g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U;
            break;
        }

        case app_zone1_state_advance_turn180:
            if (g_app_zone1_ctx.advance_turn180_cmd_failed != 0U)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
                break;
            }
            app_zone1_flow_apply_chassis_axes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY,
                                              0.0f,
                                              g_app_zone1_cfg.post_grab_forward_vy_cmd,
                                              0.0f);
            if (YawHeadingCtrl_IsBusy() == 0U)
            {
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
                g_app_zone1_ctx.yaw_cmd_issued = 0U;
#if APP_ZONE1_SKILL_MODE
                if (g_app_zone1_ctx.skill_lap != 0U)
                {
                    app_zone1_flow_grab_monitor_begin(now_ms);
                    break;
                }
#endif
                app_zone1_flow_enter_state(app_zone1_state_forward_slow_to_limit, now_ms);
            }
            else if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_forward_slow_to_limit:
            app_zone1_flow_apply_chassis_axes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY,
                                              0.0f,
                                              g_app_zone1_cfg.forward_slow_cmd,
                                              0.0f);
            if (app_zone1_flow_limit_hit_detect(fabsf(g_app_zone1_cfg.forward_slow_cmd),
                                                meas_rpm_abs,
                                                now_ms) != 0U)
            {
                app_zone1_flow_clear_motion_override();
                g_app_zone1_ctx.r1_wait_start_ms = now_ms;
                app_zone1_flow_enter_state(app_zone1_state_wait_r1_release, now_ms);
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.limit_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_wait_r1_release:
            app_zone1_flow_clear_motion_override();
            if (g_app_zone1_ctx.r1_pending != 0U)
            {
                g_app_zone1_ctx.r1_pending = 0U;
                app_zone1_flow_wait_r1_exit(now_ms, 1U);
                break;
            }
#if APP_ZONE1_WAIT_R1_TIMEOUT_ENABLE
            if ((now_ms - g_app_zone1_ctx.r1_wait_start_ms) > g_app_zone1_cfg.r1_wait_timeout_ms)
            {
                app_zone1_flow_wait_r1_exit(now_ms, 0U);
            }
#endif
            break;

        case app_zone1_state_done:
            Process_Flow_ClearChassisOverride();
            g_app_zone1_ctx.active = 0U;
            g_app_zone1_ctx.done = 1U;
            g_app_zone1_ctx.failed = 0U;
            g_app_zone1_ctx.state = app_zone1_state_idle;
            break;

        case app_zone1_state_abort:
            Process_Flow_ClearChassisOverride();
            g_app_zone1_ctx.active = 0U;
            g_app_zone1_ctx.done = 0U;
            g_app_zone1_ctx.failed = 1U;
            g_app_zone1_ctx.state = app_zone1_state_idle;
            break;

        case app_zone1_state_idle:
        default:
            g_app_zone1_ctx.active = 0U;
            g_app_zone1_ctx.state = app_zone1_state_idle;
            break;
    }

    app_zone1_dbg_refresh(now_ms, meas_rpm_abs);
}
