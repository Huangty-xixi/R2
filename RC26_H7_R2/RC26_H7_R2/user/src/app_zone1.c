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

#define APP_ZONE1_CHASSIS_AXES_NAV             ((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY | \
                                                         PROCESS_FLOW_CHASSIS_OVERRIDE_VW))  // VY和VW
#define APP_ZONE1_CHASSIS_PRIO_MOTION          PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH  // 高优先级

#define APP_ZONE1_GRAB_SWEEP_DIR_BLUE          (1)  // 蓝方扫掠方向
#define APP_ZONE1_GRAB_SWEEP_DIR_RED           (-1)  // 红方扫掠方向
#define APP_ZONE1_GRAB_SWEEP_VW_MIN_SCALE      (0.6f)  // 贴边扫掠最低速比例
#define APP_ZONE1_GRAB_Y_HYSTERESIS_FACTOR     (2.0f)  // 边界带滞后系数（相对 margin）

/* 夹取 Y 工作区：技能赛红蓝各半区，竞技赛红蓝共用全段（APP_ZONE1_SKILL_MODE + APP_ZONE2_RED_SIDE） */
#if APP_ZONE1_SKILL_MODE
#if APP_ZONE2_RED_SIDE
#define APP_ZONE1_GRAB_WORK_Y_MIN_M            (0.39f)/* 0.52-0.13=0.39*/
#define APP_ZONE1_GRAB_WORK_Y_MAX_M            (0.97f)/* 1.09-0.13=0.96 */
#else
#define APP_ZONE1_GRAB_WORK_Y_MIN_M            (0.87f)/* 1.09-0.13=0.96 */
#define APP_ZONE1_GRAB_WORK_Y_MAX_M            (1.45f)/* 1.64-0.13=1.51 */
#endif
#else
#define APP_ZONE1_GRAB_WORK_Y_MIN_M            (0.39f)/* 0.52-0.13=0.39*/
#define APP_ZONE1_GRAB_WORK_Y_MAX_M            (1.45f)/* 1.64-0.13=1.51 */
#endif

typedef enum
{
    app_zone1_grab_y_zone_unknown = 0,
    app_zone1_grab_y_zone_below,
    app_zone1_grab_y_zone_lo_band,
    app_zone1_grab_y_zone_mid,
    app_zone1_grab_y_zone_hi_band,
    app_zone1_grab_y_zone_above,
} app_zone1_grab_y_zone_t;

typedef enum
{
    app_zone1_nav_turn_none = 0,
    app_zone1_nav_turn_90,
    app_zone1_nav_turn_180,
} app_zone1_nav_turn_t;

volatile AppZone1Config g_app_zone1_cfg = {
    .open_target_x_m = APP_ZONE1_OPEN_TARGET_X_M,
    .open_target_y_m = APP_ZONE1_OPEN_TARGET_Y_M,
    .action_timeout_ms = 15000U, // 15s   动作超时时间
    .nav_odom_max_age_ms = APP_ZONE1_NAV_ODOM_MAX_AGE_MS_DEFAULT, // 500ms 导航odom最大年龄
    .grab_work_y_min_m = APP_ZONE1_GRAB_WORK_Y_MIN_M,
    .grab_work_y_max_m = APP_ZONE1_GRAB_WORK_Y_MAX_M,
    .grab_work_y_margin_m = 0.02f, // 0.02m   夹取Y工作区边距
    .shift_right_slow_cmd = 40.0f, // 40.0f 扫掠慢速速度        
    .shift_right_vy_comp_cmd = -8.0f, // -8.0f 扫掠补偿速度
    .sweep_anchor_y_m = { 0.42f, 0.62f, 0.82f, 1.02f, 1.22f, 1.42f }, /* 标定；各锚点*/
    .sweep_anchor_slow_radius_m = 0.06f, /* 锚点减速带半径 6cm */
    .clamp_timeout_ms = 30000U, // 30s   夹爪超时时间
    .clamp_upright_hold_dwell_ms = 2000U, // 2s   夹爪直立保持时间
    .post_grab_forward_vy_cmd = 10.0f, // 10.0f 夹取后旋转前进补偿   
    .forward_slow_cmd = 15.0f, // 15.0f 慢进速度
    .limit_meas_rpm_thr = 10.0f, // 10.0f 单轮堵转转速阈值
    .limit_stall_wheel_min = 3U, // 至少 3 轮低于阈值判限位（容忍 1 轮悬空）
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
    uint8_t grab_y_zone; // 滞后 Y 工作区分区（扫掠翻向边沿检测） // 上边界本轮已翻向 // 下边界本轮已翻向 // 1=允许 Y 小端翻向（离开小端滞后带后置位）
    uint8_t skill_lap; // 技能圈数标志
    uint8_t advance_turn180_cmd_failed; // 转180命令失败标志
    AppZone1GrabPhase grab_phase; // 夹爪扫掠阶段
    int8_t grab_sweep_dir; // 夹爪扫掠方向
    float center_y_m; // 中心Y坐标
    uint8_t center_y_valid; // 中心Y坐标有效标志
    float grab_sweep_vw_scale; // 扫掠 vw 综合减速比例
    uint8_t active; // 活动标志
    uint8_t done; // 完成标志
    uint8_t failed; // 失败标志
    uint32_t clamp_idle_absent_start_ms; // 失料计时(PE9=0持续200ms后retry)
    uint8_t clamp_upright_hold_dwell_started; // 夹爪直立保持开始标志
    uint32_t clamp_upright_hold_dwell_start_ms; // 夹爪直立保持开始时间
    ClampHeadState clamp_prev_state; // 夹爪前状态
    odom_nav_goto_target_t target; // 导航目标
    odom_nav_goto_err_t last_nav_rc; // 导航错误码
} app_zone1_ctx_t;

 app_zone1_ctx_t g_app_zone1_ctx; // 一区上下文

static uint8_t s_has_mission;       /* 对标二区 s_has_mission */
static uint8_t s_nav_leg_running;   /* 1=本段导航已 arm，未到点/未失败前不重复 set */
static uint32_t s_nav_leg_session;  /* 与 odom_nav_target.session_id 对齐 */

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
    {
        uint8_t ai;

        for (ai = 0U; ai < APP_ZONE1_SWEEP_ANCHOR_COUNT; ai++)
        {
            if (!isfinite(cfg->sweep_anchor_y_m[ai]))
            {
                return 0U;
            }
        }
    }
    if (!isfinite(cfg->sweep_anchor_slow_radius_m) || (cfg->sweep_anchor_slow_radius_m < 0.0f))
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
    if ((cfg->limit_stall_wheel_min == 0U) || (cfg->limit_stall_wheel_min > 4U))
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

static void app_zone1_dbg_refresh(uint32_t now_ms, float meas_rpm_abs, uint8_t stall_wheel_count)
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
    g_app_zone1_dbg.grab_retry_count = g_app_zone1_ctx.grab_retry_count;

    g_app_zone1_dbg.clamp_prev_state = g_app_zone1_ctx.clamp_prev_state;

    g_app_zone1_dbg.r1_pending = g_app_zone1_ctx.r1_pending;
    g_app_zone1_dbg.yaw_cmd_issued = g_app_zone1_ctx.yaw_cmd_issued;
    g_app_zone1_dbg.skill_lap = g_app_zone1_ctx.skill_lap;

    g_app_zone1_dbg.grab_phase = g_app_zone1_ctx.grab_phase;
    g_app_zone1_dbg.grab_sweep_dir = g_app_zone1_ctx.grab_sweep_dir;
    g_app_zone1_dbg.grab_y_zone = g_app_zone1_ctx.grab_y_zone;
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
    g_app_zone1_dbg.grab_sweep_vw_scale = g_app_zone1_ctx.grab_sweep_vw_scale;

    g_app_zone1_dbg.limit_detect_start_ms = g_app_zone1_ctx.limit_detect_start_ms;
    g_app_zone1_dbg.chassis_rpm_abs_avg = meas_rpm_abs;
    g_app_zone1_dbg.limit_stall_wheel_count = stall_wheel_count;

    g_app_zone1_dbg.last_nav_rc = g_app_zone1_ctx.last_nav_rc;
    g_app_zone1_dbg.nav_target_x_m = g_app_zone1_ctx.target.x_m;
    g_app_zone1_dbg.nav_target_y_m = g_app_zone1_ctx.target.y_m;

    g_app_zone1_dbg.pf_axis_mask = (uint8_t)process_flow_chassis_override.axis_mask;
    g_app_zone1_dbg.pf_override_vy = process_flow_chassis_override.vy;
    g_app_zone1_dbg.pf_override_vw = process_flow_chassis_override.vw;
}

static uint8_t app_zone1_flow_state_depends_on_nav_odom(app_zone1_state_t st)
{
    return (uint8_t)((st == app_zone1_state_nav_turn_open) ||
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
        (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
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
        /* 绝对四向目标，与定点调试 RunFieldDir 一致 */
#if APP_ZONE2_RED_SIDE
        YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_RIGHT);
#else
        YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_LEFT);
#endif
        return 1U;
    }
    if (turn == app_zone1_nav_turn_180)
    {
        return YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_180);
    }
    return 1U;
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

static uint8_t app_zone1_flow_count_stall_wheels(float rpm_thr)
{
    uint8_t count = 0U;

    if (fabsf((float)chassis_motor1.speed_rpm) <= rpm_thr)
    {
        count++;
    }
    if (fabsf((float)chassis_motor2.speed_rpm) <= rpm_thr)
    {
        count++;
    }
    if (fabsf((float)chassis_motor3.speed_rpm) <= rpm_thr)
    {
        count++;
    }
    if (fabsf((float)chassis_motor4.speed_rpm) <= rpm_thr)
    {
        count++;
    }
    return count;
}

static uint8_t app_zone1_flow_limit_hit_detect(float cmd_abs, uint8_t stall_wheel_count, uint32_t now_ms)
{
    if ((cmd_abs >= g_app_zone1_cfg.limit_cmd_thr) &&
        (stall_wheel_count >= g_app_zone1_cfg.limit_stall_wheel_min))
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

static void app_zone1_flow_nav_abort(void)
{
    odom_nav_goto_disarm();
    s_nav_leg_running = 0U;
    s_nav_leg_session = 0U;
}

/** 对标二区 z2_exec_nav_start_xy：disarm → set_target → 标记本段 leg */
static void app_zone1_flow_nav_start_xy(float x_m, float y_m)
{
    odom_nav_goto_set_tolerance_m(0.05f);
    YawHeadingCtrl_ParallelLegSettleReset();
    app_zone1_flow_release_for_nav();
    Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
    app_zone1_flow_nav_abort();
    app_zone1_set_nav_target(x_m, y_m);
    s_nav_leg_session = odom_nav_target.session_id;
    s_nav_leg_running = 1U;
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

static odom_nav_goto_err_t app_zone1_flow_nav_peek(void)
{
    odom_nav_goto_err_t nav_rc = odom_nav_goto_peek_last_run_result();

    if (s_nav_leg_running == 0U || odom_nav_target.session_id != s_nav_leg_session)
    {
        return ODOM_NAV_GOTO_ERR_DISARMED;
    }
    return nav_rc;
}

/** 对标二区 z2_exec_nav_poll_leg：1=本段结束（到点/超时/容差内 disarm） */
static uint8_t app_zone1_flow_nav_leg_done(odom_nav_goto_err_t nav_rc)
{
    if (s_nav_leg_running == 0U)
    {
        return 0U;
    }

    if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
    {
        s_nav_leg_running = 0U;
        return 1U;
    }

    if ((nav_rc == ODOM_NAV_GOTO_ERR_DISARMED) && (odom_nav_goto_is_armed() == 0U))
    {
        const odom_nav_goto_status_t *st = odom_nav_goto_peek_last_status();

        if ((st != 0) &&
            (st->distance_to_target_m <= g_odom_nav_goto_tune.position_tolerance_m))
        {
            s_nav_leg_running = 0U;
            return 1U;
        }
    }

    if (nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT)
    {
        s_nav_leg_running = 0U;
        odom_nav_goto_disarm();
        return 1U;
    }

    return 0U;
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

static float app_zone1_flow_grab_sweep_vw_scale(float center_y, float y_lo, float y_hi, float y_margin)
{
    float scale = 1.0f;
    float dist_hi;
    float dist_lo;

    if (y_margin <= 0.0f)
    {
        return 1.0f;
    }

    dist_hi = y_hi - center_y;
    dist_lo = center_y - y_lo;
    if (dist_hi < y_margin)
    {
        scale = dist_hi / y_margin;
    }
    else if (dist_lo < y_margin)
    {
        scale = dist_lo / y_margin;
    }

    if (scale < APP_ZONE1_GRAB_SWEEP_VW_MIN_SCALE)
    {
        scale = APP_ZONE1_GRAB_SWEEP_VW_MIN_SCALE;
    }
    if (scale > 1.0f)
    {
        scale = 1.0f;
    }
    return scale;
}

static void app_zone1_flow_sweep_anchor_range(uint8_t *start_out, uint8_t *count_out)
{
    if ((start_out == 0) || (count_out == 0))
    {
        return;
    }

#if APP_ZONE1_SKILL_MODE
#if APP_ZONE2_RED_SIDE
    *start_out = 0U;
    *count_out = APP_ZONE1_SWEEP_ANCHOR_SKILL_PER_SIDE;
#else
    *start_out = APP_ZONE1_SWEEP_ANCHOR_SKILL_PER_SIDE;
    *count_out = APP_ZONE1_SWEEP_ANCHOR_SKILL_PER_SIDE;
#endif
#else
    *start_out = 0U;
    *count_out = APP_ZONE1_SWEEP_ANCHOR_COUNT;
#endif
}

static float app_zone1_flow_grab_sweep_anchor_vw_scale(float center_y)
{
    float scale = 1.0f;
    float radius = g_app_zone1_cfg.sweep_anchor_slow_radius_m;
    uint8_t start_idx;
    uint8_t count;
    uint8_t i;

    if (radius <= 0.0f)
    {
        return 1.0f;
    }

    app_zone1_flow_sweep_anchor_range(&start_idx, &count);
    for (i = 0U; i < count; i++)
    {
        const uint8_t idx = (uint8_t)(start_idx + i);
        const float dist = fabsf(center_y - g_app_zone1_cfg.sweep_anchor_y_m[idx]);

        if (dist < radius)
        {
            float anchor_scale = dist / radius;

            if (anchor_scale < APP_ZONE1_GRAB_SWEEP_VW_MIN_SCALE)
            {
                anchor_scale = APP_ZONE1_GRAB_SWEEP_VW_MIN_SCALE;
            }
            if (anchor_scale < scale)
            {
                scale = anchor_scale;
            }
        }
    }
    return scale;
}

static float app_zone1_flow_grab_sweep_vw_scale_combined(float center_y,
                                                         float y_lo,
                                                         float y_hi,
                                                         float y_margin)
{
    float scale = app_zone1_flow_grab_sweep_vw_scale(center_y, y_lo, y_hi, y_margin);
    float anchor_scale = app_zone1_flow_grab_sweep_anchor_vw_scale(center_y);

    if (anchor_scale < scale)
    {
        scale = anchor_scale;
    }
    return scale;
}

static app_zone1_grab_y_zone_t app_zone1_flow_grab_y_classify(float center_y,
                                                             float y_lo,
                                                             float y_hi,
                                                             float y_margin)
{
    if (center_y < y_lo)
    {
        return app_zone1_grab_y_zone_below;
    }
    if (center_y > y_hi)
    {
        return app_zone1_grab_y_zone_above;
    }
    if (center_y <= (y_lo + y_margin))
    {
        return app_zone1_grab_y_zone_lo_band;
    }
    if (center_y >= (y_hi - y_margin))
    {
        return app_zone1_grab_y_zone_hi_band;
    }
    return app_zone1_grab_y_zone_mid;
}

static void app_zone1_flow_grab_sweep_assign_y_zone(float center_y, float y_lo, float y_hi, float y_margin)
{
    g_app_zone1_ctx.grab_y_zone = (uint8_t)app_zone1_flow_grab_y_classify(center_y, y_lo, y_hi, y_margin);
}

static void app_zone1_flow_grab_sweep_seed_y_zone(float center_y, float y_lo, float y_hi, float y_margin)
{
    app_zone1_flow_grab_sweep_assign_y_zone(center_y, y_lo, y_hi, y_margin);
}

static void app_zone1_flow_grab_try_flip_hi(float center_y, float y_hi, float y_margin)
{
    if (center_y < (y_hi - y_margin))
    {
        return;
    }
    g_app_zone1_ctx.grab_sweep_dir = 1;
}

static void app_zone1_flow_grab_try_flip_lo(float center_y, float y_lo, float y_margin)
{
    if (center_y > (y_lo + y_margin))
    {
        return;
    }
    g_app_zone1_ctx.grab_sweep_dir = -1;
}

static void app_zone1_flow_grab_y_zone_hysteresis_step(float center_y,
                                                       float y_lo,
                                                       float y_hi,
                                                       float y_margin)
{
    app_zone1_grab_y_zone_t prev = (app_zone1_grab_y_zone_t)g_app_zone1_ctx.grab_y_zone;
    app_zone1_grab_y_zone_t cur = prev;
    float hi_enter;
    float hi_exit;
    float lo_enter;
    float lo_exit;
    float hyst;

    hyst = y_margin * APP_ZONE1_GRAB_Y_HYSTERESIS_FACTOR;
    if (hyst > ((y_hi - y_lo) * 0.25f))
    {
        hyst = (y_hi - y_lo) * 0.25f;
    }
    hi_enter = y_hi - y_margin;
    hi_exit = y_hi - hyst;
    if (hi_exit < y_lo)
    {
        hi_exit = y_lo;
    }
    lo_enter = y_lo + y_margin;
    lo_exit = y_lo + hyst;
    if (lo_exit > y_hi)
    {
        lo_exit = y_hi;
    }

    if (center_y < y_lo)
    {
        cur = app_zone1_grab_y_zone_below;
    }
    else if (center_y > y_hi)
    {
        cur = app_zone1_grab_y_zone_above;
    }
    else if (prev == app_zone1_grab_y_zone_unknown)
    {
        cur = app_zone1_flow_grab_y_classify(center_y, y_lo, y_hi, y_margin);
    }
    else
    {
        switch (prev)
        {
            case app_zone1_grab_y_zone_mid:
                if (center_y >= hi_enter)
                {
                    cur = app_zone1_grab_y_zone_hi_band;
                }
                else if (center_y <= lo_enter)
                {
                    cur = app_zone1_grab_y_zone_lo_band;
                }
                else
                {
                    cur = app_zone1_grab_y_zone_mid;
                }
                break;

            case app_zone1_grab_y_zone_hi_band:
                if (center_y < hi_exit)
                {
                    cur = (center_y <= lo_enter) ? app_zone1_grab_y_zone_lo_band : app_zone1_grab_y_zone_mid;
                }
                else
                {
                    cur = app_zone1_grab_y_zone_hi_band;
                }
                break;

            case app_zone1_grab_y_zone_lo_band:
                if (center_y > lo_exit)
                {
                    cur = (center_y >= hi_enter) ? app_zone1_grab_y_zone_hi_band : app_zone1_grab_y_zone_mid;
                }
                else
                {
                    cur = app_zone1_grab_y_zone_lo_band;
                }
                break;

            case app_zone1_grab_y_zone_below:
            case app_zone1_grab_y_zone_above:
            default:
                cur = app_zone1_flow_grab_y_classify(center_y, y_lo, y_hi, y_margin);
                break;
        }
    }

    g_app_zone1_ctx.grab_y_zone = (uint8_t)cur;
    app_zone1_flow_grab_try_flip_hi(center_y, y_hi, y_margin);
    app_zone1_flow_grab_try_flip_lo(center_y, y_lo, y_margin);
}

static void app_zone1_flow_grab_apply_sweep_motion(float vw_cmd)
{
    app_zone1_flow_apply_chassis_axes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
                                      0.0f,
                                      g_app_zone1_cfg.shift_right_vy_comp_cmd,
                                      vw_cmd);
}

static void app_zone1_flow_enter_state(app_zone1_state_t state, uint32_t now_ms);

static void app_zone1_flow_enter_reverse_slow_to_limit(uint32_t now_ms)
{
    app_zone1_flow_clear_motion_override();
    odom_nav_goto_disarm();
    app_zone1_flow_enter_state(app_zone1_state_reverse_slow_to_limit, now_ms);
}

static void app_zone1_flow_grab_monitor_begin(uint32_t now_ms)
{
    float center_y = 0.0f;

    g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
    g_app_zone1_ctx.grab_sweep_dir = app_zone1_flow_grab_default_sweep_dir();
    g_app_zone1_ctx.grab_retry_count = 0U;
    g_app_zone1_ctx.grab_latched = 0U;
    if (app_zone1_flow_read_center_y(&center_y) != 0U)
    {
        app_zone1_flow_grab_sweep_seed_y_zone(center_y,
                                              g_app_zone1_cfg.grab_work_y_min_m,
                                              g_app_zone1_cfg.grab_work_y_max_m,
                                              g_app_zone1_cfg.grab_work_y_margin_m);
    }
    else
    {
        g_app_zone1_ctx.grab_y_zone = (uint8_t)app_zone1_grab_y_zone_lo_band;
    }
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
    float center_y = 0.0f;

    if (g_app_zone1_ctx.grab_retry_count >= APP_ZONE1_GRAB_RETRY_MAX)
    {
#if APP_ZONE1_FLOW_THROUGH_ENABLE
        app_zone1_flow_clamp_wait_exit_success(now_ms);
#else
        app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
#endif
        return 1U;
    }

    g_app_zone1_ctx.grab_retry_count++;
    ClampHeadCtrl_Init();
    g_app_zone1_ctx.grab_latched = 0U;
    g_app_zone1_ctx.grab_sweep_dir = app_zone1_flow_grab_default_sweep_dir();
    if (app_zone1_flow_read_center_y(&center_y) != 0U)
    {
        app_zone1_flow_grab_sweep_seed_y_zone(center_y,
                                              g_app_zone1_cfg.grab_work_y_min_m,
                                              g_app_zone1_cfg.grab_work_y_max_m,
                                              g_app_zone1_cfg.grab_work_y_margin_m);
    }
    else
    {
        g_app_zone1_ctx.grab_y_zone = (uint8_t)app_zone1_grab_y_zone_lo_band;
    }
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
        g_app_zone1_ctx.clamp_idle_absent_start_ms = 0U;
        g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U;
        g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms = 0U;
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
    float vw_scale;
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
        app_zone1_flow_apply_chassis_axes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VW), 0.0f, 0.0f, 0.0f);
        g_app_zone1_ctx.grab_latched = 1U;
        app_zone1_flow_enter_state(app_zone1_state_shift_right_clamp_wait, now_ms);
        g_app_zone1_ctx.clamp_prev_state = cur_s;
        return;
    }

    if (app_zone1_flow_read_center_y(&center_y) == 0U)
    {
        g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
        vw_cmd = app_zone1_flow_grab_sweep_vw_cmd(g_app_zone1_ctx.grab_sweep_dir);
        app_zone1_flow_grab_apply_sweep_motion(vw_cmd);
        g_app_zone1_ctx.clamp_prev_state = cur_s;
        return;
    }

    y_lo = g_app_zone1_cfg.grab_work_y_min_m;
    y_hi = g_app_zone1_cfg.grab_work_y_max_m;
    y_margin = g_app_zone1_cfg.grab_work_y_margin_m;

    if (center_y > y_hi)
    {
        app_zone1_flow_grab_try_flip_hi(center_y, y_hi, y_margin);
        g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_pull_back;
        vw_cmd = app_zone1_flow_grab_sweep_vw_cmd(g_app_zone1_ctx.grab_sweep_dir);
        app_zone1_flow_grab_apply_sweep_motion(vw_cmd);
        if (center_y <= (y_hi - y_margin))
        {
            g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
            app_zone1_flow_grab_sweep_assign_y_zone(center_y, y_lo, y_hi, y_margin);
        }
        else
        {
            g_app_zone1_ctx.grab_y_zone = (uint8_t)app_zone1_grab_y_zone_above;
        }
        g_app_zone1_ctx.clamp_prev_state = cur_s;
        return;
    }

    if (center_y < y_lo)
    {
        app_zone1_flow_grab_try_flip_lo(center_y, y_lo, y_margin);
        g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_pull_back;
        vw_cmd = app_zone1_flow_grab_sweep_vw_cmd(g_app_zone1_ctx.grab_sweep_dir);
        app_zone1_flow_grab_apply_sweep_motion(vw_cmd);
        if (center_y >= (y_lo + y_margin))
        {
            g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
            app_zone1_flow_grab_sweep_assign_y_zone(center_y, y_lo, y_hi, y_margin);
        }
        else
        {
            g_app_zone1_ctx.grab_y_zone = (uint8_t)app_zone1_grab_y_zone_below;
        }
        g_app_zone1_ctx.clamp_prev_state = cur_s;
        return;
    }

    g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
    app_zone1_flow_grab_y_zone_hysteresis_step(center_y, y_lo, y_hi, y_margin);
    vw_scale = app_zone1_flow_grab_sweep_vw_scale_combined(center_y, y_lo, y_hi, y_margin);
    g_app_zone1_ctx.grab_sweep_vw_scale = vw_scale;
    vw_cmd = app_zone1_flow_grab_sweep_vw_cmd(g_app_zone1_ctx.grab_sweep_dir) * vw_scale;
    app_zone1_flow_grab_apply_sweep_motion(vw_cmd);

    g_app_zone1_ctx.clamp_prev_state = cur_s;
}

void AppZone1_Reset(void)
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_disarm();
    YawHeadingCtrl_ParallelLegSettleReset();
    g_app_zone1_ctx.state = app_zone1_state_idle;
    g_app_zone1_ctx.state_enter_ms = 0U;
    g_app_zone1_ctx.limit_detect_start_ms = 0U;
    g_app_zone1_ctx.r1_wait_start_ms = 0U;
    g_app_zone1_ctx.r1_pending = 0U;
    g_app_zone1_ctx.yaw_cmd_issued = 0U;
    g_app_zone1_ctx.grab_latched = 0U;
    g_app_zone1_ctx.grab_retry_count = 0U;
    g_app_zone1_ctx.grab_y_zone = (uint8_t)app_zone1_grab_y_zone_unknown;
    g_app_zone1_ctx.skill_lap = 0U;
    g_app_zone1_ctx.advance_turn180_cmd_failed = 0U;
    g_app_zone1_ctx.grab_phase = app_zone1_grab_phase_sweep;
    g_app_zone1_ctx.grab_sweep_dir = app_zone1_flow_grab_default_sweep_dir();
    g_app_zone1_ctx.center_y_m = 0.0f;
    g_app_zone1_ctx.center_y_valid = 0U;
    g_app_zone1_ctx.grab_sweep_vw_scale = 1.0f;
    g_app_zone1_ctx.active = 0U;
    g_app_zone1_ctx.done = 0U;
    g_app_zone1_ctx.failed = 0U;
    g_app_zone1_ctx.clamp_idle_absent_start_ms = 0U;
    g_app_zone1_ctx.clamp_prev_state = clamp_head_state_idle;
    g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U;
    g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms = 0U;
    g_app_zone1_ctx.last_nav_rc = ODOM_NAV_GOTO_ERR_OK_ARRIVED;
    g_app_zone1_ctx.target.x_m = 0.0f;
    g_app_zone1_ctx.target.y_m = 0.0f;
    g_app_zone1_ctx.target.session_id = 0U;
    s_nav_leg_running = 0U;
    s_nav_leg_session = 0U;

    (void)memset((void *)&g_app_zone1_dbg, 0, sizeof(g_app_zone1_dbg));
}

void app_zone1_mission_clear(void)
{
    AppZone1_Reset();
    s_has_mission = 0U;
}

void app_zone1_poll(void)
{
    if (s_has_mission == 0U)
    {
        return;
    }
    AppZone1_Run();
}

uint8_t app_zone1_is_done(void)
{
    return (uint8_t)(g_app_zone1_ctx.done != 0U);
}

uint8_t app_zone1_is_failed(void)
{
    return (uint8_t)(g_app_zone1_ctx.failed != 0U);
}

uint8_t AppZone1_IsBusy(void)
{
    return (uint8_t)(g_app_zone1_ctx.active != 0U);
}

uint8_t AppZone1_ShouldAllowAutoGrab(void)
{
    return (uint8_t)(g_app_zone1_ctx.active != 0U &&
                     g_app_zone1_ctx.state == app_zone1_state_shift_right_monitor);
}

uint8_t AppZone1_ChassisLockHoldActive(void)
{
    if (g_app_zone1_ctx.active == 0U)
    {
        return 0U;
    }
    return (uint8_t)(g_app_zone1_ctx.state == app_zone1_state_wait_r1_release);
}

void AppZone1_Init(void)
{
    app_zone1_mission_clear();
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

    if (app_zone1_flow_post_nav_turn(app_zone1_nav_turn_90) == 0U)
    {
        app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
        return;
    }
    g_app_zone1_ctx.yaw_cmd_issued = 1U;
    app_zone1_flow_nav_start_xy(g_app_zone1_cfg.open_target_x_m,
                                g_app_zone1_cfg.open_target_y_m);
    s_has_mission = 1U;
    app_zone1_flow_enter_state(app_zone1_state_nav_turn_open, now_ms);
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

uint8_t AppZone1_IsDone(void)
{
    return app_zone1_is_done();
}

uint8_t AppZone1_IsFailed(void)
{
    return app_zone1_is_failed();
}

void AppZone1_Run(void)
{
    uint32_t now_ms;
    float meas_rpm_abs;
    uint8_t stall_wheel_count;
    odom_nav_goto_err_t nav_rc;
    ClampHeadState prev_s;
    ClampHeadState cur_s;

    if (g_app_zone1_ctx.active == 0U)
    {
        app_zone1_dbg_refresh(osKernelGetTickCount(), 0.0f, 0U);
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
            stall_wheel_count = app_zone1_flow_count_stall_wheels(g_app_zone1_cfg.limit_meas_rpm_thr);
            app_zone1_dbg_refresh(now_ms, meas_rpm_abs, stall_wheel_count);
            return;
        }
    }

    meas_rpm_abs = app_zone1_flow_get_chassis_rpm_abs_avg();
    stall_wheel_count = app_zone1_flow_count_stall_wheels(g_app_zone1_cfg.limit_meas_rpm_thr);

    switch (g_app_zone1_ctx.state)
    {
        case app_zone1_state_nav_turn_open:
            /* 导航到开口点与转 90° 并发；到点且航向稳态后才进入倒退 */
            nav_rc = app_zone1_flow_nav_peek();
            g_app_zone1_ctx.last_nav_rc = nav_rc;
            if ((YawHeadingCtrl_ParallelLegSettled() != 0U) &&
                (app_zone1_flow_nav_leg_done(nav_rc) != 0U))
            {
                app_zone1_flow_release_for_nav();
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
                g_app_zone1_ctx.yaw_cmd_issued = 0U;
                app_zone1_flow_enter_reverse_slow_to_limit(now_ms);
                break;
            }
            if ((nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
            {
                app_zone1_flow_nav_abort();
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            else if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms)
            {
                app_zone1_flow_nav_abort();
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_reverse_slow_to_limit:
            app_zone1_flow_apply_chassis_axes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY,
                                              0.0f,
                                              -g_app_zone1_cfg.forward_slow_cmd,
                                              0.0f);
            if (app_zone1_flow_limit_hit_detect(fabsf(g_app_zone1_cfg.forward_slow_cmd),
                                                stall_wheel_count,
                                                now_ms) != 0U)
            {
                app_zone1_flow_clear_motion_override();
                app_zone1_flow_grab_monitor_begin(now_ms);
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.limit_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_shift_right_monitor:
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms)
            {
#if APP_ZONE1_FLOW_THROUGH_ENABLE
                app_zone1_flow_enter_advance_turn180(now_ms);
#else
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
#endif
                break;
            }
            prev_s = g_app_zone1_ctx.clamp_prev_state;
            app_zone1_flow_run_grab_monitor(now_ms, prev_s, &cur_s);
            break;

        case app_zone1_state_shift_right_clamp_wait:
        {
            ClampHeadState clamp_cs;

            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.clamp_timeout_ms)
            {
#if APP_ZONE1_FLOW_THROUGH_ENABLE
                app_zone1_flow_clamp_wait_exit_success(now_ms);
#else
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
#endif
                break;
            }

            clamp_cs = ClampHeadCtrl_GetState();

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
                (ClampHeadCtrl_ReachedCloseLimit() == 0U) &&
                (Weapon_ClampMotor_IsBusy() == 0U))
            {
                if (ClampHeadCtrl_IsObjectPresentRaw() == 0U)
                {
                    if (g_app_zone1_ctx.clamp_idle_absent_start_ms == 0U)
                    {
                        g_app_zone1_ctx.clamp_idle_absent_start_ms = now_ms;
                    }
                    if ((now_ms - g_app_zone1_ctx.clamp_idle_absent_start_ms) >= 200U)
                    {
                        g_app_zone1_ctx.clamp_idle_absent_start_ms = 0U;
                        if (app_zone1_flow_shift_right_retry_to_monitor(now_ms) != 0U)
                        {
                            break;
                        }
                    }
                }
                else
                {
                    g_app_zone1_ctx.clamp_idle_absent_start_ms = 0U;
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
                app_zone1_flow_clear_motion_override();
                g_app_zone1_ctx.r1_wait_start_ms = now_ms;
                app_zone1_flow_enter_state(app_zone1_state_wait_r1_release, now_ms);
            }
            else if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms)
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
            s_has_mission = 0U;
            break;

        case app_zone1_state_abort:
            Process_Flow_ClearChassisOverride();
            g_app_zone1_ctx.active = 0U;
            g_app_zone1_ctx.done = 0U;
            g_app_zone1_ctx.failed = 1U;
            g_app_zone1_ctx.state = app_zone1_state_idle;
            s_has_mission = 0U;
            break;

        case app_zone1_state_idle:
        default:
            g_app_zone1_ctx.active = 0U;
            g_app_zone1_ctx.state = app_zone1_state_idle;
            break;
    }

    app_zone1_dbg_refresh(now_ms, meas_rpm_abs, stall_wheel_count);
}
