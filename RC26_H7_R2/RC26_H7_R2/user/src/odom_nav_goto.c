/**
 * @file odom_nav_goto.c
 * @brief @ref odom_nav_goto.h：世界系 PI(D) → 旋到车体系 Vy/Vw，Vx=0。
 */
#include "odom_nav_goto.h"

#include "Motion_Task.h"
#include "common.h"
#include "Process_Flow.h"
#include "upper_pc_protocol.h"

#include <math.h>
#include <string.h>

#if ODOM_NAV_GOTO_WATCH_DEBUG
volatile odom_nav_goto_dbg_t g_odom_nav_goto_dbg = {
    .enable = 0U,
    .target_x_m = 0.0f,
    .target_y_m = 0.0f,
    .fire = 0U,
    .last_run_return = 0xFFFFFFFFu,
};
#endif

odom_nav_goto_target_t odom_nav_target = {
    .x_m = 0.0f,
    .y_m = 0.0f,
    .session_id = 0U,
};

/* 现场 Watch 调参存档（与 g_odom_nav_goto_tune 默认一致） */
volatile odom_nav_goto_tune_t g_odom_nav_goto_tune = {
    .kp_far = 220.0f,//远区P
    .kp_near = 260.0f,//近区P
    .ki_far = 2.0f,//远区I
    .ki_near = 150.0f,//近区I
    .kd_xy = 0.5f,//D
    .vmax_forward = 30.0f,//远距速度上限
    .vmax_strafe = 30.0f,//左右速度上限
    .zone_far_enter_m = 0.2f,//远区进入距离
    .zone_near_enter_m = 0.1f,//近区进入距离
    .i_far_limit = 5.0f,//远区I限幅
    .i_near_limit = 20.0f,//近区I限幅
    .position_tolerance_m = 0.005f,//位置容差
    .arrival_confirm_cycles = 100U,//到位确认周期
    .timeout_ms = 8000U,//超时时间
    .last_run_return = 0xFFFFFFFFu,//最近一次运行返回值
};

typedef enum {
    odom_nav_zone_far = 0,
    odom_nav_zone_near = 1,
} odom_nav_zone_t;

typedef struct {
    uint32_t last_session;
    uint32_t t0_ms;
    uint32_t last_ms;
    float prev_ex;
    float prev_ey;
    float ix_far;
    float iy_far;
    float ix_near;
    float iy_near;
    odom_nav_zone_t zone;
    uint8_t xy_arrived_latched;
    uint8_t xy_arrive_streak;
} odom_nav_goto_state_t;

static odom_nav_goto_state_t s_st = {0xFFFFFFFFu, 0u, 0u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                     odom_nav_zone_far, 0U, 0U};

/** 0=disarm 后 run 不写 Vy/Vw；set_target 置 1 */
static uint8_t s_nav_armed;

static odom_nav_goto_status_t s_service_status;

static void odom_nav_goto_reset_session_state(uint32_t session_id)
{
    s_st.last_session = session_id;
    s_st.t0_ms = common_now_ms();
    s_st.last_ms = s_st.t0_ms;
    s_st.prev_ex = 0.0f;
    s_st.prev_ey = 0.0f;
    s_st.ix_far = 0.0f;
    s_st.iy_far = 0.0f;
    s_st.ix_near = 0.0f;
    s_st.iy_near = 0.0f;
    s_st.zone = odom_nav_zone_far;
    s_st.xy_arrived_latched = 0U;
    s_st.xy_arrive_streak = 0U;
}

static void odom_nav_goto_zone_update(float dist_m)
{
    const float far_e = g_odom_nav_goto_tune.zone_far_enter_m;
    const float near_e = g_odom_nav_goto_tune.zone_near_enter_m;

    if (s_st.zone == odom_nav_zone_far)
    {
        if (dist_m < near_e)
        {
            s_st.zone = odom_nav_zone_near;
            s_st.ix_far = 0.0f;
            s_st.iy_far = 0.0f;
        }
    }
    else
    {
        if (dist_m > far_e)
        {
            s_st.zone = odom_nav_zone_far;
            s_st.ix_near = 0.0f;
            s_st.iy_near = 0.0f;
        }
    }
}

static uint8_t odom_nav_goto_vec2_limit_sat(float *vx, float *vy, float vmax)
{
    const float mag = sqrtf((*vx) * (*vx) + (*vy) * (*vy));

    if (mag > vmax && mag > 1e-6f)
    {
        const float s = vmax / mag;

        *vx *= s;
        *vy *= s;
        return 1U;
    }
    return 0U;
}

static uint32_t odom_nav_goto_arrival_confirm_required(void)
{
    uint32_t n = g_odom_nav_goto_tune.arrival_confirm_cycles;

    if (n == 0U)
    {
        n = 1U;
    }
    return n;
}

static int odom_nav_goto_read_pose(float *x_m, float *y_m, float *yaw_deg)
{
    const rc_odom_t *p;

    if (rc_odom_is_valid() == 0U)
    {
        return -1;
    }
    p = rc_get_latest_odom();
    *x_m = p->x;
    *y_m = p->y;
    *yaw_deg = p->yaw;
    return 0;
}

/** Vx=0；Vy 前后、Vw 左右（与 Chassis_Calc 约定一致） */
static void odom_nav_goto_apply_wheel_inputs(float vy_forward, float vw_strafe)
{
    if ((process_flow_chassis_override.axis_mask != 0U) &&
        (process_flow_chassis_override.priority == PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH))
    {
        return;
    }
    process_flow_chassis_override.axis_mask = (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VX | PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                         PROCESS_FLOW_CHASSIS_OVERRIDE_VW);
    process_flow_chassis_override.priority = PROCESS_FLOW_OVERRIDE_PRIORITY_LOW;
    process_flow_chassis_override.vx = 0.0f;
    process_flow_chassis_override.vy = vy_forward;
    process_flow_chassis_override.vw = vw_strafe;
}

static uint8_t odom_nav_goto_can_clear_override(void)
{
    /* 流程高优先级占用时，导航不得清零 override */
    if ((process_flow_chassis_override.axis_mask != 0U) &&
        (process_flow_chassis_override.priority == PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH))
    {
        return 0U;
    }
    return 1U;
}

/* 到位：清 override、填 status、disarm；返回值仍为 OK_ARRIVED 供上层状态机 */
static odom_nav_goto_err_t odom_nav_goto_finish_arrived(odom_nav_goto_status_t *status, float dist_m)
{
    if (odom_nav_goto_can_clear_override() != 0U)
    {
        Process_Flow_ClearChassisOverride();
    }
    if (status != NULL)
    {
        status->distance_to_target_m = dist_m;
        status->at_xy = 1U;
        status->vy_cmd = 0.0f;
        status->vw_cmd = 0.0f;
    }
    odom_nav_goto_disarm();
    return ODOM_NAV_GOTO_ERR_OK_ARRIVED;
}

static int odom_nav_goto_validate_tune(void)
{
    const volatile odom_nav_goto_tune_t *t = &g_odom_nav_goto_tune;

    if (t->kp_far < 0.0f || t->kp_near < 0.0f || t->ki_far < 0.0f || t->ki_near < 0.0f || t->kd_xy < 0.0f)
    {
        return 0;
    }
    if (t->vmax_forward <= 0.0f || t->vmax_strafe <= 0.0f)
    {
        return 0;
    }
    if (t->position_tolerance_m <= 0.0f)
    {
        return 0;
    }
    if (t->timeout_ms == 0u)
    {
        return 0;
    }
    if (t->i_far_limit <= 0.0f || t->i_near_limit <= 0.0f)
    {
        return 0;
    }
    if (t->zone_near_enter_m <= 0.0f || t->zone_far_enter_m <= t->zone_near_enter_m)
    {
        return 0;
    }
    return 1;
}

void odom_nav_goto_clear_state(void)
{
    (void)memset(&s_st, 0, sizeof(s_st));
    s_st.last_session = 0xFFFFFFFFu;
    s_st.xy_arrived_latched = 0U;
}

void odom_nav_goto_disarm(void)
{
    s_nav_armed = 0U;
    if (odom_nav_goto_can_clear_override() != 0U)
    {
        Process_Flow_ClearChassisOverride();
    }
    odom_nav_goto_clear_state();
}

uint8_t odom_nav_goto_is_armed(void)
{
    return s_nav_armed;
}

void odom_nav_goto_set_target(float x_m, float y_m)
{
    odom_nav_target.x_m = x_m;
    odom_nav_target.y_m = y_m;

    /* 换目标自动刷新会话号，触发 run() 内部状态重置 */
    if (odom_nav_target.session_id < 0xFFFFFFFFu)
    {
        odom_nav_target.session_id++;
    }
    s_nav_armed = 1U;
}

odom_nav_goto_err_t odom_nav_goto_run(const odom_nav_goto_target_t *target, odom_nav_goto_status_t *status)
{
    float x_m;
    float y_m;
    float yaw_deg;
    int pose_rc;
    uint32_t now_ms;
    float dt_s;
    float ex;
    float ey;
    float dist;
    float yaw_rad;
    float v_wx;
    float v_wy;
    float vy_fwd;
    float vw_str;
    uint8_t xy_in_tol;
    uint32_t confirm_required;
    odom_nav_goto_err_t ret;

    if (target == NULL)
    {
        ret = ODOM_NAV_GOTO_ERR_NULL_POINTER;//空指针
        goto out;
    }

    if (!odom_nav_goto_validate_tune())
    {
        ret = ODOM_NAV_GOTO_ERR_BAD_CONFIG;//配置错误
        goto out;
    }

    if (s_nav_armed == 0U)
    {
        if (status != NULL)
        {
            (void)memset(status, 0, sizeof(*status));
        }
        ret = ODOM_NAV_GOTO_ERR_DISARMED;
        goto out;
    }

    if (s_st.last_session != target->session_id)
    {
        odom_nav_goto_reset_session_state(target->session_id);
    }

    now_ms = common_now_ms();
    dt_s = (float)((int32_t)(now_ms - s_st.last_ms)) * 0.001f;
    if (dt_s < 1e-4f || dt_s > 0.5f)
    {
        dt_s = 0.02f;
    }
    s_st.last_ms = now_ms;

    if ((now_ms - s_st.t0_ms) >= g_odom_nav_goto_tune.timeout_ms)
    {
        if (status != NULL)
        {
            (void)memset(status, 0, sizeof(*status));
        }
        odom_nav_goto_disarm();
        ret = ODOM_NAV_GOTO_ERR_TIMEOUT;//超时
        goto out;
    }

    x_m = 0.0f;
    y_m = 0.0f;
    yaw_deg = 0.0f;
    pose_rc = odom_nav_goto_read_pose(&x_m, &y_m, &yaw_deg);
    if (pose_rc != 0)
    {
        ret = ODOM_NAV_GOTO_ERR_ODOM_READ;//里程计读取错误
        goto out;
    }

    ex = target->x_m - x_m;
    ey = target->y_m - y_m;
    dist = sqrtf(ex * ex + ey * ey);

    odom_nav_goto_zone_update(dist);

    yaw_rad = yaw_deg * (M_PI_F / 180.0f);

    xy_in_tol = (uint8_t)((dist <= g_odom_nav_goto_tune.position_tolerance_m) ? 1U : 0U);
    confirm_required = odom_nav_goto_arrival_confirm_required();

    if (s_st.xy_arrived_latched != 0U)
    {
        if (xy_in_tol == 0U)
        {
            s_st.xy_arrived_latched = 0U;
            s_st.xy_arrive_streak = 0U;
        }
        else
        {
            ret = odom_nav_goto_finish_arrived(status, dist);
            goto out;
        }
    }

    if (xy_in_tol != 0U)
    {
        if (s_st.xy_arrive_streak < 255U)
        {
            s_st.xy_arrive_streak++;
        }
        if ((uint32_t)s_st.xy_arrive_streak >= confirm_required)
        {
            s_st.xy_arrived_latched = 1U;
            ret = odom_nav_goto_finish_arrived(status, dist);
            goto out;
        }
    }
    else
    {
        s_st.xy_arrive_streak = 0U;
    }

    if (status != NULL)
    {
        status->distance_to_target_m = dist;
        status->at_xy = 0U;
        status->vy_cmd = 0.0f;
        status->vw_cmd = 0.0f;
    }

    {
        const volatile odom_nav_goto_tune_t *t = &g_odom_nav_goto_tune;
        float kp;
        float ki;
        float i_lim;
        float vmax_w;
        uint8_t sat_w;

        if (s_st.zone == odom_nav_zone_far)
        {
            kp = t->kp_far;
            ki = t->ki_far;
            i_lim = t->i_far_limit;
            v_wx = kp * ex + ki * s_st.ix_far;
            v_wy = kp * ey + ki * s_st.iy_far;
        }
        else
        {
            kp = t->kp_near;
            ki = t->ki_near;
            i_lim = t->i_near_limit;
            v_wx = kp * ex + ki * s_st.ix_near;
            v_wy = kp * ey + ki * s_st.iy_near;
        }

        if (t->kd_xy > 0.0f)
        {
            v_wx += t->kd_xy * (ex - s_st.prev_ex) / dt_s;
            v_wy += t->kd_xy * (ey - s_st.prev_ey) / dt_s;
        }
        s_st.prev_ex = ex;
        s_st.prev_ey = ey;

        vmax_w = t->vmax_forward;
        sat_w = odom_nav_goto_vec2_limit_sat(&v_wx, &v_wy, vmax_w);

        if (sat_w == 0U)
        {
            if (s_st.zone == odom_nav_zone_far)
            {
                s_st.ix_far += ex * dt_s;
                s_st.iy_far += ey * dt_s;
                s_st.ix_far = clampf(s_st.ix_far, -i_lim, i_lim);
                s_st.iy_far = clampf(s_st.iy_far, -i_lim, i_lim);
            }
            else
            {
                s_st.ix_near += ex * dt_s;
                s_st.iy_near += ey * dt_s;
                s_st.ix_near = clampf(s_st.ix_near, -i_lim, i_lim);
                s_st.iy_near = clampf(s_st.iy_near, -i_lim, i_lim);
            }
        }
    }

    /* 世界系：+X 前进、+Y 左；yaw 为从 +X 到车头逆时针为正。车体系：Vy 前后、Vw 横移右为正。
     * 红：标准旋到车体；蓝：半场镜像对应另一组 sin/cos 组合。 */
#if APP_ZONE2_RED_SIDE
    vy_fwd = cosf(yaw_rad) * v_wy - sinf(yaw_rad) * v_wx;
    vw_str = sinf(yaw_rad) * v_wx + cosf(yaw_rad) * v_wy;
#else
    vy_fwd = sinf(yaw_rad) * v_wx + cosf(yaw_rad) * v_wy;
    vw_str = sinf(yaw_rad) * v_wy - cosf(yaw_rad) * v_wx;
#endif

    {
        const float vmax_b = g_odom_nav_goto_tune.vmax_forward;

        vy_fwd = clampf(vy_fwd, -vmax_b, vmax_b);
        vw_str = clampf(vw_str, -g_odom_nav_goto_tune.vmax_strafe, g_odom_nav_goto_tune.vmax_strafe);
    }

    if (status != NULL)
    {
        status->vy_cmd = vy_fwd;
        status->vw_cmd = vw_str;
    }

    odom_nav_goto_apply_wheel_inputs(vy_fwd, vw_str);

    ret = ODOM_NAV_GOTO_ERR_OK_MOVING;

out:
    g_odom_nav_goto_tune.last_run_return = (uint32_t)ret;
    return ret;
}

odom_nav_goto_err_t odom_nav_goto_peek_last_run_result(void)
{
    const uint32_t r = g_odom_nav_goto_tune.last_run_return;

    if (r == 0xFFFFFFFFu)
    {
        return ODOM_NAV_GOTO_ERR_DISARMED;
    }
    return (odom_nav_goto_err_t)r;
}

const odom_nav_goto_status_t *odom_nav_goto_peek_last_status(void)
{
    return &s_service_status;
}

void odom_nav_goto_service_tick(void)
{
    if (control_mode != full_auto_control)
    {
        return;
    }
    if (Process_UpStairs_IsBusy() != 0U || Process_DownStairs_IsBusy() != 0U || Process_GetKFS_IsBusy() != 0U)
    {
        return;
    }

    {
        odom_nav_goto_target_t snap;

        snap.x_m = odom_nav_target.x_m;
        snap.y_m = odom_nav_target.y_m;
        snap.session_id = odom_nav_target.session_id;
        (void)odom_nav_goto_run(&snap, &s_service_status);
    }

#if ODOM_NAV_GOTO_WATCH_DEBUG
    if (g_odom_nav_goto_dbg.enable != 0U && g_odom_nav_goto_dbg.fire != 0U)
    {
        g_odom_nav_goto_dbg.last_run_return = (uint32_t)odom_nav_goto_peek_last_run_result();
    }
#endif
}

#if ODOM_NAV_GOTO_WATCH_DEBUG
void odom_nav_goto_poll_debug(void)
{
    static uint32_t s_last_fire = 0U;
    static uint32_t s_session = 0U;
    static uint8_t s_armed = 0U;

    /* Bench debug only when nothing else owns goto (no zone2, no CH5/CH7 flow_*). */
    const uint8_t mode_ok =
        (control_mode == full_auto_control && flow_mode == flow_none && app_flow_mode == app_flow_none) ? 1U
                                                                                                         : 0U;

    if (mode_ok == 0U || g_odom_nav_goto_dbg.enable == 0U)
    {
        if (s_armed != 0U)
        {
            if (mode_ok != 0U)
            {
                odom_nav_goto_disarm();
            }
            s_armed = 0U;
            s_last_fire = 0U;
        }
        return;
    }

    /* fire==0：未触发，避免 enable 后默认飞向 (0,0) */
    if (g_odom_nav_goto_dbg.fire == 0U)
    {
        return;
    }

    if (g_odom_nav_goto_dbg.fire != s_last_fire)
    {
        s_last_fire = g_odom_nav_goto_dbg.fire;
        if (s_session < 0xFFFFFFFEu)
        {
            s_session++;
        }
        else
        {
            s_session = 1U;
        }
        odom_nav_goto_set_target(g_odom_nav_goto_dbg.target_x_m, g_odom_nav_goto_dbg.target_y_m);
        odom_nav_target.session_id = s_session;
        s_armed = 1U;
    }
}
#endif
