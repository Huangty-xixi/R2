/**
 * @file odom_nav_goto.c
 * @brief @ref odom_nav_goto.h：世界系 PI(D) → 旋到车体系 Vy/Vw，Vx=0。
 */
#include "odom_nav_goto.h"

#if ODOM_NAV_GOTO_WATCH_DEBUG
#include "Motion_Task.h"
#endif
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

volatile odom_nav_goto_tune_t g_odom_nav_goto_tune = {
    .kp_xy = 220.0f,
    .ki_xy = 1.0f,
    .kd_xy = 0.5f,
    .vmax_forward = 30.0f,
    .vmax_strafe = 30.0f,
    .position_tolerance_m = 0.05f,
    .timeout_ms = 8000U,
    .i_xy_limit = 5.0f,
};

typedef struct {
    uint32_t last_session;
    uint32_t t0_ms;
    uint32_t last_ms;
    float prev_ex;
    float prev_ey;
    float ix;
    float iy;
} odom_nav_goto_state_t;

static odom_nav_goto_state_t s_st = {0xFFFFFFFFu, 0u, 0u, 0.0f, 0.0f, 0.0f, 0.0f};

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

static int odom_nav_goto_validate_tune(void)
{
    const volatile odom_nav_goto_tune_t *t = &g_odom_nav_goto_tune;

    if (t->kp_xy < 0.0f || t->ki_xy < 0.0f || t->kd_xy < 0.0f)
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
    if (t->i_xy_limit <= 0.0f)
    {
        return 0;
    }
    return 1;
}

void odom_nav_goto_clear_state(void)
{
    (void)memset(&s_st, 0, sizeof(s_st));
    s_st.last_session = 0xFFFFFFFFu;
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
    uint8_t xy_done;

    if (target == NULL)
    {
        return ODOM_NAV_GOTO_ERR_NULL_POINTER;//空指针
    }

    if (!odom_nav_goto_validate_tune())
    {
        return ODOM_NAV_GOTO_ERR_BAD_CONFIG;//配置错误
    }

    if (s_st.last_session != target->session_id)
    {
        s_st.last_session = target->session_id;
        s_st.t0_ms = common_now_ms();
        s_st.last_ms = s_st.t0_ms;
        s_st.prev_ex = 0.0f;
        s_st.prev_ey = 0.0f;
        s_st.ix = 0.0f;
        s_st.iy = 0.0f;
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
        if (odom_nav_goto_can_clear_override() != 0U)
        {
            Process_Flow_ClearChassisOverride();
        }
        if (status != NULL)
        {
            (void)memset(status, 0, sizeof(*status));
        }
        return ODOM_NAV_GOTO_ERR_TIMEOUT;//超时
    }

    x_m = 0.0f;
    y_m = 0.0f;
    yaw_deg = 0.0f;
    pose_rc = odom_nav_goto_read_pose(&x_m, &y_m, &yaw_deg);
    if (pose_rc != 0)
    {
        return ODOM_NAV_GOTO_ERR_ODOM_READ;//里程计读取错误
    }

    ex = target->x_m - x_m;
    ey = target->y_m - y_m;
    dist = sqrtf(ex * ex + ey * ey);

    yaw_rad = yaw_deg * (M_PI_F / 180.0f);

    xy_done = (dist <= g_odom_nav_goto_tune.position_tolerance_m) ? 1u : 0u;

    if (status != NULL)
    {
        status->distance_to_target_m = dist;
        status->at_xy = xy_done;
        status->vy_cmd = 0.0f;
        status->vw_cmd = 0.0f;
    }

    if (xy_done != 0u)
    {
        if (odom_nav_goto_can_clear_override() != 0U)
        {
            Process_Flow_ClearChassisOverride();
        }
        return ODOM_NAV_GOTO_ERR_OK_ARRIVED;
    }

    s_st.ix += ex * dt_s;
    s_st.iy += ey * dt_s;
    s_st.ix = clampf(s_st.ix, -g_odom_nav_goto_tune.i_xy_limit, g_odom_nav_goto_tune.i_xy_limit);
    s_st.iy = clampf(s_st.iy, -g_odom_nav_goto_tune.i_xy_limit, g_odom_nav_goto_tune.i_xy_limit);

    v_wx = g_odom_nav_goto_tune.kp_xy * ex + g_odom_nav_goto_tune.ki_xy * s_st.ix;
    v_wy = g_odom_nav_goto_tune.kp_xy * ey + g_odom_nav_goto_tune.ki_xy * s_st.iy;
    if (g_odom_nav_goto_tune.kd_xy > 0.0f)
    {
        v_wx += g_odom_nav_goto_tune.kd_xy * (ex - s_st.prev_ex) / dt_s;
        v_wy += g_odom_nav_goto_tune.kd_xy * (ey - s_st.prev_ey) / dt_s;
    }
    s_st.prev_ex = ex;
    s_st.prev_ey = ey;

    vec2_limit(&v_wx, &v_wy, g_odom_nav_goto_tune.vmax_forward);

    /* 世界系：+X 前进、+Y 左；yaw 为从 +X 到车头逆时针为正（顺时针为负）。
     * 车体系：Vy 前后；Vw 横移以右为正。 */
    vy_fwd = cosf(yaw_rad) * v_wx + sinf(yaw_rad) * v_wy;
    vw_str = sinf(yaw_rad) * v_wx - cosf(yaw_rad) * v_wy;

    vy_fwd = clampf(vy_fwd, -g_odom_nav_goto_tune.vmax_forward, g_odom_nav_goto_tune.vmax_forward);
    vw_str = clampf(vw_str, -g_odom_nav_goto_tune.vmax_strafe, g_odom_nav_goto_tune.vmax_strafe);

    if (status != NULL)
    {
        status->vy_cmd = vy_fwd;
        status->vw_cmd = vw_str;
    }

    odom_nav_goto_apply_wheel_inputs(vy_fwd, vw_str);

    return ODOM_NAV_GOTO_ERR_OK_MOVING;
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
            Process_Flow_ClearChassisOverride();
            odom_nav_goto_clear_state();
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
    }

    s_armed = 1U;

    {
        odom_nav_goto_target_t tgt;

        tgt.x_m = g_odom_nav_goto_dbg.target_x_m;
        tgt.y_m = g_odom_nav_goto_dbg.target_y_m;
        tgt.session_id = s_session;
        g_odom_nav_goto_dbg.last_run_return = (uint32_t)odom_nav_goto_run(&tgt, NULL);
    }
}
#endif
