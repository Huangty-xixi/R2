/**
 * @file odom_nav_goto.c
 * @brief @ref odom_nav_goto.h：世界系 PI(D) → 旋到车体系 Vy/Vw，Vx=0。
 */
#include "odom_nav_goto.h"

#include "common.h"
#include "Process_Flow.h"
#include "upper_pc_protocol.h"

#include <math.h>
#include <string.h>

volatile odom_nav_goto_tune_t g_odom_nav_goto_tune = {
    .kp_xy = 2.0f,
    .ki_xy = 0.1f,
    .kd_xy = 0.5f,
    .vmax_forward = 80.0f,
    .vmax_strafe = 80.0f,
    .position_tolerance_m = 0.15f,
    .timeout_ms = 120000U,
    .i_xy_limit = 2.0f,
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
    process_flow_chassis_override.axis_mask = (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VX | PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                         PROCESS_FLOW_CHASSIS_OVERRIDE_VW);
    process_flow_chassis_override.vx = 0.0f;
    process_flow_chassis_override.vy = vy_forward;
    process_flow_chassis_override.vw = vw_strafe;
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
        return ODOM_NAV_GOTO_ERR_NULL_POINTER;
    }

    if (!odom_nav_goto_validate_tune())
    {
        return ODOM_NAV_GOTO_ERR_BAD_CONFIG;
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
        Process_Flow_ClearChassisOverride();
        if (status != NULL)
        {
            (void)memset(status, 0, sizeof(*status));
        }
        return ODOM_NAV_GOTO_ERR_TIMEOUT;
    }

    x_m = 0.0f;
    y_m = 0.0f;
    yaw_deg = 0.0f;
    pose_rc = odom_nav_goto_read_pose(&x_m, &y_m, &yaw_deg);
    if (pose_rc != 0)
    {
        return ODOM_NAV_GOTO_ERR_ODOM_READ;
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
        Process_Flow_ClearChassisOverride();
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

    vy_fwd = cosf(yaw_rad) * v_wy + sinf(yaw_rad) * v_wx;
    vw_str = -sinf(yaw_rad) * v_wy + cosf(yaw_rad) * v_wx;

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
