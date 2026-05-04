/**
 * @file rc_nav_goto.c
 * @brief @ref rc_nav_goto.h 的实现：位姿闭环、世界系速度解算、经回调输出底盘三轴指令
 * @date&author 2026/5/4 Hty
 */
#include "rc_nav_goto.h"

#include <math.h>
#include <string.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

typedef struct {
    uint32_t magic;//魔数，用于校验工作区是否合法
    uint32_t last_session;//上次目标点 session_id
    uint32_t t0_ms;//开始时间戳
    uint32_t last_ms;//上次时间戳
    float prev_ex;//上次误差
    float prev_ey;//上次误差
    float prev_yaw_err;//上次航向误差
    float ix;//积分误差
    float iy;//积分误差
    float iyaw;//积分航向误差
} rc_nav_goto_impl_t;

#define RC_NAV_GOTO_MAGIC 0x4E415600u /* "NAV\0" */

_Static_assert(sizeof(rc_nav_goto_impl_t) <= RC_NAV_GOTO_WORKSPACE_SIZE,
               "RcNavGoto_Workspace too small for impl");

/**
 * @brief 将对外不透明工作区指针转为内部实现结构指针（零拷贝，同一片内存）
 * @param ws：不透明工作区指针
 * @return 内部 @c rc_nav_goto_impl_t 指针
 * @date&author 2026/5/4 Hty
 */
static rc_nav_goto_impl_t *ws_to_impl(RcNavGoto_Workspace *ws)
{
    return (rc_nav_goto_impl_t *)(void *)ws->opaque;
}

/**
 * @brief 将任意角度（度）折叠到约 [-180, 180] 主值区间，用于航向误差计算
 * @param deg：输入角度（度）
 * @return 折叠后的角度（度）
 * @date&author 2026/5/4 Hty
 */
static float wrap_deg_180(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

/**
 * @brief 校验 @ref RcNavGoto_Config 必填回调与增益、限幅、容差、超时是否合法
 * @param cfg：配置指针（仅由 @ref RcNavGoto_Run 在指针判空通过后调用）
 * @return 1 合法；0 非法（函数指针缺失或参数越出允许范围）
 * @date&author 2026/5/4 Hty
 */
static int validate_config(const RcNavGoto_Config *cfg)
{
    if (cfg->get_time_ms == NULL || cfg->read_pose == NULL || cfg->apply_wheel_inputs == NULL ||
        cfg->apply_wheel_stop == NULL)
    {
        return 0;
    }
    if (cfg->kp_xy < 0.0f || cfg->ki_xy < 0.0f || cfg->kd_xy < 0.0f)
    {
        return 0;
    }
    if (cfg->kp_yaw < 0.0f || cfg->ki_yaw < 0.0f || cfg->kd_yaw < 0.0f)
    {
        return 0;
    }
    if (cfg->vmax_forward <= 0.0f || cfg->vmax_strafe <= 0.0f || cfg->vmax_rotation <= 0.0f)
    {
        return 0;
    }
    if (cfg->position_tolerance_m <= 0.0f || cfg->yaw_tolerance_deg <= 0.0f)
    {
        return 0;
    }
    if (cfg->timeout_ms == 0u)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 将二维速度矢量模长限制在 @a vmax 以内（超过则等比例缩放）
 * @param vx：指向世界系 X 方向速度分量（输入输出）
 * @param vy：指向世界系 Y 方向速度分量（输入输出）
 * @param vmax：允许的最大合速度模长
 * @date&author 2026/5/4 Hty
 */
static void vec2_limit(float *vx, float *vy, float vmax)
{
    const float mag = sqrtf((*vx) * (*vx) + (*vy) * (*vy));
    if (mag > vmax && mag > 1e-6f)
    {
        const float s = vmax / mag;
        *vx *= s;
        *vy *= s;
    }
}

/**
 * @brief 浮点限幅到闭区间 [lo, hi]
 * @param v：待限幅值
 * @param lo：下限
 * @param hi：上限（需满足 lo <= hi）
 * @return 限幅后的值
 * @date&author 2026/5/4 Hty
 */
static float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/**
 * @brief 单周期点到点导航：读位姿、算误差、PI(D) 输出 Vy/Vw/Vx，经 cfg 回调下发或停车
 * @param ws：不透明工作区（首次使用前清零；session_id 变化时内部重置积分与超时起点）
 * @param cfg：平台钩子与 PID/限幅/容差/超时配置
 * @param target：目标位置（米）与可选终端航向（度）、lock_final_yaw、session_id
 * @param status：可选输出本拍距离与航向误差；可为 NULL
 * @return 见 @ref RcNavGoto_Error（到达、运动中、参数/里程计/超时等）
 * @date&author 2026/5/4 Hty
 */
RcNavGoto_Error RcNavGoto_Run(RcNavGoto_Workspace *ws,
                              const RcNavGoto_Config *cfg,
                              const RcNavGoto_Target *target,
                              RcNavGoto_Status *status)
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
    float vx_rot;
    float heading_goal_deg;
    float yaw_err;
    uint8_t xy_done;
    uint8_t yaw_done;
    rc_nav_goto_impl_t *im;

    if (ws == NULL || cfg == NULL || target == NULL)
    {
        return RC_NAV_GOTO_ERR_NULL_POINTER;
    }

    if (!validate_config(cfg))
    {
        return RC_NAV_GOTO_ERR_BAD_CONFIG;
    }

    im = ws_to_impl(ws);
    if (im->magic != RC_NAV_GOTO_MAGIC)
    {
        (void)memset(ws->opaque, 0, sizeof(ws->opaque));
        im->magic = RC_NAV_GOTO_MAGIC;
        im->last_session = 0xFFFFFFFFu;
    }

    if (im->last_session != target->session_id)
    {
        im->last_session = target->session_id;
        im->t0_ms = cfg->get_time_ms(cfg->platform_user);
        im->last_ms = im->t0_ms;
        im->prev_ex = 0.0f;
        im->prev_ey = 0.0f;
        im->prev_yaw_err = 0.0f;
        im->ix = 0.0f;
        im->iy = 0.0f;
        im->iyaw = 0.0f;
    }

    now_ms = cfg->get_time_ms(cfg->platform_user);
    dt_s = (float)((int32_t)(now_ms - im->last_ms)) * 0.001f;
    if (dt_s < 1e-4f || dt_s > 0.5f)
    {
        dt_s = 0.02f;
    }
    im->last_ms = now_ms;

    if ((now_ms - im->t0_ms) >= cfg->timeout_ms)
    {
        cfg->apply_wheel_stop(cfg->platform_user);
        if (status != NULL)
        {
            (void)memset(status, 0, sizeof(*status));
        }
        return RC_NAV_GOTO_ERR_TIMEOUT;
    }

    x_m = 0.0f;
    y_m = 0.0f;
    yaw_deg = 0.0f;
    pose_rc = cfg->read_pose(cfg->platform_user, &x_m, &y_m, &yaw_deg);
    if (pose_rc != 0)
    {
        return RC_NAV_GOTO_ERR_ODOM_READ;
    }

    ex = target->x_m - x_m;
    ey = target->y_m - y_m;
    dist = sqrtf(ex * ex + ey * ey);

    yaw_rad = yaw_deg * (M_PI_F / 180.0f);

    xy_done = (dist <= cfg->position_tolerance_m) ? 1u : 0u;

    if (xy_done == 0u)
    {
        heading_goal_deg = atan2f(ey, ex) * (180.0f / M_PI_F);
    }
    else if (target->lock_final_yaw != 0u)
    {
        heading_goal_deg = target->yaw_deg;
    }
    else
    {
        heading_goal_deg = yaw_deg;
    }

    yaw_err = wrap_deg_180(heading_goal_deg - yaw_deg);
    yaw_done = (fabsf(yaw_err) <= cfg->yaw_tolerance_deg) ? 1u : 0u;

    if (status != NULL)
    {
        status->distance_to_target_m = dist;
        status->yaw_error_deg = yaw_err;
        status->at_xy = xy_done;
        status->at_yaw = yaw_done;
    }

    if (xy_done != 0u && (target->lock_final_yaw == 0u || yaw_done != 0u))
    {
        cfg->apply_wheel_stop(cfg->platform_user);
        return RC_NAV_GOTO_ERR_OK_ARRIVED;
    }

    im->ix += ex * dt_s;
    im->iy += ey * dt_s;
    im->iyaw += yaw_err * dt_s;

    im->ix = clampf(im->ix, -2.0f, 2.0f);
    im->iy = clampf(im->iy, -2.0f, 2.0f);
    im->iyaw = clampf(im->iyaw, -30.0f, 30.0f);

    v_wx = cfg->kp_xy * ex + cfg->ki_xy * im->ix;
    v_wy = cfg->kp_xy * ey + cfg->ki_xy * im->iy;
    if (cfg->kd_xy > 0.0f)
    {
        v_wx += cfg->kd_xy * (ex - im->prev_ex) / dt_s;
        v_wy += cfg->kd_xy * (ey - im->prev_ey) / dt_s;
    }
    im->prev_ex = ex;
    im->prev_ey = ey;

    vec2_limit(&v_wx, &v_wy, cfg->vmax_forward);

    vy_fwd = cosf(yaw_rad) * v_wy + sinf(yaw_rad) * v_wx;
    vw_str = -sinf(yaw_rad) * v_wy + cosf(yaw_rad) * v_wx;

    vy_fwd = clampf(vy_fwd, -cfg->vmax_forward, cfg->vmax_forward);
    vw_str = clampf(vw_str, -cfg->vmax_strafe, cfg->vmax_strafe);

    vx_rot = cfg->kp_yaw * yaw_err + cfg->ki_yaw * im->iyaw;
    if (cfg->kd_yaw > 0.0f)
    {
        vx_rot += cfg->kd_yaw * (yaw_err - im->prev_yaw_err) / dt_s;
    }
    im->prev_yaw_err = yaw_err;

    vx_rot = clampf(vx_rot, -cfg->vmax_rotation, cfg->vmax_rotation);

    if (xy_done != 0u && target->lock_final_yaw != 0u)
    {
        vy_fwd = 0.0f;
        vw_str = 0.0f;
    }

    cfg->apply_wheel_inputs(cfg->platform_user, vx_rot, vy_fwd, vw_str);

    return RC_NAV_GOTO_ERR_OK_MOVING;
}
