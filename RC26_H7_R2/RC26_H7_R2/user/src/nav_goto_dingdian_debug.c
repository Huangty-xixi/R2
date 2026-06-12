/**
 * @file nav_goto_dingdian_debug.c
 * @brief 定点导航调试（见 @ref nav_goto_dingdian_debug.h）
 */
#include "nav_goto_dingdian_debug.h"

#include "Motion_Task.h"
#include "odom_center_offset.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"
#include "app_zone2.h"
#include "yaw_heading_ctrl.h"

#include <stdint.h>

#ifndef NAV_GOTO_DINGDIAN_STEP_M
#define NAV_GOTO_DINGDIAN_STEP_M (1.0f)
#endif

volatile nav_goto_dingdian_debug_t g_nav_goto_dingdian_debug = {
    .a = 0U,
    .b = 0U,
};

typedef enum {
    dingdian_mode_none = 0,
    dingdian_mode_a = 1,
    dingdian_mode_b = 2,
} dingdian_mode_t;

typedef struct {
    uint8_t active;
    dingdian_mode_t mode;
    uint8_t leg;
    float anchor_x_m;
    float anchor_y_m;
} dingdian_run_t;

static dingdian_run_t s_run = {0U, dingdian_mode_none, 0U, 0.0f, 0.0f};

static uint8_t dingdian_mode_gate_ok(void)
{
    return (uint8_t)((control_mode == full_auto_control) && (flow_mode == flow_none) &&
                     (app_flow_mode == app_flow_none));
}

static int dingdian_read_center_xy(float *x_m, float *y_m)
{
    const rc_odom_t *p;

    if (rc_odom_is_valid() == 0U)
    {
        return -1;
    }
    p = rc_get_latest_odom();
    odom_center_offset_odom_to_center(p, x_m, y_m);
    return 0;
}

static const app_zone2_field_dir_t s_yaw_square_dir[4] = {
    APP_ZONE2_FIELD_LEFT,   /* 第1段：场左 */
    APP_ZONE2_FIELD_BACK,   /* 第2段：场后 */
    APP_ZONE2_FIELD_RIGHT,  /* 第3段：场右 */
    APP_ZONE2_FIELD_FRONT,  /* 第4段：场前(回原点) */
};

static void dingdian_leg_target(dingdian_mode_t mode, uint8_t leg, float ax, float ay, float *tx, float *ty)
{
    static const int8_t k_path_a[4][2] = {{1, 0}, {1, 1}, {0, 1}, {0, 0}};
    static const int8_t k_path_b[4][2] = {{1, 1}, {0, 1}, {1, 0}, {0, 0}};
    const int8_t *cell;
    const float step = NAV_GOTO_DINGDIAN_STEP_M;

    if (mode == dingdian_mode_a)
    {
        cell = k_path_a[leg];
    }
    else
    {
        cell = k_path_b[leg];
    }

    *tx = ax + (float)cell[0] * step;
    *ty = ay + (float)cell[1] * step;
}

static void dingdian_abort(void)
{
    odom_nav_goto_disarm();
    YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_FACE_SKIP);
    YawHeadingCtrl_ParallelLegSettleReset();
    s_run.active = 0U;
    s_run.mode = dingdian_mode_none;
    s_run.leg = 0U;
    g_nav_goto_dingdian_debug.a = 0U;
    g_nav_goto_dingdian_debug.b = 0U;
    g_nav_goto_dingdian_debug.c = 0U;
}

static void dingdian_start_leg(uint8_t leg)
{
    float tx;
    float ty;

    dingdian_leg_target(s_run.mode, leg, s_run.anchor_x_m, s_run.anchor_y_m, &tx, &ty);
    odom_nav_goto_set_target(tx, ty);
    if (g_nav_goto_dingdian_debug.c != 0U)
    {
        YawHeadingCtrl_RunFieldDir(s_yaw_square_dir[leg]); /* 导航与转向并发 */
    }
    YawHeadingCtrl_ParallelLegSettleReset();
    s_run.leg = leg;
}

static uint8_t dingdian_nav_leg_finished(void)
{
    const odom_nav_goto_err_t rc = odom_nav_goto_peek_last_run_result();
    const uint8_t nav_done = (uint8_t)((rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) ||
                                       (rc == ODOM_NAV_GOTO_ERR_TIMEOUT));

    if (nav_done == 0U)
    {
        return 0U;
    }
    /* c=1 并发：到点且航向稳态后再切下一段（同 app_zone1） */
    if (g_nav_goto_dingdian_debug.c != 0U)
    {
        return YawHeadingCtrl_ParallelLegSettled();
    }
    return 1U;
}

static void dingdian_try_arm(void)
{
    const uint8_t want_a = (g_nav_goto_dingdian_debug.a != 0U) ? 1U : 0U;
    const uint8_t want_b = (g_nav_goto_dingdian_debug.b != 0U) ? 1U : 0U;
    float ax;
    float ay;

    if (want_a != 0U && want_b != 0U)
    {
        return;
    }
    if (want_a == 0U && want_b == 0U)
    {
        return;
    }
    if (dingdian_read_center_xy(&ax, &ay) != 0)
    {
        return;
    }

    s_run.active = 1U;
    s_run.anchor_x_m = ax;
    s_run.anchor_y_m = ay;
    s_run.mode = (want_a != 0U) ? dingdian_mode_a : dingdian_mode_b;
    dingdian_start_leg(0U);
}

void nav_goto_dingdian_debug_poll(void)
{
    if (!dingdian_mode_gate_ok())
    {
        if (s_run.active != 0U)
        {
            dingdian_abort();
        }
        return;
    }

    if (g_nav_goto_dingdian_debug.a != 0U && g_nav_goto_dingdian_debug.b != 0U)
    {
        if (s_run.active != 0U)
        {
            dingdian_abort();
        }
        return;
    }

    if (s_run.active == 0U)
    {
        dingdian_try_arm();
        return;
    }

    if (dingdian_nav_leg_finished() == 0U)
    {
        return;
    }

    if (s_run.leg >= 3U)
    {
        dingdian_abort();
        return;
    }

    dingdian_start_leg((uint8_t)(s_run.leg + 1U));
}
