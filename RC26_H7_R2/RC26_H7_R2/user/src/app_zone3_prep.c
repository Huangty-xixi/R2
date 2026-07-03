/**
 * @file app_zone3_prep.c
 * @brief 三区技能赛预备阶段实现
 *
 * 流程：等R1上坡 -> 自己上坡 -> 取KFS2 -> 取KFS3 -> 导航到出口 -> 交棒Zone3
 * 前提：遥控模式下吸盘2已吸好第1个KFS
 */
#include "app_zone3_prep.h"
#include "app_zone3.h"
#include "app_init.h"

#include "Process_Flow.h"
#include "Motion_Task.h"
#include "odom_nav_goto.h"
#include "yaw_heading_ctrl.h"
#include "kfs.h"

#include "cmsis_os.h"

#include <string.h>

typedef struct {
    app_zone3_prep_state_t state;
    uint32_t state_enter_ms;
    float nav_x_m;
    float nav_y_m;
    uint32_t nav_session_id;
    uint8_t kfs_sent;     // 已调用Process_GetKFS标志
    uint8_t active;
    uint8_t done;
    uint8_t failed;
} app_zone3_prep_ctx_t;

static app_zone3_prep_ctx_t g_prep;

static void app_zone3_prep_enter_state(app_zone3_prep_state_t st, uint32_t now_ms)
{
    g_prep.state = st;
    g_prep.state_enter_ms = now_ms;
}

static void app_zone3_prep_clear_motion(void)
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_disarm();
}

static void app_zone3_prep_begin_nav(float x_m, float y_m,
                                      app_zone3_prep_state_t nav_state,
                                      uint32_t now_ms)
{
    app_zone3_prep_clear_motion();
    odom_nav_goto_set_target(x_m, y_m);
    g_prep.nav_x_m = x_m;
    g_prep.nav_y_m = y_m;
    g_prep.nav_session_id = odom_nav_target.session_id;
    app_zone3_prep_enter_state(nav_state, now_ms);
}

static odom_nav_goto_err_t app_zone3_prep_nav_peek(void)
{
    odom_nav_goto_err_t nav_rc = odom_nav_goto_peek_last_run_result();

    if (g_prep.nav_session_id != odom_nav_target.session_id)
        return ODOM_NAV_GOTO_ERR_DISARMED;
    return nav_rc;
}

static uint8_t app_zone3_prep_nav_failed(odom_nav_goto_err_t nav_rc,
                                          uint32_t now_ms,
                                          uint32_t timeout_ms)
{
    if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
        (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
        (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) ||
        (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED) ||
        ((now_ms - g_prep.state_enter_ms) > timeout_ms))
    {
        app_zone3_prep_clear_motion();
        flow_mode = flow_none;
        g_prep.failed = 1U;
        g_prep.active = 0U;
        app_zone3_prep_enter_state(app_zone3_prep_state_failed, now_ms);
        return 1U;
    }
    return 0U;
}

void AppZone3Prep_Init(void)
{
    AppZone3Prep_Reset();
}

void AppZone3Prep_Start(void)
{
    Process_Flow_ResetAll();
    g_prep.active = 1U;
    g_prep.done = 0U;
    g_prep.failed = 0U;
    g_prep.nav_session_id = 0U;
    flow_mode = flow_none;
    app_zone3_prep_enter_state(app_zone3_prep_state_wait_r1_upslope,
                                osKernelGetTickCount());
}

void AppZone3Prep_Reset(void)
{
    app_zone3_prep_clear_motion();
    flow_mode = flow_none;

    g_prep.state = app_zone3_prep_state_idle;
    g_prep.state_enter_ms = 0U;
    g_prep.nav_session_id = 0U;
    g_prep.active = 0U;
    g_prep.done = 0U;
    g_prep.failed = 0U;
}

uint8_t AppZone3Prep_IsActive(void) { return g_prep.active; }
uint8_t AppZone3Prep_IsDone(void)   { return g_prep.done; }
uint8_t AppZone3Prep_IsFailed(void)  { return g_prep.failed; }

void AppZone3Prep_Run(void)
{
    uint32_t now_ms;
    odom_nav_goto_err_t nav_rc;

    if (control_mode != full_auto_control)
    {
        AppZone3Prep_Reset();
        return;
    }

    if (g_prep.active == 0U || g_prep.state == app_zone3_prep_state_idle)
        return;

    now_ms = osKernelGetTickCount();

    switch (g_prep.state)
    {
        case app_zone3_prep_state_wait_r1_upslope:
            /* 等待R1先上坡（吸盘2已有KFS#1） */
            if ((now_ms - g_prep.state_enter_ms) >= APP_Z3_PREP_WAIT_R1_MS)
            {
                Process_UpSlope_Reset();
                g_process_upslope_tune.p1_x_m = PROCESS_UPSLOPE_P1_X_M;
                g_process_upslope_tune.p1_y_m = PROCESS_UPSLOPE_P1_Y_M;
                Process_UpSlope();
                app_zone3_prep_enter_state(app_zone3_prep_state_upslope, now_ms);
            }
            break;

	        case app_zone3_prep_state_upslope:
	            Process_UpSlope();
	            if (Process_UpSlope_IsBusy() == 0U)
	            {
	                flow_mode = flow_none;
	                app_zone3_prep_begin_nav(g_app_zone3_cfg.g1_x_m,
	                                          g_app_zone3_cfg.g1_y_m,
	                                          app_zone3_prep_state_nav_to_g1,
	                                          now_ms);
	            }
	            break;

	        case app_zone3_prep_state_nav_to_g1:
	            nav_rc = app_zone3_prep_nav_peek();
	            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
	            {
	                app_zone3_prep_clear_motion();
	                g_prep.kfs_sent = 0U;
	                app_zone3_prep_enter_state(app_zone3_prep_state_get_kfs_g1, now_ms);
	            }
	            else
	            {
	                (void)app_zone3_prep_nav_failed(nav_rc, now_ms, 30000U);
	            }
	            break;

	        case app_zone3_prep_state_get_kfs_g1:
	            if (g_prep.kfs_sent == 0U)
	            {
	                kfs_spin_position = kfs_spin_p2;
	                YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_FRONT);
	                Process_GetKFS(APP_ZONE2_GET_KFS_GROUND);
	                g_prep.kfs_sent = 1U;
	                break;
	            }
	            Process_GetKFS(APP_ZONE2_GET_KFS_GROUND);
	            if (Process_GetKFS_IsChassisForwardDone())
	            {
	                Process_Flow_ClearChassisOverrideAxes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY | PROCESS_FLOW_CHASSIS_OVERRIDE_VW));
	                g_prep.kfs_sent = 0U;
	                app_zone3_prep_begin_nav(g_app_zone3_cfg.g2_x_m,
	                                          g_app_zone3_cfg.g2_y_m,
	                                          app_zone3_prep_state_nav_to_g2,
	                                          now_ms);
	            }
	            else if (Process_GetKFS_IsBusy() == 0U)
	            {
	                g_prep.kfs_sent = 0U;
	                app_zone3_prep_begin_nav(g_app_zone3_cfg.g2_x_m,
	                                          g_app_zone3_cfg.g2_y_m,
	                                          app_zone3_prep_state_nav_to_g2,
	                                          now_ms);
	            }
	            break;

	        case app_zone3_prep_state_nav_to_g2:
	            nav_rc = app_zone3_prep_nav_peek();
	            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
	            {
	                app_zone3_prep_clear_motion();
	                g_prep.kfs_sent = 0U;
	                app_zone3_prep_enter_state(app_zone3_prep_state_get_kfs_g2, now_ms);
	            }
	            else
	            {
	                (void)app_zone3_prep_nav_failed(nav_rc, now_ms, 30000U);
	            }
	            break;

	        case app_zone3_prep_state_get_kfs_g2:
	            if (g_prep.kfs_sent == 0U)
	            {
	                kfs_spin_position = kfs_spin_p2;
	                YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_FRONT);
	                Process_GetKFS(APP_ZONE2_GET_KFS_GROUND);
	                g_prep.kfs_sent = 1U;
	                break;
	            }
	            Process_GetKFS(APP_ZONE2_GET_KFS_GROUND);
	            if (Process_GetKFS_IsChassisForwardDone())
	            {
	                Process_Flow_ClearChassisOverrideAxes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY | PROCESS_FLOW_CHASSIS_OVERRIDE_VW));
	                g_prep.kfs_sent = 0U;
	                Process_Flow_ResetAll(); /* 砍尾巴: get_kfs→idle, 清底盘, disarm */
	                g_prep.done = 1U;
	                g_prep.active = 0U;
	                app_zone3_prep_enter_state(app_zone3_prep_state_done, now_ms);
	                AppZone3_Start();
	            }
	            else if (Process_GetKFS_IsBusy() == 0U)
	            {
	                three_kfs_position = three_kfs_p2; /* 尾巴跑完了(p3), 拉回p2 */
	                g_prep.kfs_sent = 0U;
	                Process_Flow_ResetAll();
	                g_prep.done = 1U;
	                g_prep.active = 0U;
	                app_zone3_prep_enter_state(app_zone3_prep_state_done, now_ms);
	                AppZone3_Start();
	            }
	            break;

        case app_zone3_prep_state_done:
        case app_zone3_prep_state_failed:
        case app_zone3_prep_state_idle:
        default:
            break;
    }
}
