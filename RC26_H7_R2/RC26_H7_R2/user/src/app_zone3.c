/**
 * @file app_zone3.c
 */

#include "app_zone3.h"

#include "Process_Flow.h"
#include "Motion_Task.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"
#include "kfs.h"

#include "cmsis_os.h"

#include <string.h>

volatile AppZone3Config g_app_zone3_cfg = {
    .p1_x_m = 0.0f,
    .p1_y_m = 0.0f,
    .p2_x_m = 1.0f,
    .p2_y_m = 0.0f,
    .p3_x_m = 2.0f,
    .p3_y_m = 0.0f,
    .p4_x_m = 3.0f,
    .p4_y_m = 0.0f,
    .up_r1_delay_ms = 5000U,
    .nav_timeout_ms = 30000U,
    .action_timeout_ms = 60000U,
};

typedef enum
{
    app_zone3_state_idle = 0,   // 空闲
    app_zone3_state_nav,        // 导航
    app_zone3_state_put_kfs,    // 放卡夫曼
    app_zone3_state_up_r1_delay, // 上R1延迟
    app_zone3_state_up_r1_run,  // 上R1运行
    app_zone3_state_up_r1_lift_p3, /* 上R1后主轴抬升到 p3 */
    app_zone3_state_stop_nav,   // 停止导航
    app_zone3_state_done,       // 完成
    app_zone3_state_failed,     // 失败
} app_zone3_state_t;

typedef struct
{
    app_zone3_state_t state;
    app_zone3_cmd_id_t active_cmd;
    uint32_t state_enter_ms;
    float nav_x_m;
    float nav_y_m;
    uint32_t nav_session_id;
    uint8_t on_r1;
    uint8_t active;
    uint8_t done;
    uint8_t failed;
    volatile uint8_t cmd_pending;
    volatile uint8_t stop_pending;
    app_zone3_r1_cmd_t pending_cmd;
} app_zone3_ctx_t;

static app_zone3_ctx_t g_z3;

static void app_zone3_clear_motion(void)
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_disarm();
}

static void app_zone3_enter_state(app_zone3_state_t st, uint32_t now_ms)
{
    g_z3.state = st;
    g_z3.state_enter_ms = now_ms;
}

static void app_zone3_get_point(app_zone3_cmd_id_t id, float *x_m, float *y_m)
{
    if (x_m == NULL || y_m == NULL)
    {
        return;
    }

    switch (id)
    {
        case APP_Z3_CMD_PUT_KFS_P2: // 放2层左 导航点2
            *x_m = g_app_zone3_cfg.p2_x_m;  
            *y_m = g_app_zone3_cfg.p2_y_m;
            break;
        case APP_Z3_CMD_PUT_KFS_P3: // 放2层中 导航点3
            *x_m = g_app_zone3_cfg.p3_x_m;
            *y_m = g_app_zone3_cfg.p3_y_m;
            break;
        case APP_Z3_CMD_PUT_KFS_P4: // 放2层右 导航点4
            *x_m = g_app_zone3_cfg.p4_x_m;
            *y_m = g_app_zone3_cfg.p4_y_m;
            break;
        default: // 无效指令 导航点1
            *x_m = g_app_zone3_cfg.p1_x_m;
            *y_m = g_app_zone3_cfg.p1_y_m;
            break;
    }
}

static void app_zone3_begin_nav(float x_m, float y_m, uint32_t now_ms)
{
    app_zone3_clear_motion();
    odom_nav_goto_set_target(x_m, y_m);
    g_z3.nav_x_m = x_m;
    g_z3.nav_y_m = y_m;
    g_z3.nav_session_id = odom_nav_target.session_id;
    app_zone3_enter_state(app_zone3_state_nav, now_ms);
}

static odom_nav_goto_err_t app_zone3_nav_peek(void)
{
    odom_nav_goto_err_t nav_rc = odom_nav_goto_peek_last_run_result();

    if (g_z3.nav_session_id != odom_nav_target.session_id)
    {
        return ODOM_NAV_GOTO_ERR_DISARMED;
    }
    return nav_rc;
}

static void app_zone3_begin_stop(uint32_t now_ms)
{
    g_z3.active_cmd = APP_Z3_CMD_STOP_ACTION;
    flow_mode = flow_none;
    app_zone3_clear_motion();
    app_zone3_begin_nav(g_app_zone3_cfg.p1_x_m, g_app_zone3_cfg.p1_y_m, now_ms);
    g_z3.state = app_zone3_state_stop_nav;
}

static void app_zone3_dispatch_cmd(const app_zone3_r1_cmd_t *cmd, uint32_t now_ms)
{
    float x_m;
    float y_m;

    if (cmd == NULL)
    {
        return;
    }

    g_z3.active = 1U;
    g_z3.done = 0U;
    g_z3.failed = 0U;
    g_z3.active_cmd = cmd->id;

    switch (cmd->id)
    {
        case APP_Z3_CMD_STOP_ACTION:
            app_zone3_begin_stop(now_ms);
            break;

        case APP_Z3_CMD_PUT_KFS_P2: // 放2层左 导航点2
        case APP_Z3_CMD_PUT_KFS_P3: // 放2层中 导航点3
        case APP_Z3_CMD_PUT_KFS_P4: // 放2层右 导航点4
            app_zone3_get_point(cmd->id, &x_m, &y_m);
            app_zone3_begin_nav(x_m, y_m, now_ms);
            break;

        case APP_Z3_CMD_UP_R1: // 上R1
            app_zone3_clear_motion();
            app_zone3_enter_state(app_zone3_state_up_r1_delay, now_ms);
            break;

        case APP_Z3_CMD_PUT_KFS_ON_R1: // 放3层
            if (g_z3.on_r1 != 0U)
            {
                Process_PutKFS();
                g_z3.done = 1U;
                g_z3.active = 0U;
                app_zone3_enter_state(app_zone3_state_done, now_ms);
            }
            else
            {
                app_zone3_get_point(APP_Z3_CMD_PUT_KFS_P4, &x_m, &y_m);
                app_zone3_begin_nav(x_m, y_m, now_ms);
            }
            break;

        default:
            break;
    }
}

void AppZone3_Init(void)
{
    AppZone3_Reset();
}

void AppZone3_Reset(void) // 重置
{
    __disable_irq();
    g_z3.cmd_pending = 0U;
    g_z3.stop_pending = 0U;
    (void)memset(&g_z3.pending_cmd, 0, sizeof(g_z3.pending_cmd));
    __enable_irq();

    app_zone3_clear_motion();
    flow_mode = flow_none;

    g_z3.state = app_zone3_state_idle;
    g_z3.active_cmd = APP_Z3_CMD_NONE;
    g_z3.state_enter_ms = 0U;
    g_z3.nav_session_id = 0U;
    g_z3.on_r1 = 0U;
    g_z3.active = 0U;
    g_z3.done = 0U;
    g_z3.failed = 0U;
}

void AppZone3_PostR1Cmd(const app_zone3_r1_cmd_t *cmd)
{
    if (cmd == NULL || cmd->id == APP_Z3_CMD_NONE)
    {
        return;
    }

    __disable_irq();
    g_z3.pending_cmd = *cmd;
    if (cmd->id == APP_Z3_CMD_STOP_ACTION)
    {
        g_z3.stop_pending = 1U;
        g_z3.cmd_pending = 0U;
    }
    else
    {
        g_z3.cmd_pending = 1U;
    }
    __enable_irq();
}

uint8_t AppZone3_IsActive(void)
{
    return g_z3.active;
}

uint8_t AppZone3_IsDone(void)
{
    return g_z3.done;
}

uint8_t AppZone3_IsFailed(void)
{
    return g_z3.failed;
}

uint8_t AppZone3_IsOnR1(void)
{
    return g_z3.on_r1;
}

void AppZone3_Run(void)
{
    uint32_t now_ms;
    odom_nav_goto_err_t nav_rc;
    app_zone3_r1_cmd_t cmd;
    uint8_t do_stop = 0U;
    uint8_t do_cmd = 0U;

    if (rc_odom_is_valid() == 0U)
    {
        if (g_z3.state == app_zone3_state_nav ||
            g_z3.state == app_zone3_state_stop_nav)
        {
            app_zone3_clear_motion();
            g_z3.failed = 1U;
            g_z3.active = 0U;
            g_z3.state = app_zone3_state_failed;
        }
    }

    __disable_irq();
    if (g_z3.stop_pending != 0U)
    {
        do_stop = 1U;
        cmd = g_z3.pending_cmd;
        g_z3.stop_pending = 0U;
        g_z3.cmd_pending = 0U;
    }
    else if (g_z3.cmd_pending != 0U)
    {
        do_cmd = 1U;
        cmd = g_z3.pending_cmd;
        g_z3.cmd_pending = 0U;
    }
    __enable_irq();

    now_ms = osKernelGetTickCount();

    if (do_stop != 0U)
    {
        app_zone3_begin_stop(now_ms);
        return;
    }

    if (do_cmd != 0U)
    {
        app_zone3_dispatch_cmd(&cmd, now_ms);
    }

    if (g_z3.active == 0U)
    {
        return;
    }

    switch (g_z3.state)
    {
        case app_zone3_state_nav:
        case app_zone3_state_stop_nav:
            nav_rc = app_zone3_nav_peek();
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                app_zone3_clear_motion();
                if (g_z3.state == app_zone3_state_stop_nav)
                {
                    g_z3.active = 0U;
                    g_z3.done = 1U;
                    app_zone3_enter_state(app_zone3_state_done, now_ms);
                    break;
                }
                if (g_z3.active_cmd == APP_Z3_CMD_PUT_KFS_ON_R1 && g_z3.on_r1 == 0U)
                {
                    Process_PutKFS();
                    g_z3.done = 1U;
                    g_z3.active = 0U;
                    app_zone3_enter_state(app_zone3_state_done, now_ms);
                    break;
                }
                app_zone3_enter_state(app_zone3_state_put_kfs, now_ms);
                break;
            }
            if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) ||
                (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED))
            {
                app_zone3_clear_motion();
                g_z3.failed = 1U;
                g_z3.active = 0U;
                app_zone3_enter_state(app_zone3_state_failed, now_ms);
            }
            else if ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.nav_timeout_ms)
            {
                app_zone3_clear_motion();
                g_z3.failed = 1U;
                g_z3.active = 0U;
                app_zone3_enter_state(app_zone3_state_failed, now_ms);
            }
            break;

        case app_zone3_state_put_kfs:
            Process_PutKFS();
            g_z3.done = 1U;
            g_z3.active = 0U;
            app_zone3_enter_state(app_zone3_state_done, now_ms);
            break;

        case app_zone3_state_up_r1_delay:
            if ((now_ms - g_z3.state_enter_ms) >= g_app_zone3_cfg.up_r1_delay_ms)
            {
                flow_mode = flow_upstairs_mode; // 上楼流程
                Process_UpStairs();
                app_zone3_enter_state(app_zone3_state_up_r1_run, now_ms);
            }
            else if ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.action_timeout_ms)
            {
                g_z3.failed = 1U;
                g_z3.active = 0U;
                app_zone3_enter_state(app_zone3_state_failed, now_ms);
            }
            break;

        case app_zone3_state_up_r1_run:
            if (Process_UpStairs_IsBusy() == 0U)
            {
                flow_mode = flow_none;
                g_z3.on_r1 = 1U;
                main_lift_position = main_lift_p3;
                app_zone3_enter_state(app_zone3_state_up_r1_lift_p3, now_ms);
            }
            else if ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.action_timeout_ms)
            {
                flow_mode = flow_none;
                app_zone3_clear_motion();
                g_z3.failed = 1U;
                g_z3.active = 0U;
                app_zone3_enter_state(app_zone3_state_failed, now_ms);
            }
            break;

        case app_zone3_state_up_r1_lift_p3:
            if (Kfs_MainLift_IsAtPosition(main_lift_p3) != 0U)
            {
                g_z3.done = 1U;
                g_z3.active = 0U;
                app_zone3_enter_state(app_zone3_state_done, now_ms);
            }
            else if ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.action_timeout_ms)
            {
                g_z3.failed = 1U;
                g_z3.active = 0U;
                app_zone3_enter_state(app_zone3_state_failed, now_ms);
            }
            break;

        case app_zone3_state_done:
        case app_zone3_state_failed:
        case app_zone3_state_idle:
        default:
            break;
    }
}
