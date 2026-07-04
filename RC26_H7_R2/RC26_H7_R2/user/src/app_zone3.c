/**
 * @file app_zone3.c
 */
 /*
 * === 业务调用链(IR->zone3执行) ===
 * AppZone3_PostR1Cmd(&cmd)              // IR中断内调用，锁存到g_z3
 *   -> STOP: g_z3.stop_pending=1         // 仅zone3激活时接收
 *   -> 非STOP: g_z3.cmd_pending=1        // 仅zone3激活时接收
 *   -> 非STOP且zone3未激活: 静默丢弃
 *
 * AppZone3_Run()                        // Motion_Task每周期调用
 *   -> app_zone3_take_normal_cmd()       // 取出pending指令
 *   -> app_zone3_dispatch_cmd()          // 分发到具体流程
 */

#include "app_zone3.h"

#include "Process_Flow.h"
#include "Motion_Task.h"
#include "yaw_heading_ctrl.h"
#include "kfs.h"
#include "main.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"

#include "cmsis_os.h"

#include <string.h>

/** 上楼结束后 main_lift 到 p4 开环等待(ms)，实车可改 */
#define APP_ZONE3_UP_R1_MAIN_LIFT_WAIT_MS 1500U

/* put_sub Y offset: blue L+y R-y, red L-y R+y */
#define APP_ZONE3_PUT_Y_OFFSET_M  0.05f

volatile AppZone3Config g_app_zone3_cfg = {
    .p1_x_m = 2.42f,
    .p1_y_m = 11.50f,

    .p2_x_m = 0.94f,
    .p2_y_m = 11.3f,

    .p3_x_m = 0.94f,
    .p3_y_m = 10.80f,

    .p4_x_m = 0.94f,
    .p4_y_m = 10.30f,

    .p5_x_m = 3.94f,
    .p5_y_m = 10.99f,

    .g1_x_m = 2.23f,  /* G1 */
    .g1_y_m = 10.65f,

    .g2_x_m = 1.51f,  /* G2 */
    .g2_y_m = 10.65f,
    
    .up_r1_delay_ms = 5000U,
    .nav_timeout_ms = 30000U,
    .action_timeout_ms = 60000U,
};

typedef enum
{
    app_zone3_state_idle = 0,      // 空闲
    app_zone3_state_entry_nav,     // 进入三区，先去点1
    app_zone3_state_wait_r1_cmd,   // 在点1等待R1命令
    app_zone3_state_nav_to_put,    // 去点2/3/4放KFS
    app_zone3_state_put_kfs,       // 放KFS
    app_zone3_state_return_point1, // 普通动作结束后回点1
    app_zone3_state_up_r1_delay,   // 上R1前等待
    app_zone3_state_up_r1_run,     // 上R1运行
    app_zone3_state_up_r1_lift_p4, //
    app_zone3_state_up_r1_climb,   // Process_UpR1 上R1后主轴抬升到p4，开环等待
    app_zone3_state_on_r1_wait_cmd,// 在R1上等待放三层命令
    app_zone3_state_on_r1_put_kfs, // 在R1上直接放KFS
    app_zone3_state_nav_to_g1,     // 导航到取KFS点1(G1)
    app_zone3_state_get_kfs_g1,    // 取第一个地面KFS(G1)
    app_zone3_state_nav_to_g2,     // 导航到取KFS点2(G2)
    app_zone3_state_get_kfs_g2,    // 取第二个地面KFS(G2)
    app_zone3_state_stop_nav,      // STOP后回点1
    app_zone3_state_done,          // 完成
    app_zone3_state_failed,        // 失败
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
    uint8_t up_r1_deferred;  /* entry_nav途中收UP_R1,到P1后执行 */
    uint8_t active;
    uint8_t done;
    uint8_t failed;
    volatile uint8_t cmd_pending;
    volatile uint8_t stop_pending;
    app_zone3_r1_cmd_t pending_cmd;
    uint8_t put_sub;
    uint8_t last_seq_valid;
    uint8_t last_seq;
} app_zone3_ctx_t;

static app_zone3_ctx_t g_z3;

#if APP_ZONE3_DBG_FAKE_CMD
static const app_zone3_cmd_id_t s_dbg_fake_cmd_seq[] = { APP_ZONE3_DBG_FAKE_CMD_SEQ };
static uint8_t  s_dbg_fake_cmd_idx;
static uint32_t s_dbg_fake_cmd_enter_tick;
#endif

static uint32_t app_zone3_irq_save(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void app_zone3_irq_restore(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void app_zone3_clear_pending(void)
{
    uint32_t primask = app_zone3_irq_save();

    g_z3.cmd_pending = 0U;
    g_z3.stop_pending = 0U;
    (void)memset(&g_z3.pending_cmd, 0, sizeof(g_z3.pending_cmd));
    app_zone3_irq_restore(primask);
}

static void app_zone3_clear_motion(void)
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_disarm();
}

static void app_zone3_enter_state(app_zone3_state_t st, uint32_t now_ms)
{
    g_z3.state = st;
    g_z3.state_enter_ms = now_ms;
    /* 回到等待状态时清除 active_cmd，允许同命令再次触发 */
    if (st == app_zone3_state_wait_r1_cmd || st == app_zone3_state_on_r1_wait_cmd)
    {
        g_z3.active_cmd = APP_Z3_CMD_NONE;
    }
}

static uint8_t app_zone3_state_accepts_normal_cmd(app_zone3_state_t st)
{
    return (uint8_t)((st == app_zone3_state_idle) ||
                     (st == app_zone3_state_entry_nav) ||
                     (st == app_zone3_state_wait_r1_cmd) ||
                     (st == app_zone3_state_return_point1) ||
                     (st == app_zone3_state_on_r1_wait_cmd));
}

static uint8_t app_zone3_cmd_is_ground_put(app_zone3_cmd_id_t id)
{
    return (uint8_t)((id == APP_Z3_CMD_PUT_KFS_P2) ||
                     (id == APP_Z3_CMD_PUT_KFS_P3) ||
                     (id == APP_Z3_CMD_PUT_KFS_P4));
}

static uint8_t app_zone3_cmd_is_any_put(app_zone3_cmd_id_t id)
{
    return (uint8_t)(app_zone3_cmd_is_ground_put(id) || (id == APP_Z3_CMD_PUT_KFS_ON_R1));
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
        case APP_Z3_CMD_GET_KFS_G1:
            *x_m = g_app_zone3_cfg.g1_x_m;
            *y_m = g_app_zone3_cfg.g1_y_m;
            break;
        case APP_Z3_CMD_GET_KFS_G2:
            *x_m = g_app_zone3_cfg.g2_x_m;
            *y_m = g_app_zone3_cfg.g2_y_m;
            break;
        default: // 无效命令 默认点1
            *x_m = g_app_zone3_cfg.p1_x_m;
            *y_m = g_app_zone3_cfg.p1_y_m;
            break;
    }
}

static void app_zone3_begin_nav(float x_m, float y_m, app_zone3_state_t nav_state, uint32_t now_ms)
{
    app_zone3_clear_motion();
    odom_nav_goto_set_target(x_m, y_m);
    g_z3.nav_x_m = x_m;
    g_z3.nav_y_m = y_m;
    g_z3.nav_session_id = odom_nav_target.session_id;
    app_zone3_enter_state(nav_state, now_ms);
}

static void app_zone3_begin_nav_keep_vx(float x_m, float y_m, app_zone3_state_t nav_state, uint32_t now_ms)
{
    /* 只换导航目标,不碰chassis override — 保留YawHeadingCtrl的VX */
    odom_nav_goto_set_target(x_m, y_m);
    g_z3.nav_x_m = x_m;
    g_z3.nav_y_m = y_m;
    g_z3.nav_session_id = odom_nav_target.session_id;
    app_zone3_enter_state(nav_state, now_ms);
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

static void app_zone3_start_core(uint32_t now_ms, uint8_t clear_pending)
{
    if (clear_pending != 0U)
    {
        app_zone3_clear_pending();
    }

    /* 释放底盘+清除旧导航，让 prep GetKFS 尾巴继续后台跑 */
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_disarm();
    g_z3.active = 1U;
    g_z3.done = 0U;
    g_z3.failed = 0U;
    g_z3.on_r1 = 0U;
    g_z3.up_r1_deferred = 0U;
    g_z3.active_cmd = APP_Z3_CMD_NONE;
    g_z3.put_sub = R1_LINK_Z3_CMD_PUT_SUB_NONE;
    g_z3.nav_session_id = 0U;
    app_flow_mode = app_flow_zone3;
    /* 进三区预置：spin到P2, lift到P4。three_kfs 不动，由 Process_PutKFS retract 步驱动 */
    kfs_spin_position = kfs_spin_p2;
    main_lift_position = main_lift_p4;
    app_zone3_begin_nav(g_app_zone3_cfg.p1_x_m,
                        g_app_zone3_cfg.p1_y_m,
                        app_zone3_state_entry_nav,
                        now_ms);
    /* entry_nav并行转向:蓝区->FIELD_RIGHT,红区->FIELD_LEFT */
#if APP_ZONE2_RED_SIDE
    YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_LEFT);
#else
    YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_RIGHT);
#endif
}

static void app_zone3_begin_stop(uint32_t now_ms)
{
    Process_Flow_ResetAll();
    app_zone3_clear_pending();
    g_z3.active = 1U;
    g_z3.done = 0U;
    g_z3.failed = 0U;
    g_z3.on_r1 = 0U;
    g_z3.up_r1_deferred = 0U;
    g_z3.active_cmd = APP_Z3_CMD_STOP_ACTION;
    flow_mode = flow_none;
    app_zone3_begin_nav(g_app_zone3_cfg.p1_x_m,
                        g_app_zone3_cfg.p1_y_m,
                        app_zone3_state_stop_nav,
                        now_ms);
}

static void app_zone3_apply_put_y_offset(uint8_t put_sub, float *y_m)
{
    float sign;
    if (put_sub == R1_LINK_Z3_CMD_PUT_SUB_NONE || y_m == NULL)
        return;
    if (put_sub == R1_LINK_Z3_CMD_PUT_SUB_LEFT)
        sign = (APP_ZONE2_RED_SIDE == 0U) ? 1.0f : -1.0f;
    else /* PUT_SUB_RIGHT */
        sign = (APP_ZONE2_RED_SIDE == 0U) ? -1.0f : 1.0f;
    *y_m += sign * APP_ZONE3_PUT_Y_OFFSET_M;
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
            g_z3.put_sub = cmd->put_sub;
            if (g_z3.on_r1 != 0U)
            {
                app_zone3_enter_state(app_zone3_state_on_r1_put_kfs, now_ms);
            }
            else
            {
                app_zone3_get_point(cmd->id, &x_m, &y_m);
                app_zone3_apply_put_y_offset(cmd->put_sub, &y_m);
                app_zone3_begin_nav(x_m, y_m, app_zone3_state_nav_to_put, now_ms);
            }
            break;

        case APP_Z3_CMD_UP_R1: // 上R1
            app_zone3_clear_motion();
            flow_mode = flow_none;
            app_zone3_enter_state(app_zone3_state_up_r1_delay, now_ms);
            break;

        case APP_Z3_CMD_GET_KFS_G1: // 取第一个地面KFS
        case APP_Z3_CMD_GET_KFS_G2: // 取第二个地面KFS
            if (g_z3.on_r1 != 0U)
            {
                break;  /* R1上不支持取地面KFS */
            }
            app_zone3_get_point(cmd->id, &x_m, &y_m);
            /* 预备:转场朝前+主轴到p2 */
            main_lift_position = main_lift_p2;
            YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_FRONT);
            app_zone3_begin_nav(x_m, y_m,
                (cmd->id == APP_Z3_CMD_GET_KFS_G1)
                    ? app_zone3_state_nav_to_g1
                    : app_zone3_state_nav_to_g2,
                now_ms);
            break;

        case APP_Z3_CMD_PUT_KFS_ON_R1: // 放3层(仅R1在位有效)
            if (g_z3.on_r1 != 0U)
            {
                app_zone3_enter_state(app_zone3_state_on_r1_put_kfs, now_ms);
            }
            break;

        default:
            break;
    }
}

static uint8_t app_zone3_take_stop_cmd(app_zone3_r1_cmd_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return 0U;
    }

    primask = app_zone3_irq_save();
    if (g_z3.stop_pending == 0U)
    {
        app_zone3_irq_restore(primask);
        return 0U;
    }
    *out = g_z3.pending_cmd;
    g_z3.stop_pending = 0U;
    g_z3.cmd_pending = 0U;
    app_zone3_irq_restore(primask);
    return 1U;
}

static uint8_t app_zone3_take_normal_cmd(app_zone3_r1_cmd_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return 0U;
    }

    primask = app_zone3_irq_save();
    if (g_z3.cmd_pending == 0U)
    {
        app_zone3_irq_restore(primask);
        return 0U;
    }
    *out = g_z3.pending_cmd;
    g_z3.cmd_pending = 0U;
    app_zone3_irq_restore(primask);
    return 1U;
}

static uint8_t app_zone3_nav_failed(odom_nav_goto_err_t nav_rc, uint32_t now_ms)
{
    if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
        (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
        (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) ||
        (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED) ||
        ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.nav_timeout_ms))
    {
        app_zone3_clear_motion();
        flow_mode = flow_none;
        g_z3.failed = 1U;
        g_z3.active = 0U;
        app_zone3_enter_state(app_zone3_state_failed, now_ms);
        return 1U;
    }

    return 0U;
}

static void app_zone3_run_put_kfs(uint32_t now_ms, app_zone3_state_t done_state)
{
    static uint8_t s_nav_armed = 0U;

    Process_PutKFS();

    /* retract 步开始：底盘已释放，立刻导航回 P1，put 尾巴与导航并行 */
    if (done_state == app_zone3_state_return_point1
        && put_kfs_step == put_kfs_step_retract
        && s_nav_armed == 0U)
    {
        app_zone3_begin_nav(g_app_zone3_cfg.p1_x_m,
                            g_app_zone3_cfg.p1_y_m,
                            app_zone3_state_return_point1,
                            now_ms);
        s_nav_armed = 1U;
    }

    if (AppZone3_PutKFS_IsBusy() != 0U)
    {
        if ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.action_timeout_ms)
        {
            g_z3.failed = 1U;
            g_z3.active = 0U;
            app_zone3_enter_state(app_zone3_state_failed, now_ms);
        }
        return;
    }

    s_nav_armed = 0U;
    if (done_state != app_zone3_state_return_point1)
    {
        app_zone3_enter_state(done_state, now_ms);
    }
}

void AppZone3_Init(void)
{
    AppZone3_Reset();
}

void AppZone3_Start(void)
{
    app_zone3_start_core(osKernelGetTickCount(), 1U);
}

void AppZone3_Reset(void) // 复位
{
    uint32_t primask;

    primask = app_zone3_irq_save();
    g_z3.active = 0U;
    g_z3.cmd_pending = 0U;
    g_z3.stop_pending = 0U;
    (void)memset(&g_z3.pending_cmd, 0, sizeof(g_z3.pending_cmd));
    app_zone3_irq_restore(primask);

    app_zone3_clear_motion();
    flow_mode = flow_none;

    g_z3.state = app_zone3_state_idle;
    g_z3.active_cmd = APP_Z3_CMD_NONE;
    g_z3.put_sub = R1_LINK_Z3_CMD_PUT_SUB_NONE;
    g_z3.state_enter_ms = 0U;
    g_z3.nav_session_id = 0U;
    g_z3.on_r1 = 0U;
    g_z3.up_r1_deferred = 0U;
    g_z3.done = 0U;
    g_z3.failed = 0U;
    g_z3.last_seq_valid = 0U;
    g_z3.last_seq = 0U;
#if APP_ZONE3_DBG_FAKE_CMD
    s_dbg_fake_cmd_idx = 0U;
    s_dbg_fake_cmd_enter_tick = 0U;
#endif
}

void AppZone3_PostR1Cmd(const app_zone3_r1_cmd_t *cmd)
{
    uint32_t primask;

    if (cmd == NULL || cmd->id == APP_Z3_CMD_NONE)
    {
        return;
    }

    primask = app_zone3_irq_save();
    if (cmd->id == APP_Z3_CMD_STOP_ACTION)
    {
        if (g_z3.active != 0U)
        {
            g_z3.pending_cmd = *cmd;
            g_z3.stop_pending = 1U;
            g_z3.cmd_pending = 0U;
        }
        app_zone3_irq_restore(primask);
        return;
    }

    if (cmd->seq != 0U && g_z3.last_seq_valid != 0U && cmd->seq == g_z3.last_seq)
    {
        app_zone3_irq_restore(primask);
        return;
    }

    /* 同命令去重：已执行过的 cmd_id 不再重复受理，直到收到不同命令才放行 */
    if (g_z3.active_cmd != APP_Z3_CMD_NONE && cmd->id == g_z3.active_cmd)
    {
        app_zone3_irq_restore(primask);
        return;
    }

    if (g_z3.active == 0U)
    {
        app_zone3_irq_restore(primask);
        return;
    }

    if (app_zone3_state_accepts_normal_cmd(g_z3.state) != 0U)
    {
        g_z3.pending_cmd = *cmd;
        g_z3.cmd_pending = 1U;
        if (cmd->seq != 0U)
        {
            g_z3.last_seq = cmd->seq;
            g_z3.last_seq_valid = 1U;
        }
    }
    app_zone3_irq_restore(primask);
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

    if (control_mode != full_auto_control)
    {
        AppZone3_Reset();
        return;
    }

    now_ms = osKernelGetTickCount();

    if (g_z3.active == 0U)
    {
        if (app_flow_mode == app_flow_zone3)
        {
            app_zone3_start_core(now_ms, 1U);
        }
        return;
    }

    if (g_z3.state == app_zone3_state_idle)
    {
        app_zone3_start_core(now_ms, 0U);
        return;
    }

    if (app_zone3_take_stop_cmd(&cmd) != 0U)
    {
        if (g_z3.on_r1 != 0U)
        {
            Process_PutKFS_AbortAndRollback();
            app_zone3_clear_pending();
            flow_mode = flow_none;
            app_zone3_enter_state(app_zone3_state_on_r1_wait_cmd, now_ms);
        }
        else
        {
            app_zone3_begin_stop(now_ms);
        }
        return;
    }

    if (rc_odom_is_valid() == 0U)
    {
        if (g_z3.state == app_zone3_state_entry_nav ||
            g_z3.state == app_zone3_state_nav_to_put ||
            g_z3.state == app_zone3_state_nav_to_g1 ||
            g_z3.state == app_zone3_state_nav_to_g2 ||
            g_z3.state == app_zone3_state_return_point1 ||
            g_z3.state == app_zone3_state_stop_nav)
        {
            app_zone3_clear_motion();
            flow_mode = flow_none;
            g_z3.failed = 1U;
            g_z3.active = 0U;
            app_zone3_enter_state(app_zone3_state_failed, now_ms);
            return;
        }
    }

    if ((g_z3.state == app_zone3_state_wait_r1_cmd || g_z3.state == app_zone3_state_on_r1_wait_cmd) &&
        app_zone3_take_normal_cmd(&cmd) != 0U)
    {
        if (g_z3.state == app_zone3_state_on_r1_wait_cmd && app_zone3_cmd_is_any_put(cmd.id) == 0U)
        {
            return;
        }
        app_zone3_dispatch_cmd(&cmd, now_ms);
    }

    switch (g_z3.state)
    {
        case app_zone3_state_entry_nav:
        {
            app_zone3_r1_cmd_t cmd;

            /* 优先处理途中R1命令 */
            if (app_zone3_take_normal_cmd(&cmd) != 0U)
            {
                float x_m, y_m;
                g_z3.active_cmd = cmd.id;
                switch (cmd.id)
                {
                    case APP_Z3_CMD_PUT_KFS_P2:
                    case APP_Z3_CMD_PUT_KFS_P3:
                    case APP_Z3_CMD_PUT_KFS_P4:
                        g_z3.put_sub = cmd.put_sub;
                        app_zone3_get_point(cmd.id, &x_m, &y_m);
                        app_zone3_apply_put_y_offset(cmd.put_sub, &y_m);
                        app_zone3_begin_nav_keep_vx(x_m, y_m, app_zone3_state_nav_to_put, now_ms);
                        break;

                    case APP_Z3_CMD_GET_KFS_G1:
                    case APP_Z3_CMD_GET_KFS_G2:
                        app_zone3_get_point(cmd.id, &x_m, &y_m);
                        main_lift_position = main_lift_p2;
                        app_zone3_begin_nav_keep_vx(x_m, y_m,
                            (cmd.id == APP_Z3_CMD_GET_KFS_G1)
                                ? app_zone3_state_nav_to_g1
                                : app_zone3_state_nav_to_g2,
                            now_ms);
                        break;

                    case APP_Z3_CMD_UP_R1:
                        g_z3.up_r1_deferred = 1U;
                        break;

                    default:
                        break;
                }
                break;
            }

            nav_rc = app_zone3_nav_peek();
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                app_zone3_clear_motion();
                if (g_z3.up_r1_deferred != 0U)
                {
                    g_z3.up_r1_deferred = 0U;
                    main_lift_position = main_lift_p4;
                    kfs_spin_position = kfs_spin_p3;
                    flow_mode = flow_up_r1_mode;
                    app_zone3_enter_state(app_zone3_state_up_r1_climb, now_ms);
                }
                else
                {
                    app_zone3_enter_state(app_zone3_state_wait_r1_cmd, now_ms);
                }
            }
            else
            {
                (void)app_zone3_nav_failed(nav_rc, now_ms);
            }
            break;
        }

        case app_zone3_state_nav_to_put:
            nav_rc = app_zone3_nav_peek();
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                app_zone3_clear_motion();
                app_zone3_enter_state(app_zone3_state_put_kfs, now_ms);
            }
            else
            {
                (void)app_zone3_nav_failed(nav_rc, now_ms);
            }
            break;

        case app_zone3_state_return_point1:
        case app_zone3_state_stop_nav:
            nav_rc = app_zone3_nav_peek();
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                app_zone3_clear_motion();
                /* three_kfs 减量已移至 Process_PutKFS 内部 retract 步骤完成时执行 */
                if (g_z3.state == app_zone3_state_return_point1 && g_z3.up_r1_deferred != 0U)
                {
                    g_z3.up_r1_deferred = 0U;
                    main_lift_position = main_lift_p4;
                    kfs_spin_position = kfs_spin_p3;
                    flow_mode = flow_up_r1_mode;
                    app_zone3_enter_state(app_zone3_state_up_r1_climb, now_ms);
                }
                else
                {
                    app_zone3_enter_state(app_zone3_state_wait_r1_cmd, now_ms);
                }
            }
            else
            {
                (void)app_zone3_nav_failed(nav_rc, now_ms);
            }
            break;


        case app_zone3_state_nav_to_g1:
        case app_zone3_state_nav_to_g2:
            nav_rc = app_zone3_nav_peek();
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                app_zone3_clear_motion();
                /* 到位后启动取地面KFS */
                kfs_spin_position = kfs_spin_p2;
                Process_GetKFS(APP_ZONE2_GET_KFS_GROUND);
                app_zone3_enter_state(
                    (g_z3.state == app_zone3_state_nav_to_g1)
                        ? app_zone3_state_get_kfs_g1
                        : app_zone3_state_get_kfs_g2,
                    now_ms);
            }
            else
            {
                (void)app_zone3_nav_failed(nav_rc, now_ms);
            }
            break;

        case app_zone3_state_get_kfs_g1:
        case app_zone3_state_get_kfs_g2:
            Process_GetKFS(APP_ZONE2_GET_KFS_GROUND);
            if (Process_GetKFS_IsBusy() != 0U)
            {
                if ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.action_timeout_ms)
                {
                    g_z3.failed = 1U;
                    g_z3.active = 0U;
                    app_zone3_enter_state(app_zone3_state_failed, now_ms);
                }
                break;
            }
            /* KFS取完回点1 */
            flow_mode = flow_none;
            if (APP_ZONE2_RED_SIDE == 0U)
                YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_RIGHT);
            else
                YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_LEFT);
            app_zone3_begin_nav(g_app_zone3_cfg.p1_x_m,
                                g_app_zone3_cfg.p1_y_m,
                                app_zone3_state_return_point1,
                                now_ms);
            break;
        case app_zone3_state_put_kfs:
            app_zone3_run_put_kfs(now_ms, app_zone3_state_return_point1);
            break;

        case app_zone3_state_up_r1_delay:
            main_lift_position = main_lift_p4;
            kfs_spin_position = kfs_spin_p3;
            if ((now_ms - g_z3.state_enter_ms) >= g_app_zone3_cfg.up_r1_delay_ms)
            {
                flow_mode = flow_up_r1_mode;
                app_zone3_enter_state(app_zone3_state_up_r1_climb, now_ms);
            }
            else if ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.action_timeout_ms)
            {
                g_z3.failed = 1U;
                g_z3.active = 0U;
                app_zone3_enter_state(app_zone3_state_failed, now_ms);
            }
            break;

        case app_zone3_state_up_r1_climb:
            if (Process_UpR1_IsBusy() != 0U)
            {
                if ((now_ms - g_z3.state_enter_ms) > g_app_zone3_cfg.action_timeout_ms)
                {
                    flow_mode = flow_none;
                    Process_Flow_ClearChassisOverride();
                    g_z3.failed = 1U;
                    g_z3.active = 0U;
                    app_zone3_enter_state(app_zone3_state_failed, now_ms);
                }
                break;
            }
            g_z3.on_r1 = 1U;
            app_zone3_enter_state(app_zone3_state_on_r1_wait_cmd, now_ms);
            break;

        case app_zone3_state_on_r1_put_kfs:
            app_zone3_run_put_kfs(now_ms, app_zone3_state_on_r1_wait_cmd);
            break;

        case app_zone3_state_wait_r1_cmd:
#if APP_ZONE3_DBG_FAKE_CMD
            if (g_z3.cmd_pending == 0U && g_z3.stop_pending == 0U)
            {
                if (s_dbg_fake_cmd_enter_tick == 0U)
                {
                    s_dbg_fake_cmd_enter_tick = now_ms;
                }
                else if ((now_ms - s_dbg_fake_cmd_enter_tick) >= APP_ZONE3_DBG_FAKE_CMD_DELAY_MS)
                {
                    if (s_dbg_fake_cmd_idx < APP_ZONE3_DBG_FAKE_CMD_COUNT)
                    {
                        app_zone3_r1_cmd_t fake_cmd;
                        fake_cmd.id = s_dbg_fake_cmd_seq[s_dbg_fake_cmd_idx];
                        fake_cmd.seq = (uint8_t)(s_dbg_fake_cmd_idx + 1U);
                        fake_cmd.raw_cmd = 0U;
                        AppZone3_PostR1Cmd(&fake_cmd);
                        s_dbg_fake_cmd_idx++;
                        s_dbg_fake_cmd_enter_tick = 0U;
                    }
                }
            }
#endif
            /* fall through */
        case app_zone3_state_on_r1_wait_cmd:
        case app_zone3_state_done:
        case app_zone3_state_failed:
        case app_zone3_state_idle:
        default:
            break;
    }
}

uint8_t AppZone3_PutKFS_IsBusy(void)
{
    return Process_PutKFS_IsBusy();
}
