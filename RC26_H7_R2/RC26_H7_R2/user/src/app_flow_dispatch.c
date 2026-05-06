#include "app_flow_dispatch.h"

#include "Motion_Task.h"
#include "Process_Flow.h"
#include "app_clamp_head_ctrl.h"
#include "app_yaw_heading_ctrl.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"

#include <math.h>

//导航目标点x坐标
#define APP_FLOW_NAV_TARGET_X_M            (1.0f)
//导航目标点y坐标
#define APP_FLOW_NAV_TARGET_Y_M            (1.0f)
//导航超时时间
#define APP_FLOW_NAV_TIMEOUT_MS            (15000U)
//动作超时时间
#define APP_FLOW_ACTION_TIMEOUT_MS         (15000U)
//会话id初始值
#define APP_FLOW_SESSION_ID_INIT           (1U)

#define APP_FLOW_ZONE1_FORWARD_X_M         (1.0f)
#define APP_FLOW_ZONE1_FORWARD_Y_M         (1.0f)
#define APP_FLOW_ZONE1_BACKOFF_DIST_M      (0.60f)
#define APP_FLOW_ZONE1_BACK_SLOW_DIST_M    (0.80f)
#define APP_FLOW_ZONE1_SHIFT_RIGHT_CMD     (-25.0f)
#define APP_FLOW_ZONE1_BACK_SLOW_CMD       (-20.0f)
#define APP_FLOW_ZONE1_LIMIT_MEAS_RPM_THR  (25.0f)
#define APP_FLOW_ZONE1_LIMIT_CMD_THR       (10.0f)
#define APP_FLOW_ZONE1_LIMIT_DEBOUNCE_MS   (180U)
#define APP_FLOW_ZONE1_LIMIT_TIMEOUT_MS    (6000U)
#define APP_FLOW_ZONE1_CLAMP_TIMEOUT_MS    (5000U)
#define APP_FLOW_ZONE1_DOCK_TIMEOUT_MS     (20000U)

typedef enum
{
    app_flow_state_idle = 0, //空闲
    app_flow_state_nav_to_point, //导航到点
    app_flow_state_do_action, //执行动作
    app_flow_state_zone1_turn_left_90, //一区左转90°
    app_flow_state_zone1_forward_to_limit, //一区前进触限位
    app_flow_state_zone1_shift_right_and_clamp, //一区右移并夹取
    app_flow_state_zone1_backoff, //一区后退定距
    app_flow_state_zone1_turn_180, //一区旋转180°
    app_flow_state_zone1_back_slow, //一区慢速后退
    app_flow_state_zone1_back_to_limit, //一区慢速后退触限位
    app_flow_state_zone1_wait_dock_ok, //一区等待对接确认
    app_flow_state_done, //完成
    app_flow_state_abort, //中止
} AppFlowState;

typedef enum
{
    app_flow_action_none = 0, //无动作
    app_flow_action_upstairs, //上台阶
    app_flow_action_downstairs, //下台阶
    app_flow_action_get_kfs, //取kfs
    app_flow_action_put_kfs, //放kfs
    app_flow_action_zone1_clamp_head, //一区夹枪头流程
} AppFlowAction;

typedef odom_nav_goto_err_t (*AppFlowNavRunFn)(const odom_nav_goto_target_t *target, odom_nav_goto_status_t *status);
typedef void (*AppFlowActionFn)(void);


//上下文结构体
typedef struct
{
    AppFlowState state; //状态
    AppFlowAction action; //动作
    Semi_auto_mode action_mode; //动作模式
    uint32_t state_enter_ms; //状态进入时间
    uint32_t session_id_seed; //会话id种子
    odom_nav_goto_target_t target; //目标
    AppFlowNavRunFn nav_run; //导航运行函数
    AppFlowActionFn action_run; //动作运行函数
    uint32_t limit_detect_start_ms; //限位检测起始时间
    uint32_t clamp_lock_start_ms; //夹爪锁定起始时间
    uint32_t dock_wait_start_ms; //对接等待起始时间
    uint8_t dock_ok_notified; //外部对接通知标志
    uint8_t yaw_cmd_issued; //旋转命令已发送标志
    uint8_t zone1_back_stage; //一区后退阶段标志
    float zone1_back_start_x_m; //后退起点x
    float zone1_back_start_y_m; //后退起点y
} AppFlowDispatchCtx;

static AppFlowDispatchCtx g_app_flow_ctx;
volatile AppFlowDispatchDebug g_app_flow_dispatch_debug = {0U};

//根据模式获取动作
static uint8_t app_flow_action_from_mode(Semi_auto_mode mode, AppFlowAction *action_out)
{
    if (action_out == 0)
    {
        return 0U;
    }

    switch (mode)
    {
        case semi_auto_upstairs_mode:   //上台阶模式
            *action_out = app_flow_action_upstairs;
            return 1U;
        case semi_auto_downstairs_mode: //下台阶模式
            *action_out = app_flow_action_downstairs;
            return 1U;
        case semi_auto_get_kfs_mode: //取kfs模式
            *action_out = app_flow_action_get_kfs;
            return 1U;
        case semi_auto_put_kfs_mode: //放kfs模式
            *action_out = app_flow_action_put_kfs;
            return 1U;
        case semi_auto_zone1_clamp_head_mode: //一区夹枪头模式
            *action_out = app_flow_action_zone1_clamp_head;
            return 1U;
        default: //无模式   
            *action_out = app_flow_action_none;
            return 0U;
    }
}

static void app_flow_debug_snapshot(uint32_t now_ms, float cmd_vy, float cmd_vw, float meas_rpm_abs)
{
    if (g_app_flow_dispatch_debug.enable == 0U)
    {
        return;
    }
    g_app_flow_dispatch_debug.seq++;
    g_app_flow_dispatch_debug.now_ms = now_ms;
    g_app_flow_dispatch_debug.flow_state = (uint32_t)g_app_flow_ctx.state;
    g_app_flow_dispatch_debug.flow_action = (uint32_t)g_app_flow_ctx.action;
    g_app_flow_dispatch_debug.limit_debounce_ms = g_app_flow_ctx.limit_detect_start_ms;
    g_app_flow_dispatch_debug.cmd_vy = cmd_vy;
    g_app_flow_dispatch_debug.cmd_vw = cmd_vw;
    g_app_flow_dispatch_debug.meas_chassis_rpm_abs = meas_rpm_abs;
    g_app_flow_dispatch_debug.target_x_m = g_app_flow_ctx.target.x_m;
    g_app_flow_dispatch_debug.target_y_m = g_app_flow_ctx.target.y_m;
}

static void app_flow_apply_chassis_cmd(float vx_cmd, float vy_cmd, float vw_cmd)
{
    process_flow_chassis_override.axis_mask = (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VX |
                                                        PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                        PROCESS_FLOW_CHASSIS_OVERRIDE_VW);
    process_flow_chassis_override.vx = vx_cmd;
    process_flow_chassis_override.vy = vy_cmd;
    process_flow_chassis_override.vw = vw_cmd;
}

static float app_flow_get_chassis_rpm_abs_avg(void)
{
    float rpm_sum = 0.0f;
    rpm_sum += fabsf((float)chassis_motor1.speed_rpm);
    rpm_sum += fabsf((float)chassis_motor2.speed_rpm);
    rpm_sum += fabsf((float)chassis_motor3.speed_rpm);
    rpm_sum += fabsf((float)chassis_motor4.speed_rpm);
    return rpm_sum * 0.25f;
}

static uint8_t app_flow_limit_hit_detect(float cmd_abs, float meas_abs, uint32_t now_ms)
{
    if ((cmd_abs >= APP_FLOW_ZONE1_LIMIT_CMD_THR) && (meas_abs <= APP_FLOW_ZONE1_LIMIT_MEAS_RPM_THR))
    {
        if (g_app_flow_ctx.limit_detect_start_ms == 0U)
        {
            g_app_flow_ctx.limit_detect_start_ms = now_ms;
        }
        if ((now_ms - g_app_flow_ctx.limit_detect_start_ms) >= APP_FLOW_ZONE1_LIMIT_DEBOUNCE_MS)
        {
            return 1U;
        }
        return 0U;
    }
    g_app_flow_ctx.limit_detect_start_ms = 0U;
    return 0U;
}

static uint8_t app_flow_read_odom_xy(float *x_m_out, float *y_m_out)
{
    const rc_odom_t *odom_ptr;
    if ((x_m_out == 0) || (y_m_out == 0))
    {
        return 0U;
    }
    if (rc_odom_is_valid() == 0U)
    {
        return 0U;
    }
    odom_ptr = rc_get_latest_odom();
    *x_m_out = odom_ptr->x;
    *y_m_out = odom_ptr->y;
    return 1U;
}

static uint8_t app_flow_zone1_start_back_nav(float back_dist_m)
{
    float cur_x_m;
    float cur_y_m;
    if (app_flow_read_odom_xy(&cur_x_m, &cur_y_m) == 0U)
    {
        return 0U;
    }
    g_app_flow_ctx.zone1_back_start_x_m = cur_x_m;
    g_app_flow_ctx.zone1_back_start_y_m = cur_y_m;
    g_app_flow_ctx.target.x_m = cur_x_m;
    g_app_flow_ctx.target.y_m = cur_y_m - back_dist_m;
    g_app_flow_ctx.target.session_id = g_app_flow_ctx.session_id_seed++;
    odom_nav_goto_clear_state();
    return 1U;
}

static void app_flow_zone1_enter_state(AppFlowState state, uint32_t now_ms)
{
    g_app_flow_ctx.state = state;
    g_app_flow_ctx.state_enter_ms = now_ms;
    g_app_flow_ctx.limit_detect_start_ms = 0U;
}

//默认动作运行
static void app_flow_default_action_run(void)
{
    switch (g_app_flow_ctx.action)
    {
        case app_flow_action_upstairs: //上台阶动作
            Process_UpStairs();
            break;
        case app_flow_action_downstairs: //下台阶动作
            Process_DownStairs();
            break;
        case app_flow_action_get_kfs: //取kfs动作
            Process_GetKFS();
            break;
        case app_flow_action_put_kfs: //放kfs动作
            Process_PutKFS();
            break;
        case app_flow_action_none: //无动作     
        default: //默认动作                                                                  
            break;
    }
}

static uint8_t app_flow_zone1_is_action_done(void)
{
    AppClampHeadState clamp_state = AppClampHeadCtrl_GetState();
    return (uint8_t)((clamp_state == app_clamp_head_state_upright_hold) ||
                     (clamp_state == app_clamp_head_state_dock_ok));
}
//清除所有状态，并回到空闲状态
static void app_flow_cleanup_to_idle(void)          
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_clear_state();
    g_app_flow_ctx.action = app_flow_action_none;
    g_app_flow_ctx.action_mode = semi_auto_none;
    g_app_flow_ctx.state = app_flow_state_idle;
    g_app_flow_ctx.state_enter_ms = osKernelGetTickCount();
    g_app_flow_ctx.limit_detect_start_ms = 0U;
    g_app_flow_ctx.clamp_lock_start_ms = 0U;
    g_app_flow_ctx.dock_wait_start_ms = 0U;
    g_app_flow_ctx.dock_ok_notified = 0U;
    g_app_flow_ctx.yaw_cmd_issued = 0U;
    g_app_flow_ctx.zone1_back_stage = 0U;
    g_app_flow_ctx.zone1_back_start_x_m = 0.0f;
    g_app_flow_ctx.zone1_back_start_y_m = 0.0f;
}

//开始导航
static void app_flow_start_nav(AppFlowAction action, Semi_auto_mode action_mode)
{
    g_app_flow_ctx.action = action;
    g_app_flow_ctx.action_mode = action_mode;
    g_app_flow_ctx.target.x_m = APP_FLOW_NAV_TARGET_X_M;
    g_app_flow_ctx.target.y_m = APP_FLOW_NAV_TARGET_Y_M;
    if (action == app_flow_action_zone1_clamp_head)
    {
        g_app_flow_ctx.target.x_m = APP_FLOW_ZONE1_FORWARD_X_M;
        g_app_flow_ctx.target.y_m = APP_FLOW_ZONE1_FORWARD_Y_M;
    }
    g_app_flow_ctx.target.session_id = g_app_flow_ctx.session_id_seed++;
    odom_nav_goto_clear_state();
    g_app_flow_ctx.state = app_flow_state_nav_to_point;
    g_app_flow_ctx.state_enter_ms = osKernelGetTickCount();
    g_app_flow_ctx.limit_detect_start_ms = 0U;
    g_app_flow_ctx.clamp_lock_start_ms = 0U;
    g_app_flow_ctx.dock_wait_start_ms = 0U;
    g_app_flow_ctx.dock_ok_notified = 0U;
    g_app_flow_ctx.yaw_cmd_issued = 0U;
    g_app_flow_ctx.zone1_back_stage = 0U;
    g_app_flow_ctx.zone1_back_start_x_m = 0.0f;
    g_app_flow_ctx.zone1_back_start_y_m = 0.0f;
}

void AppFlowDispatch_NotifyDockOk(void)
{
    g_app_flow_ctx.dock_ok_notified = 1U;
}

//初始化
void AppFlowDispatch_Init(void)
{
    g_app_flow_ctx.state = app_flow_state_idle;
    g_app_flow_ctx.action = app_flow_action_none;
    g_app_flow_ctx.action_mode = semi_auto_none;
    g_app_flow_ctx.state_enter_ms = 0U;
    g_app_flow_ctx.session_id_seed = APP_FLOW_SESSION_ID_INIT;
    g_app_flow_ctx.target.x_m = 0.0f;
    g_app_flow_ctx.target.y_m = 0.0f;
    g_app_flow_ctx.target.session_id = APP_FLOW_SESSION_ID_INIT;
    g_app_flow_ctx.nav_run = odom_nav_goto_run;
    g_app_flow_ctx.action_run = app_flow_default_action_run;
    g_app_flow_ctx.limit_detect_start_ms = 0U;
    g_app_flow_ctx.clamp_lock_start_ms = 0U;
    g_app_flow_ctx.dock_wait_start_ms = 0U;
    g_app_flow_ctx.dock_ok_notified = 0U;
    g_app_flow_ctx.yaw_cmd_issued = 0U;
    g_app_flow_ctx.zone1_back_stage = 0U;
    g_app_flow_ctx.zone1_back_start_x_m = 0.0f;
    g_app_flow_ctx.zone1_back_start_y_m = 0.0f;
}
    
//运行
void AppFlowDispatch_Run(void)
{
    uint32_t now_ms;
    AppFlowAction request_action = app_flow_action_none;
    odom_nav_goto_err_t nav_rc;
    float meas_rpm_abs = 0.0f;

    if ((control_mode != semi_auto_control) || (g_app_flow_ctx.nav_run == 0) || (g_app_flow_ctx.action_run == 0))
    {
        app_flow_cleanup_to_idle();
        return;
    }

    now_ms = osKernelGetTickCount();
    meas_rpm_abs = app_flow_get_chassis_rpm_abs_avg();
    app_flow_debug_snapshot(now_ms, process_flow_chassis_override.vy, process_flow_chassis_override.vw, meas_rpm_abs);

    switch (g_app_flow_ctx.state)
    {
        case app_flow_state_idle:
            if (app_flow_action_from_mode(semi_auto_mode, &request_action) != 0U)
            {
                app_flow_start_nav(request_action, semi_auto_mode);
            }
            break;

        case app_flow_state_nav_to_point:
            if ((semi_auto_mode == semi_auto_none) || ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_NAV_TIMEOUT_MS))
            {
                g_app_flow_ctx.state = app_flow_state_abort;
                g_app_flow_ctx.state_enter_ms = now_ms;
                break;
            }

            nav_rc = g_app_flow_ctx.nav_run(&g_app_flow_ctx.target, 0);
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                semi_auto_mode = g_app_flow_ctx.action_mode;
                if (g_app_flow_ctx.action == app_flow_action_zone1_clamp_head)
                {
                    AppClampHeadCtrl_Init();
                    app_flow_zone1_enter_state(app_flow_state_zone1_turn_left_90, now_ms);
                }
                else
                {
                    app_flow_zone1_enter_state(app_flow_state_do_action, now_ms);
                }
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
            }
            else
            {
                /* moving */
            }
            break;

        case app_flow_state_do_action:
            if ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_ACTION_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
                break;
            }

            g_app_flow_ctx.action_run();
            if (semi_auto_mode == semi_auto_none)
            {
                app_flow_zone1_enter_state(app_flow_state_done, now_ms);
            }
            break;

        case app_flow_state_zone1_turn_left_90:
            if (g_app_flow_ctx.yaw_cmd_issued == 0U)
            {
                if (AppYawHeadingCtrl_PostCommand(app_yaw_heading_cmd_turn_left_90) == 0U)
                {
                    app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
                    break;
                }
                g_app_flow_ctx.yaw_cmd_issued = 1U;
            }
            AppYawHeadingCtrl_Run();
            if ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_ACTION_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
                break;
            }
            if ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) == 0U)
            {
                g_app_flow_ctx.yaw_cmd_issued = 0U;
                app_flow_zone1_enter_state(app_flow_state_zone1_forward_to_limit, now_ms);
            }
            break;

        case app_flow_state_zone1_forward_to_limit:
            app_flow_apply_chassis_cmd(0.0f, APP_FLOW_ZONE1_SHIFT_RIGHT_CMD * -1.0f, 0.0f);
            if (app_flow_limit_hit_detect(fabsf(APP_FLOW_ZONE1_SHIFT_RIGHT_CMD), meas_rpm_abs, now_ms) != 0U)
            {
                Process_Flow_ClearChassisOverride();
                app_flow_zone1_enter_state(app_flow_state_zone1_shift_right_and_clamp, now_ms);
                break;
            }
            if ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_ZONE1_LIMIT_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
            }
            break;

        case app_flow_state_zone1_shift_right_and_clamp:
            AppClampHeadCtrl_Run();
            app_flow_apply_chassis_cmd(0.0f, 0.0f, APP_FLOW_ZONE1_SHIFT_RIGHT_CMD);
            if (app_flow_zone1_is_action_done() != 0U)
            {
                Process_Flow_ClearChassisOverride();
                g_app_flow_ctx.clamp_lock_start_ms = now_ms;
                if (app_flow_zone1_start_back_nav(APP_FLOW_ZONE1_BACKOFF_DIST_M) == 0U)
                {
                    app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
                    break;
                }
                app_flow_zone1_enter_state(app_flow_state_zone1_backoff, now_ms);
                break;
            }
            if ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_ZONE1_CLAMP_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
            }
            break;

        case app_flow_state_zone1_backoff:
            AppClampHeadCtrl_Run();
            if ((now_ms - g_app_flow_ctx.clamp_lock_start_ms) > APP_FLOW_ZONE1_CLAMP_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
                break;
            }
            nav_rc = g_app_flow_ctx.nav_run(&g_app_flow_ctx.target, 0);
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                app_flow_zone1_enter_state(app_flow_state_zone1_turn_180, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
            }
            break;

        case app_flow_state_zone1_turn_180:
            if (g_app_flow_ctx.yaw_cmd_issued == 0U)
            {
                if (AppYawHeadingCtrl_PostCommand(app_yaw_heading_cmd_turn_180) == 0U)
                {
                    app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
                    break;
                }
                g_app_flow_ctx.yaw_cmd_issued = 1U;
            }
            AppYawHeadingCtrl_Run();
            if ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_ACTION_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
                break;
            }
            if ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) == 0U)
            {
                g_app_flow_ctx.yaw_cmd_issued = 0U;
                if (app_flow_zone1_start_back_nav(APP_FLOW_ZONE1_BACK_SLOW_DIST_M) == 0U)
                {
                    app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
                    break;
                }
                app_flow_zone1_enter_state(app_flow_state_zone1_back_slow, now_ms);
            }
            break;

        case app_flow_state_zone1_back_slow:
            nav_rc = g_app_flow_ctx.nav_run(&g_app_flow_ctx.target, 0);
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                app_flow_zone1_enter_state(app_flow_state_zone1_back_to_limit, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
            }
            break;

        case app_flow_state_zone1_back_to_limit:
            app_flow_apply_chassis_cmd(0.0f, APP_FLOW_ZONE1_BACK_SLOW_CMD, 0.0f);
            if (app_flow_limit_hit_detect(fabsf(APP_FLOW_ZONE1_BACK_SLOW_CMD), meas_rpm_abs, now_ms) != 0U)
            {
                Process_Flow_ClearChassisOverride();
                g_app_flow_ctx.dock_wait_start_ms = now_ms;
                app_flow_zone1_enter_state(app_flow_state_zone1_wait_dock_ok, now_ms);
                break;
            }
            if ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_ZONE1_LIMIT_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
            }
            break;

        case app_flow_state_zone1_wait_dock_ok:
            Process_Flow_ClearChassisOverride();
            AppClampHeadCtrl_Run();
            if (g_app_flow_ctx.dock_ok_notified != 0U)
            {
                AppClampHeadCtrl_NotifyDockOk();
                g_app_flow_ctx.dock_ok_notified = 0U;
                semi_auto_mode = semi_auto_none;
                app_flow_zone1_enter_state(app_flow_state_done, now_ms);
                break;
            }
            if ((now_ms - g_app_flow_ctx.dock_wait_start_ms) > APP_FLOW_ZONE1_DOCK_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
            }
            break;

        case app_flow_state_done:
            app_flow_cleanup_to_idle();
            break;

        case app_flow_state_abort:
            Process_Flow_ClearChassisOverride();
            semi_auto_mode = semi_auto_none;
            app_flow_cleanup_to_idle();
            break;

        default:
            g_app_flow_ctx.state = app_flow_state_abort;
            g_app_flow_ctx.state_enter_ms = now_ms;
            break;
    }
}
