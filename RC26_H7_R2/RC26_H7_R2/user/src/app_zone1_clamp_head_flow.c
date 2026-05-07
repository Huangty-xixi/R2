#include "app_zone1_clamp_head_flow.h"

#include "Process_Flow.h"
#include "app_clamp_head_ctrl.h"
#include "app_yaw_heading_ctrl.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"

#include <math.h>

#define APP_ZONE1_FORWARD_X_M                (1.0f) //前进x坐标
#define APP_ZONE1_FORWARD_Y_M                (1.0f) //前进y坐标
#define APP_ZONE1_BACKOFF_DIST_M             (0.60f) //后退距离
#define APP_ZONE1_BACK_SLOW_DIST_M           (0.80f) //后退慢速距离
#define APP_ZONE1_SHIFT_RIGHT_CMD            (-25.0f) //右移命令
#define APP_ZONE1_BACK_SLOW_CMD              (-20.0f) //后退慢速命令
#define APP_ZONE1_LIMIT_MEAS_RPM_THR         (25.0f) //限制测量rpm阈值
#define APP_ZONE1_LIMIT_CMD_THR              (10.0f) //限制命令阈值
#define APP_ZONE1_LIMIT_DEBOUNCE_MS          (180U) //限制抖动时间
#define APP_ZONE1_LIMIT_TIMEOUT_MS           (6000U) //限制超时时间
#define APP_ZONE1_CLAMP_TIMEOUT_MS           (5000U) //夹爪超时时间
#define APP_ZONE1_DOCK_TIMEOUT_MS            (20000U) //对接超时时间
#define APP_ZONE1_ACTION_TIMEOUT_MS          (15000U) //动作超时时间
#define APP_ZONE1_SESSION_ID_INIT            (1000U) //会话id初始值

typedef enum
{
    app_zone1_clamp_head_flow_state_idle = 0, //空闲状态
    app_zone1_clamp_head_flow_state_turn_left_90, //左转90度状态
    app_zone1_clamp_head_flow_state_forward_to_limit, //前进到限制状态
    app_zone1_clamp_head_flow_state_shift_right_and_clamp, //右移并夹紧状态
    app_zone1_clamp_head_flow_state_backoff, //后退状态
    app_zone1_clamp_head_flow_state_turn_180, //转180度状态
    app_zone1_clamp_head_flow_state_back_slow, //后退慢速状态                                                   
    app_zone1_clamp_head_flow_state_back_to_limit, //后退到限制状态                               
    app_zone1_clamp_head_flow_state_wait_dock_ok, //等待对接成功状态                   
    app_zone1_clamp_head_flow_state_done, //完成状态                           
    app_zone1_clamp_head_flow_state_abort,      //中止状态
} app_zone1_clamp_head_flow_state_t;

typedef struct
{
    app_zone1_clamp_head_flow_state_t state; //状态
    uint32_t state_enter_ms; //状态进入时间
    uint32_t session_id_seed; //会话id种子
    uint32_t limit_detect_start_ms; //限制检测开始时间
    uint32_t clamp_lock_start_ms; //夹爪锁定时间
    uint32_t dock_wait_start_ms; //等待对接时间
    uint8_t dock_ok_notified; //对接成功通知标志
    uint8_t yaw_cmd_issued; //航向命令已发出标志
    uint8_t active; //活跃标志                                              
    uint8_t done; //完成标志   
    uint8_t failed; //失败标志
    odom_nav_goto_target_t target; //目标
} app_zone1_clamp_head_flow_ctx_t;                  

static app_zone1_clamp_head_flow_ctx_t g_app_zone1_ctx;
volatile AppZone1ClampHeadFlowDebug g_app_zone1_clamp_head_flow_debug = {0U};

static void app_zone1_flow_debug_snapshot(uint32_t now_ms, float cmd_vy, float cmd_vw, float meas_rpm_abs)
{
    if (g_app_zone1_clamp_head_flow_debug.enable == 0U)
    {
        return;
    }

    g_app_zone1_clamp_head_flow_debug.seq++; //序列号递增           
    g_app_zone1_clamp_head_flow_debug.now_ms = now_ms; //当前时间
    g_app_zone1_clamp_head_flow_debug.state = (uint32_t)g_app_zone1_ctx.state; //状态
    g_app_zone1_clamp_head_flow_debug.busy = (uint32_t)g_app_zone1_ctx.active; //忙碌标志
    g_app_zone1_clamp_head_flow_debug.done = (uint32_t)g_app_zone1_ctx.done; //完成标志
    g_app_zone1_clamp_head_flow_debug.failed = (uint32_t)g_app_zone1_ctx.failed; //失败标志
    g_app_zone1_clamp_head_flow_debug.cmd_vy = cmd_vy; //命令vy
    g_app_zone1_clamp_head_flow_debug.cmd_vw = cmd_vw; //命令vw
    g_app_zone1_clamp_head_flow_debug.meas_chassis_rpm_abs = meas_rpm_abs; //测量底盘rpm绝对值
    g_app_zone1_clamp_head_flow_debug.target_x_m = g_app_zone1_ctx.target.x_m; //目标x坐标
    g_app_zone1_clamp_head_flow_debug.target_y_m = g_app_zone1_ctx.target.y_m; //目标y坐标
}

static void app_zone1_flow_apply_chassis_cmd(float vx_cmd, float vy_cmd, float vw_cmd)
{
    process_flow_chassis_override.axis_mask = (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VX |
                                                        PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                        PROCESS_FLOW_CHASSIS_OVERRIDE_VW);
    process_flow_chassis_override.vx = vx_cmd; //vx命令
    process_flow_chassis_override.vy = vy_cmd; //vy命令
    process_flow_chassis_override.vw = vw_cmd; //vw命令
}

static float app_zone1_flow_get_chassis_rpm_abs_avg(void)
{
    float rpm_sum = 0.0f;
    rpm_sum += fabsf((float)chassis_motor1.speed_rpm); //底盘电机1转速绝对值
    rpm_sum += fabsf((float)chassis_motor2.speed_rpm); //底盘电机2转速绝对值
    rpm_sum += fabsf((float)chassis_motor3.speed_rpm); //底盘电机3转速绝对值
    rpm_sum += fabsf((float)chassis_motor4.speed_rpm); //底盘电机4转速绝对值
    return rpm_sum * 0.25f;
}

static uint8_t app_zone1_flow_limit_hit_detect(float cmd_abs, float meas_abs, uint32_t now_ms)
{
    if ((cmd_abs >= APP_ZONE1_LIMIT_CMD_THR) && (meas_abs <= APP_ZONE1_LIMIT_MEAS_RPM_THR))
    {
        if (g_app_zone1_ctx.limit_detect_start_ms == 0U)
        {
            g_app_zone1_ctx.limit_detect_start_ms = now_ms;
        }
        if ((now_ms - g_app_zone1_ctx.limit_detect_start_ms) >= APP_ZONE1_LIMIT_DEBOUNCE_MS)
        {
            return 1U; //返回1表示限制命中
        }
        return 0U; //返回0表示限制未命中
    }
    g_app_zone1_ctx.limit_detect_start_ms = 0U;
    return 0U; //返回0表示限制未命中                        
}

static uint8_t app_zone1_flow_is_action_done(void)
{
    AppClampHeadState clamp_state = AppClampHeadCtrl_GetState();
    return (uint8_t)((clamp_state == app_clamp_head_state_upright_hold) ||
                     (clamp_state == app_clamp_head_state_dock_ok));
}

static uint8_t app_zone1_flow_read_odom_xy(float *x_m_out, float *y_m_out)
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

static uint8_t app_zone1_flow_start_back_nav(float back_dist_m)
{
    float cur_x_m; //当前x坐标
    float cur_y_m; //当前y坐标

    if (app_zone1_flow_read_odom_xy(&cur_x_m, &cur_y_m) == 0U)
    {
        return 0U; //返回0表示读取失败
    }
    g_app_zone1_ctx.target.x_m = cur_x_m; //目标x坐标
    g_app_zone1_ctx.target.y_m = cur_y_m - back_dist_m; //目标y坐标
    g_app_zone1_ctx.target.session_id = g_app_zone1_ctx.session_id_seed++; //会话id递增
    odom_nav_goto_clear_state(); //清除导航状态
    return 1U; //返回1表示导航开始
}

static void app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_t state, uint32_t now_ms)
{
    g_app_zone1_ctx.state = state; //状态
    g_app_zone1_ctx.state_enter_ms = now_ms; //状态进入时间
    g_app_zone1_ctx.limit_detect_start_ms = 0U; //限制检测开始时间
}

void AppZone1ClampHeadFlow_Reset(void)
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_clear_state();

    g_app_zone1_ctx.state = app_zone1_clamp_head_flow_state_idle; //状态为空闲状态          
    g_app_zone1_ctx.state_enter_ms = 0U;
    g_app_zone1_ctx.session_id_seed = APP_ZONE1_SESSION_ID_INIT; //会话id种子
    g_app_zone1_ctx.limit_detect_start_ms = 0U; //限制检测开始时间
    g_app_zone1_ctx.clamp_lock_start_ms = 0U; //夹爪锁定时间
    g_app_zone1_ctx.dock_wait_start_ms = 0U;
    g_app_zone1_ctx.dock_ok_notified = 0U; //对接成功通知标志
    g_app_zone1_ctx.yaw_cmd_issued = 0U; //航向命令已发出标志
    g_app_zone1_ctx.active = 0U;
    g_app_zone1_ctx.done = 0U; //完成标志
    g_app_zone1_ctx.failed = 0U; //失败标志
    g_app_zone1_ctx.target.x_m = APP_ZONE1_FORWARD_X_M; //目标x坐标
    g_app_zone1_ctx.target.y_m = APP_ZONE1_FORWARD_Y_M; //目标y坐标
    g_app_zone1_ctx.target.session_id = APP_ZONE1_SESSION_ID_INIT; //会话id
}

void AppZone1ClampHeadFlow_Init(void)
{
    AppZone1ClampHeadFlow_Reset(); //重置
}

void AppZone1ClampHeadFlow_Start(void)
{
    uint32_t now_ms = osKernelGetTickCount();

    AppZone1ClampHeadFlow_Reset();
    AppClampHeadCtrl_Init();

    g_app_zone1_ctx.active = 1U;
    g_app_zone1_ctx.done = 0U;
    g_app_zone1_ctx.failed = 0U;
    app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_turn_left_90, now_ms);
}

void AppZone1ClampHeadFlow_NotifyDockOk(void)
{
    g_app_zone1_ctx.dock_ok_notified = 1U; //对接成功通知标志
}

uint8_t AppZone1ClampHeadFlow_IsBusy(void)
{
    return g_app_zone1_ctx.active; //活跃标志
}

uint8_t AppZone1ClampHeadFlow_IsDone(void)
{
    return g_app_zone1_ctx.done; //完成标志                         
}

uint8_t AppZone1ClampHeadFlow_IsFailed(void)            
{
    return g_app_zone1_ctx.failed; //失败标志                      
}

void AppZone1ClampHeadFlow_Run(void)
{
    uint32_t now_ms;
    float meas_rpm_abs; //测量底盘rpm绝对值                         
    odom_nav_goto_err_t nav_rc;

    if (g_app_zone1_ctx.active == 0U)       
    {
        return; //返回空闲状态                        
    }

    now_ms = osKernelGetTickCount();            
    meas_rpm_abs = app_zone1_flow_get_chassis_rpm_abs_avg(); //测量底盘rpm绝对值
    app_zone1_flow_debug_snapshot(now_ms, process_flow_chassis_override.vy, process_flow_chassis_override.vw, meas_rpm_abs);

    switch (g_app_zone1_ctx.state)
    {
        case app_zone1_clamp_head_flow_state_turn_left_90:
            if (g_app_zone1_ctx.yaw_cmd_issued == 0U)
            {
                if (AppYawHeadingCtrl_PostCommand(app_yaw_heading_cmd_turn_left_90) == 0U)
                {
                    app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                    break;
                }
                g_app_zone1_ctx.yaw_cmd_issued = 1U; //航向命令已发出标志
            }
            AppYawHeadingCtrl_Run();
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > APP_ZONE1_ACTION_TIMEOUT_MS)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                break;
            }
            if (AppYawHeadingCtrl_IsBusy() == 0U)
            {
                g_app_zone1_ctx.yaw_cmd_issued = 0U; //航向命令已发出标志
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_forward_to_limit, now_ms);
            }
            break;

        case app_zone1_clamp_head_flow_state_forward_to_limit:
            app_zone1_flow_apply_chassis_cmd(0.0f, APP_ZONE1_SHIFT_RIGHT_CMD * -1.0f, 0.0f);
            if (app_zone1_flow_limit_hit_detect(fabsf(APP_ZONE1_SHIFT_RIGHT_CMD), meas_rpm_abs, now_ms) != 0U)
            {
                Process_Flow_ClearChassisOverride();
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_shift_right_and_clamp, now_ms); //进入右移并夹紧状态
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > APP_ZONE1_LIMIT_TIMEOUT_MS)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
            }
            break;

        case app_zone1_clamp_head_flow_state_shift_right_and_clamp:
            AppClampHeadCtrl_Run();
            app_zone1_flow_apply_chassis_cmd(0.0f, 0.0f, APP_ZONE1_SHIFT_RIGHT_CMD);
            if (app_zone1_flow_is_action_done() != 0U)
            {
                Process_Flow_ClearChassisOverride();
                g_app_zone1_ctx.clamp_lock_start_ms = now_ms; //夹爪锁定时间
                if (app_zone1_flow_start_back_nav(APP_ZONE1_BACKOFF_DIST_M) == 0U)
                {
                    app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                    break;
                }
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_backoff, now_ms); //进入后退状态
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > APP_ZONE1_CLAMP_TIMEOUT_MS)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态                       
            }
            break;

        case app_zone1_clamp_head_flow_state_backoff:
            AppClampHeadCtrl_Run();
            if ((now_ms - g_app_zone1_ctx.clamp_lock_start_ms) > APP_ZONE1_CLAMP_TIMEOUT_MS)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                break;
            }
            nav_rc = odom_nav_goto_run(&g_app_zone1_ctx.target, 0);
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_turn_180, now_ms); //进入转180度状态
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
            }
            break;

        case app_zone1_clamp_head_flow_state_turn_180:
            if (g_app_zone1_ctx.yaw_cmd_issued == 0U)
            {
                if (AppYawHeadingCtrl_PostCommand(app_yaw_heading_cmd_turn_180) == 0U)
                {
                    app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                    break;
                }
                g_app_zone1_ctx.yaw_cmd_issued = 1U; //航向命令已发出标志
            }
            AppYawHeadingCtrl_Run();
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > APP_ZONE1_ACTION_TIMEOUT_MS)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                break;
            }
            if (AppYawHeadingCtrl_IsBusy() == 0U)
            {
                g_app_zone1_ctx.yaw_cmd_issued = 0U; //航向命令已发出标志
                if (app_zone1_flow_start_back_nav(APP_ZONE1_BACK_SLOW_DIST_M) == 0U)
                {
                    app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                    break;
                }
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_back_slow, now_ms); //进入后退慢速状态
            }
            break;

        case app_zone1_clamp_head_flow_state_back_slow:
            nav_rc = odom_nav_goto_run(&g_app_zone1_ctx.target, 0);
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_back_to_limit, now_ms); //进入后退到限制状态
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
            }
            break;

        case app_zone1_clamp_head_flow_state_back_to_limit:
            app_zone1_flow_apply_chassis_cmd(0.0f, APP_ZONE1_BACK_SLOW_CMD, 0.0f);
            if (app_zone1_flow_limit_hit_detect(fabsf(APP_ZONE1_BACK_SLOW_CMD), meas_rpm_abs, now_ms) != 0U)
            {
                Process_Flow_ClearChassisOverride();
                g_app_zone1_ctx.dock_wait_start_ms = now_ms; //等待对接时间                         
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_wait_dock_ok, now_ms); //进入等待对接成功状态
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > APP_ZONE1_LIMIT_TIMEOUT_MS)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
            }
            break;

        case app_zone1_clamp_head_flow_state_wait_dock_ok:
            Process_Flow_ClearChassisOverride();
            AppClampHeadCtrl_Run();
            if (g_app_zone1_ctx.dock_ok_notified != 0U)
            {
                AppClampHeadCtrl_NotifyDockOk();
                g_app_zone1_ctx.dock_ok_notified = 0U; //对接成功通知标志
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_done, now_ms); //进入完成状态
                break;
            }
            if ((now_ms - g_app_zone1_ctx.dock_wait_start_ms) > APP_ZONE1_DOCK_TIMEOUT_MS)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
            }
            break;

        case app_zone1_clamp_head_flow_state_done:
            Process_Flow_ClearChassisOverride();
            g_app_zone1_ctx.active = 0U; //活跃标志
            g_app_zone1_ctx.done = 1U; //完成标志
            g_app_zone1_ctx.failed = 0U; //失败标志
            g_app_zone1_ctx.state = app_zone1_clamp_head_flow_state_idle; //状态为空闲状态
            break;

        case app_zone1_clamp_head_flow_state_abort:
            Process_Flow_ClearChassisOverride();
            g_app_zone1_ctx.active = 0U; //活跃标志
            g_app_zone1_ctx.done = 0U; //完成标志
            g_app_zone1_ctx.failed = 1U; //失败标志                 
            g_app_zone1_ctx.state = app_zone1_clamp_head_flow_state_idle;
            break;

        case app_zone1_clamp_head_flow_state_idle:
        default:
            g_app_zone1_ctx.active = 0U;
            break;
    }
}
