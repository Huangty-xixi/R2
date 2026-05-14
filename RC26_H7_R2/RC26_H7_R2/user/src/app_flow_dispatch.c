#include "app_flow_dispatch.h"

#include "Motion_Task.h"
#include "Process_Flow.h"
#include "app_clamp_head_ctrl.h"
#include "app_zone1_clamp_head_flow.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"

#include <math.h>

//导航目标点x坐标
#define APP_FLOW_NAV_TARGET_X_M            (0.0f)
//导航目标点y坐标
#define APP_FLOW_NAV_TARGET_Y_M            (0.0f)
//导航超时时间
#define APP_FLOW_NAV_TIMEOUT_MS            (15000U)
//动作超时时间
#define APP_FLOW_ACTION_TIMEOUT_MS         (15000U)
//会话id初始值
#define APP_FLOW_SESSION_ID_INIT           (1U)

typedef enum
{
    app_flow_state_idle = 0, //空闲
    app_flow_state_nav_to_point, //导航到点
    app_flow_state_do_action, //执行动作
    app_flow_state_zone1_flow, //一区夹枪头整流程
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
    Flow_mode action_mode;
    uint32_t state_enter_ms; //状态进入时间
    uint32_t session_id_seed; //会话id种子
    odom_nav_goto_target_t target; //目标
    AppFlowNavRunFn nav_run; //导航运行函数
    AppFlowActionFn action_run; //动作运行函数
} AppFlowDispatchCtx;

static AppFlowDispatchCtx g_app_flow_ctx;
volatile AppFlowDispatchDebug g_app_flow_dispatch_debug = {0U};

typedef enum    
{
    app_flow_abort_reason_none = 0,     
    app_flow_abort_reason_mode_none,
    app_flow_abort_reason_nav_timeout,  
    app_flow_abort_reason_nav_odom_read,
    app_flow_abort_reason_nav_bad_config,
    app_flow_abort_reason_nav_timeout_inner,
    app_flow_abort_reason_zone1_failed,
} app_flow_abort_reason_t;

//根据模式获取动作
static uint8_t app_flow_action_from_mode(Flow_mode mode, AppFlowAction *action_out)
{
    if (action_out == 0)
    {
        return 0U;
    }

    switch (mode)
    {
        case flow_upstairs_mode:
            *action_out = app_flow_action_upstairs;
            return 1U;
        case flow_downstairs_mode:
            *action_out = app_flow_action_downstairs;
            return 1U;
        case flow_get_kfs_mode:
            *action_out = app_flow_action_get_kfs;
            return 1U;
        default: //无模式   
            *action_out = app_flow_action_none;
            return 0U;
    }
}

static void app_flow_debug_snapshot(uint32_t now_ms,
                                    float cmd_vy,
                                    float cmd_vw,
                                    float meas_rpm_abs,
                                    odom_nav_goto_err_t nav_rc,
                                    app_flow_abort_reason_t abort_reason)
{
    if (g_app_flow_dispatch_debug.enable == 0U)
    {
        return;
    }
    g_app_flow_dispatch_debug.seq++;
    g_app_flow_dispatch_debug.now_ms = now_ms;
    g_app_flow_dispatch_debug.flow_state = (uint32_t)g_app_flow_ctx.state;
    g_app_flow_dispatch_debug.flow_action = (uint32_t)g_app_flow_ctx.action;
    g_app_flow_dispatch_debug.flow_mode = (uint32_t)flow_mode;
    g_app_flow_dispatch_debug.control_mode = (uint32_t)control_mode;
    g_app_flow_dispatch_debug.nav_rc = (uint32_t)nav_rc;
    g_app_flow_dispatch_debug.abort_reason = (uint32_t)abort_reason;
    g_app_flow_dispatch_debug.odom_valid = (uint32_t)rc_odom_is_valid();
    g_app_flow_dispatch_debug.odom_age_ms = (uint32_t)rc_get_odom_age_ms();
    g_app_flow_dispatch_debug.limit_debounce_ms = 0U;
    g_app_flow_dispatch_debug.cmd_vy = cmd_vy;
    g_app_flow_dispatch_debug.cmd_vw = cmd_vw;
    g_app_flow_dispatch_debug.meas_chassis_rpm_abs = meas_rpm_abs;
    g_app_flow_dispatch_debug.target_x_m = g_app_flow_ctx.target.x_m;
    g_app_flow_dispatch_debug.target_y_m = g_app_flow_ctx.target.y_m;
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

static void app_flow_zone1_enter_state(AppFlowState state, uint32_t now_ms)
{
    g_app_flow_ctx.state = state;
    g_app_flow_ctx.state_enter_ms = now_ms;
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
            Process_GetKFS(APP_ZONE2_GET_KFS_LOW_TO_HIGH); /* 流程调度无桩高差，占位默认 */
            break;
        case app_flow_action_put_kfs: //放kfs动作
            Process_PutKFS();
            break;
        case app_flow_action_none: //无动作     
        default: //默认动作                                                                  
            break;
    }
}

//清除所有状态，并回到空闲状态
static void app_flow_cleanup_to_idle(void)          
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_clear_state();
    AppZone1ClampHeadFlow_Reset();
    g_app_flow_ctx.action = app_flow_action_none;
    g_app_flow_ctx.action_mode = flow_none;
    g_app_flow_ctx.state = app_flow_state_idle;
    g_app_flow_ctx.state_enter_ms = osKernelGetTickCount();
    if (app_flow_mode == app_flow_zone1)
        app_flow_mode = app_flow_none;
}

//开始导航
static void app_flow_start_nav(AppFlowAction action, Flow_mode action_mode)
{
    g_app_flow_ctx.action = action;             //动作  
    g_app_flow_ctx.action_mode = action_mode;  //动作模式
    g_app_flow_ctx.target.x_m = APP_FLOW_NAV_TARGET_X_M; //目标x坐标
    g_app_flow_ctx.target.y_m = APP_FLOW_NAV_TARGET_Y_M; //目标y坐标
    g_app_flow_ctx.target.session_id = g_app_flow_ctx.session_id_seed++; //会话id
    odom_nav_goto_clear_state(); //清除导航状态
    g_app_flow_ctx.state = app_flow_state_nav_to_point; //状态为导航到点
    g_app_flow_ctx.state_enter_ms = osKernelGetTickCount(); //状态进入时间
}

void AppFlowDispatch_NotifyDockOk(void)
{
    AppZone1ClampHeadFlow_NotifyDockOk();
}

//初始化
void AppFlowDispatch_Init(void)
{
    g_app_flow_ctx.state = app_flow_state_idle;
    g_app_flow_ctx.action = app_flow_action_none;
    g_app_flow_ctx.action_mode = flow_none;
    g_app_flow_ctx.state_enter_ms = 0U;
    g_app_flow_ctx.session_id_seed = APP_FLOW_SESSION_ID_INIT;
    g_app_flow_ctx.target.x_m = 0.0f;
    g_app_flow_ctx.target.y_m = 0.0f;
    g_app_flow_ctx.target.session_id = APP_FLOW_SESSION_ID_INIT;
    g_app_flow_ctx.nav_run = odom_nav_goto_run;
    g_app_flow_ctx.action_run = app_flow_default_action_run;
    AppZone1ClampHeadFlow_Init();

    /* Debug 默认关，需在调试器里将 g_app_flow_dispatch_debug.enable 置 1 */
}
    
//运行
void AppFlowDispatch_Run(void)
{
    uint32_t now_ms;
    AppFlowAction request_action = app_flow_action_none; //请求动作
    odom_nav_goto_err_t nav_rc; //导航返回码
    float meas_rpm_abs = 0.0f;
    app_flow_abort_reason_t abort_reason = app_flow_abort_reason_none;

    if ((control_mode != full_auto_control) || (g_app_flow_ctx.nav_run == 0) || (g_app_flow_ctx.action_run == 0))
    {
        app_flow_cleanup_to_idle();
        return;
    }

    now_ms = osKernelGetTickCount();
    meas_rpm_abs = app_flow_get_chassis_rpm_abs_avg();
    nav_rc = ODOM_NAV_GOTO_ERR_OK_MOVING;

    switch (g_app_flow_ctx.state)
    {
        case app_flow_state_idle:    //空闲状态
            if (app_flow_action_from_mode(flow_mode, &request_action) != 0U)
            {
                app_flow_start_nav(request_action, flow_mode);
            }
            else if ((app_flow_mode == app_flow_zone1) && (flow_mode == flow_none))
            {
                g_app_flow_ctx.action = app_flow_action_zone1_clamp_head;
                g_app_flow_ctx.action_mode = flow_none;
                AppZone1ClampHeadFlow_Start();
                app_flow_zone1_enter_state(app_flow_state_zone1_flow, now_ms);
            }
            break;

        case app_flow_state_nav_to_point: //导航到点状态        
            if ((flow_mode == flow_none) || ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_NAV_TIMEOUT_MS))
            {
                abort_reason = (flow_mode == flow_none) ? app_flow_abort_reason_mode_none //模式为空
                                                                 : app_flow_abort_reason_nav_timeout; //导航超时
                g_app_flow_ctx.state = app_flow_state_abort;
                g_app_flow_ctx.state_enter_ms = now_ms;
                break;
            }

            nav_rc = g_app_flow_ctx.nav_run(&g_app_flow_ctx.target, 0);
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) //到达目标点
            {
                Process_Flow_ClearChassisOverride(); //清除底盘覆盖
                flow_mode = g_app_flow_ctx.action_mode; //动作模式
                if (g_app_flow_ctx.action == app_flow_action_zone1_clamp_head) //动作为一区夹枪头流程
                {
                    AppZone1ClampHeadFlow_Start(); //启动一区夹枪头流程
                    app_flow_zone1_enter_state(app_flow_state_zone1_flow, now_ms);
                }
                else
                {
                    app_flow_zone1_enter_state(app_flow_state_do_action, now_ms); //进入执行动作状态
                }
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) || //导航超时
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) || //odom读取错误
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG)) //导航配置错误
            {
                if (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) //odom读取错误
                {
                    abort_reason = app_flow_abort_reason_nav_odom_read; //odom读取错误
                }
                else if (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) //导航配置错误
                {
                    abort_reason = app_flow_abort_reason_nav_bad_config; //导航配置错误
                }
                else
                {
                    abort_reason = app_flow_abort_reason_nav_timeout_inner; //导航超时内部
                }
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms); //进入中止状态
            }
            else
            {
                /* moving */ //移动中   
            }
            break;

        case app_flow_state_do_action:
            if ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_ACTION_TIMEOUT_MS)
            {
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms); //进入中止状态
                break;
            }

            g_app_flow_ctx.action_run(); //执行动作
            if (flow_mode == flow_none)
            {
                app_flow_zone1_enter_state(app_flow_state_done, now_ms); //进入完成状态
            }
            break;

        case app_flow_state_zone1_flow:
            AppZone1ClampHeadFlow_Run(); //运行一区夹枪头流程
            if (AppZone1ClampHeadFlow_IsFailed() != 0U)
            {
                abort_reason = app_flow_abort_reason_zone1_failed; //一区夹枪头流程失败
                app_flow_zone1_enter_state(app_flow_state_abort, now_ms);
            }
            else if (AppZone1ClampHeadFlow_IsDone() != 0U) //一区夹枪头流程完成
            {
                flow_mode = flow_none;
                app_flow_zone1_enter_state(app_flow_state_done, now_ms); //进入完成状态
            }
            break;

        case app_flow_state_done:
            app_flow_cleanup_to_idle(); //清理到空闲状态
            break;

        case app_flow_state_abort:
            Process_Flow_ClearChassisOverride(); //清除底盘覆盖
            flow_mode = flow_none;
            app_flow_cleanup_to_idle(); //清理到空闲状态
            break;

        default:
            g_app_flow_ctx.state = app_flow_state_abort; //状态为中止状态
            g_app_flow_ctx.state_enter_ms = now_ms; //状态进入时间
            break;
    }

    app_flow_debug_snapshot(now_ms,
                            process_flow_chassis_override.vy, //底盘vy命令          
                            process_flow_chassis_override.vw, //底盘vw命令
                            meas_rpm_abs, //测量rpm绝对值
                            nav_rc, //导航返回码
                            abort_reason); //中止原因       
}
