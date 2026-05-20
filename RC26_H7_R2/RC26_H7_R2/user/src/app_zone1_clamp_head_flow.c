#include "app_zone1_clamp_head_flow.h"

#include "Process_Flow.h"
#include "app_clamp_head_ctrl.h"
#include "app_yaw_heading_ctrl.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "odom_nav_goto.h"
#include "odom_center_offset.h"
#include "upper_pc_protocol.h"

#include <math.h>

#define APP_ZONE1_SESSION_ID_INIT            (1000U)
/** 导航子状态中里程计最大允许龄期默认值（ms）；0 可在配置中关闭龄期判据 */
#define APP_ZONE1_NAV_ODOM_MAX_AGE_MS_DEFAULT  (500U)

volatile AppZone1ClampHeadFlowConfig g_app_zone1_clamp_head_flow_cfg = {
    .forward_target_x_m = 0.3f,              //前进x坐标  
    .forward_target_y_m = 0.0f,              //前进y坐标          
    .backoff_dist_m = 0.30f,                 //后退距离
    .back_slow_dist_m = 0.30f,               //后退慢速距离
    .shift_right_cmd = -10.0f,               //右移命令
    .back_slow_cmd = -10.0f,                 //后退慢速命令
    .limit_meas_rpm_thr = 10.0f,             //限制测量rpm阈值
    .limit_cmd_thr = 2.0f,                   //限制命令阈值
    .limit_debounce_ms = 180U,               //限制debounce时间
    .limit_timeout_ms = 6000U,              //限制超时时间
    .clamp_timeout_ms = 5000U,              //夹爪超时时间
    .dock_timeout_ms = 20000U,              //对接超时时间      
    .action_timeout_ms = 15000U,            //动作超时时间
    .nav_odom_max_age_ms = APP_ZONE1_NAV_ODOM_MAX_AGE_MS_DEFAULT,
};

volatile AppZone1ClampHeadFlowStepCtrl g_app_zone1_clamp_head_flow_step = {
    .enable = 0U,
    .allow = 1U,
    .last_from_state = 0U,
    .last_to_state = 0U,
    .last_transition_ms = 0U,
};

static uint8_t app_zone1_cfg_validate(const AppZone1ClampHeadFlowConfig *cfg)
{
    if (cfg == 0)
    {
        return 0U;
    }

    /* 目标点范围不做死限制，避免绑死地图；仅校验 NaN/Inf 与合理的超时/阈值 */
    if (!isfinite(cfg->forward_target_x_m) || !isfinite(cfg->forward_target_y_m))
    {
        return 0U;
    }
    if (!isfinite(cfg->backoff_dist_m) || cfg->backoff_dist_m < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->back_slow_dist_m) || cfg->back_slow_dist_m < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->shift_right_cmd) || !isfinite(cfg->back_slow_cmd))
    {
        return 0U;
    }
    if (!isfinite(cfg->limit_meas_rpm_thr) || cfg->limit_meas_rpm_thr < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->limit_cmd_thr) || cfg->limit_cmd_thr < 0.0f)
    {
        return 0U;
    }
    if (cfg->limit_debounce_ms == 0U || cfg->limit_timeout_ms == 0U ||
        cfg->clamp_timeout_ms == 0U || cfg->dock_timeout_ms == 0U || cfg->action_timeout_ms == 0U)
    {
        return 0U;
    }
    return 1U;
}

typedef enum
{
    app_zone1_clamp_head_flow_state_idle = 0, //空闲状态
    app_zone1_clamp_head_flow_state_nav_to_fixed_point, //导航到固定点状态
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

/**
 * @brief 判断当前状态是否依赖里程计导航（需在单步 allow=0 之前做失效安全处理）。
 * @return 1=依赖；0=不依赖
 */
static uint8_t app_zone1_flow_state_depends_on_nav_odom(app_zone1_clamp_head_flow_state_t st)
{
    return (uint8_t)((st == app_zone1_clamp_head_flow_state_nav_to_fixed_point) ||
                     (st == app_zone1_clamp_head_flow_state_backoff) ||
                     (st == app_zone1_clamp_head_flow_state_back_slow));
}

/**
 * @brief 判断当前里程计是否可用于导航子状态。
 * @return 1=可信；0=不可信（应中止流程并清底盘覆盖）
 */
static uint8_t app_zone1_flow_nav_odom_trustworthy(void)
{
    if (rc_odom_is_valid() == 0U)
    {
        return 0U;
    }
    if (g_app_zone1_clamp_head_flow_cfg.nav_odom_max_age_ms > 0U)
    {
        if (rc_get_odom_age_ms() > g_app_zone1_clamp_head_flow_cfg.nav_odom_max_age_ms)
        {
            return 0U;
        }
    }
    return 1U;
}

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
    g_app_zone1_clamp_head_flow_debug.step_enable = (uint32_t)g_app_zone1_clamp_head_flow_step.enable;
    g_app_zone1_clamp_head_flow_debug.step_allow = (uint32_t)g_app_zone1_clamp_head_flow_step.allow;
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
    if ((cmd_abs >= g_app_zone1_clamp_head_flow_cfg.limit_cmd_thr) &&
        (meas_abs <= g_app_zone1_clamp_head_flow_cfg.limit_meas_rpm_thr))
    {
        if (g_app_zone1_ctx.limit_detect_start_ms == 0U)
        {
            g_app_zone1_ctx.limit_detect_start_ms = now_ms;
        }
        if ((now_ms - g_app_zone1_ctx.limit_detect_start_ms) >= g_app_zone1_clamp_head_flow_cfg.limit_debounce_ms)
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
    odom_center_offset_odom_to_center(odom_ptr, x_m_out, y_m_out);
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
    /* 单步模式：每次发生状态跳转后自动暂停，等待外部再次放行 */
    if (g_app_zone1_clamp_head_flow_step.enable != 0U)
    {
        g_app_zone1_clamp_head_flow_step.last_from_state = (uint32_t)g_app_zone1_ctx.state;
        g_app_zone1_clamp_head_flow_step.last_to_state = (uint32_t)state;
        g_app_zone1_clamp_head_flow_step.last_transition_ms = now_ms;
        g_app_zone1_clamp_head_flow_step.allow = 0U;
    }

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
    g_app_zone1_ctx.target.x_m = g_app_zone1_clamp_head_flow_cfg.forward_target_x_m;
    g_app_zone1_ctx.target.y_m = g_app_zone1_clamp_head_flow_cfg.forward_target_y_m;
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
    g_app_zone1_ctx.target.x_m = g_app_zone1_clamp_head_flow_cfg.forward_target_x_m;
    g_app_zone1_ctx.target.y_m = g_app_zone1_clamp_head_flow_cfg.forward_target_y_m;
    g_app_zone1_ctx.target.session_id = g_app_zone1_ctx.session_id_seed++;
    odom_nav_goto_clear_state();
    app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_nav_to_fixed_point, now_ms);
}

uint8_t AppZone1ClampHeadFlow_GetConfig(AppZone1ClampHeadFlowConfig *out)
{
    if (out == 0)
    {
        return 0U;
    }
    *out = g_app_zone1_clamp_head_flow_cfg;
    return 1U;
}

uint8_t AppZone1ClampHeadFlow_SetConfig(const AppZone1ClampHeadFlowConfig *cfg)
{
    if (app_zone1_cfg_validate(cfg) == 0U)
    {
        return 0U;
    }
    g_app_zone1_clamp_head_flow_cfg = *cfg;
    return 1U;
}

uint8_t AppZone1ClampHeadFlow_SetForwardTarget(float x_m, float y_m)
{
    AppZone1ClampHeadFlowConfig cfg = g_app_zone1_clamp_head_flow_cfg;
    cfg.forward_target_x_m = x_m;
    cfg.forward_target_y_m = y_m;
    return AppZone1ClampHeadFlow_SetConfig(&cfg);
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

    /*
     * 单步 allow=0 时若先 return，则依赖里程计的导航子状态永远无法进入 switch，
     * odom_nav_goto_run 的 ODOM_READ 分支也执行不到，表现为“卡在状态 1、failed=0”。
     * 故：里程计失效时必须在单步闸门之前中止并清覆盖。
     */
    if (app_zone1_flow_state_depends_on_nav_odom(g_app_zone1_ctx.state) != 0U)
    {
        if (app_zone1_flow_nav_odom_trustworthy() == 0U)
        {
            Process_Flow_ClearChassisOverride();
            app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms);
            return;
        }
    }

    /* 单步模式：未放行则不执行本拍状态机（终止态必须放行，否则 abort 分支无法落盘） */
    if ((g_app_zone1_clamp_head_flow_step.enable != 0U) &&
        (g_app_zone1_clamp_head_flow_step.allow == 0U) &&
        (g_app_zone1_ctx.state != app_zone1_clamp_head_flow_state_abort) &&
        (g_app_zone1_ctx.state != app_zone1_clamp_head_flow_state_done))
    {
        /* 单步暂停时必须清空覆盖命令，避免上一拍速度命令持续生效导致底盘持续运动。 */
        Process_Flow_ClearChassisOverride();
        return;
    }

    meas_rpm_abs = app_zone1_flow_get_chassis_rpm_abs_avg(); //测量底盘rpm绝对值
    app_zone1_flow_debug_snapshot(now_ms, process_flow_chassis_override.vy, process_flow_chassis_override.vw, meas_rpm_abs);

    switch (g_app_zone1_ctx.state)
    {
        case app_zone1_clamp_head_flow_state_nav_to_fixed_point:
            nav_rc = odom_nav_goto_run(&g_app_zone1_ctx.target, 0);
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_turn_left_90, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
            }
            else if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_clamp_head_flow_cfg.action_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
            }
            break;

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
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_clamp_head_flow_cfg.action_timeout_ms)
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
            app_zone1_flow_apply_chassis_cmd(0.0f, g_app_zone1_clamp_head_flow_cfg.shift_right_cmd * -1.0f, 0.0f);
            if (app_zone1_flow_limit_hit_detect(fabsf(g_app_zone1_clamp_head_flow_cfg.shift_right_cmd), meas_rpm_abs, now_ms) != 0U)
            {
                Process_Flow_ClearChassisOverride();
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_shift_right_and_clamp, now_ms); //进入右移并夹紧状态
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_clamp_head_flow_cfg.limit_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
            }
            break;

        case app_zone1_clamp_head_flow_state_shift_right_and_clamp:
            AppClampHeadCtrl_Run();
            app_zone1_flow_apply_chassis_cmd(0.0f, 0.0f, g_app_zone1_clamp_head_flow_cfg.shift_right_cmd);
            if (app_zone1_flow_is_action_done() != 0U)
            {
                Process_Flow_ClearChassisOverride();
                g_app_zone1_ctx.clamp_lock_start_ms = now_ms; //夹爪锁定时间
                if (app_zone1_flow_start_back_nav(g_app_zone1_clamp_head_flow_cfg.backoff_dist_m) == 0U)
                {
                    app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                    break;
                }
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_backoff, now_ms); //进入后退状态
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_clamp_head_flow_cfg.clamp_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态                       
            }
            break;

        case app_zone1_clamp_head_flow_state_backoff:
            AppClampHeadCtrl_Run();
            if ((now_ms - g_app_zone1_ctx.clamp_lock_start_ms) > g_app_zone1_clamp_head_flow_cfg.clamp_timeout_ms)
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
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_clamp_head_flow_cfg.action_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_abort, now_ms); //进入中止状态
                break;
            }
            if (AppYawHeadingCtrl_IsBusy() == 0U)
            {
                g_app_zone1_ctx.yaw_cmd_issued = 0U; //航向命令已发出标志
                if (app_zone1_flow_start_back_nav(g_app_zone1_clamp_head_flow_cfg.back_slow_dist_m) == 0U)
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
            app_zone1_flow_apply_chassis_cmd(0.0f, g_app_zone1_clamp_head_flow_cfg.back_slow_cmd, 0.0f);
            if (app_zone1_flow_limit_hit_detect(fabsf(g_app_zone1_clamp_head_flow_cfg.back_slow_cmd), meas_rpm_abs, now_ms) != 0U)
            {
                Process_Flow_ClearChassisOverride();
                g_app_zone1_ctx.dock_wait_start_ms = now_ms; //等待对接时间                         
                app_zone1_flow_enter_state(app_zone1_clamp_head_flow_state_wait_dock_ok, now_ms); //进入等待对接成功状态
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_clamp_head_flow_cfg.limit_timeout_ms)
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
            if ((now_ms - g_app_zone1_ctx.dock_wait_start_ms) > g_app_zone1_clamp_head_flow_cfg.dock_timeout_ms)
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
