#include "app_zone1.h"
#include "app_init.h"

#include "Process_Flow.h"
#include "clamp_head_ctrl.h"
#include "yaw_heading_ctrl.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"

#include <math.h>

#define APP_ZONE1_NAV_ODOM_MAX_AGE_MS_DEFAULT  (500U)
/* 临时兜底：1=wait_r1_release 超时后自动继续，0=始终等待 R1 */
#define APP_ZONE1_GRAB_RETRY_MAX               (8U)

#define APP_ZONE1_WAIT_R1_TIMEOUT_ENABLE       (1U)

#define APP_ZONE1_CHASSIS_AXES_NAV             ((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY | \
                                                         PROCESS_FLOW_CHASSIS_OVERRIDE_VW))
#define APP_ZONE1_CHASSIS_PRIO_MOTION          PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH

typedef enum
{
    app_zone1_nav_turn_none = 0,
    app_zone1_nav_turn_90,
    app_zone1_nav_turn_180,
} app_zone1_nav_turn_t;

//底盘输出前后命令给Y（+前-后），左右命令给Z（-左+右），旋转给X（-左+右）
volatile AppZone1Config g_app_zone1_cfg = {
    .forward_target_x_m = 1.4f, //第一导航点x坐标
    .forward_target_y_m = 0.4f, //第一导航点y坐标
    .forward2_target_x_m = 1.0f, //第二导航点x坐标
    .forward2_target_y_m = 0.95f, //第二导航点y坐标
    .step_start_target_x_m = 3.0f, //台阶起始点x坐标
    .step_start_target_y_m = 2.78f, //台阶起始点y坐标
    .shift_right_slow_cmd = 20.0f, //右移慢速vz指令（符号按底盘约定）
    .shift_right_vy_comp_cmd = -5.0f, //右移监控时的vy抵消补偿
    .back_slow_cmd = -10.0f, //慢退顶限位vy指令
    .forward_slow_cmd = 10.0f, //慢进顶限位vy指令       
    .limit_meas_rpm_thr = 10.0f, //限位检测：轮速绝对值均值低于此认为顶住
    .limit_cmd_thr = 2.0f, //限位检测：指令绝对值高于此才判堵转 单位：rpm           
    .limit_debounce_ms = 180U, //限位检测防抖时间 单位：ms  
    .limit_timeout_ms = 6000U, //单段顶限位最大时间 单位：ms（右移命令保持时间）
    .clamp_timeout_ms = 10000U, //夹爪：右移触发后等到直立保持超时 单位：ms
    .clamp_upright_hold_dwell_ms = 2000U, //直立保持后再保持时间 单位：ms（0=不延长）
    .r1_wait_timeout_ms = 5000U, //等待 R1 释放指令超时（占位） 单位：ms
    .action_timeout_ms = 15000U, //单步转向等动作超时 单位：ms
    .nav_odom_max_age_ms = APP_ZONE1_NAV_ODOM_MAX_AGE_MS_DEFAULT, //导航里程计最大允许年龄 单位：ms 
};

static uint8_t app_zone1_cfg_validate(const AppZone1Config *cfg)       //配置验证      
{
    if (cfg == 0)
    {
        return 0U;
    }
    if (!isfinite(cfg->forward_target_x_m) || !isfinite(cfg->forward_target_y_m))   //第一导航点x坐标和y坐标是否有效    
    {
        return 0U;
    }
    if (!isfinite(cfg->forward2_target_x_m) || !isfinite(cfg->forward2_target_y_m)) //第二导航点x坐标和y坐标是否有效    
    {
        return 0U;
    }
    if (!isfinite(cfg->step_start_target_x_m) || !isfinite(cfg->step_start_target_y_m)) //台阶起始点x坐标和y坐标是否有效
    {
        return 0U;
    }
    if (!isfinite(cfg->shift_right_slow_cmd) || !isfinite(cfg->shift_right_vy_comp_cmd) ||
        !isfinite(cfg->back_slow_cmd) ||
        !isfinite(cfg->forward_slow_cmd)) //右移慢速vy指令、慢退顶限位vy指令、慢进顶限位vy指令是否有效    
    {
        return 0U;
    }
    /* 补偿量不能喧宾夺主，约束在右移主命令的 30% 内 */
    if (fabsf(cfg->shift_right_vy_comp_cmd) > (0.3f * fabsf(cfg->shift_right_slow_cmd)))
    {
        return 0U;
    }
    if (!isfinite(cfg->limit_meas_rpm_thr) || cfg->limit_meas_rpm_thr < 0.0f) //限位检测：轮速绝对值均值低于此认为顶住是否有效    
    {
        return 0U;
    }
    if (!isfinite(cfg->limit_cmd_thr) || cfg->limit_cmd_thr < 0.0f) //限位检测：指令绝对值高于此才判堵转是否有效    
    {
        return 0U;
    }
    if (cfg->limit_debounce_ms == 0U || cfg->limit_timeout_ms == 0U ||
        cfg->clamp_timeout_ms == 0U ||
        cfg->r1_wait_timeout_ms == 0U || cfg->action_timeout_ms == 0U) //单段顶限位最大时间、夹爪：右移触发后等到直立保持超时、等待对接通知超时、等待 R1 释放指令超时、单步转向等动作超时是否有效    
    {
        return 0U;
    }
    return 1U;
}

typedef enum
{
    app_zone1_state_idle = 0,           //空闲状态
    app_zone1_state_nav_to_fixed_point, //导航到第一导航点
    app_zone1_state_back_slow_to_limit, //慢退顶限位    
    app_zone1_state_shift_right_monitor, //右移监控
    app_zone1_state_shift_right_clamp_wait, //右移夹爪等待
    app_zone1_state_nav_to_forward2, //导航到第二导航点
    app_zone1_state_forward_slow_to_limit, //慢进顶限位    
    app_zone1_state_wait_r1_release, //等待 R1 释放指令    
    app_zone1_state_nav_to_step_start, //导航到台阶起始点
    app_zone1_state_done, //完成状态    
    app_zone1_state_abort, //中止状态       
} app_zone1_state_t;

typedef struct
{
    app_zone1_state_t state; //当前状态 
    uint32_t state_enter_ms; //状态进入时间
    uint32_t limit_detect_start_ms; //限位检测开始时间
    uint32_t r1_wait_start_ms; //等待 R1 释放指令开始时间
    volatile uint8_t r1_pending; //等待 R1 释放指令标志
    uint8_t yaw_cmd_issued; //转向指令已发出标志
    uint8_t grab_latched; //抓取触发已锁底盘标志
    uint32_t grab_retry_count; /* 夹爪等待内“夹空→回右移”已发生次数 */
    uint8_t grab_was_active_in_clamp_wait; /* 本段夹爪等待内曾进入过非 idle 子状态 */
    uint8_t active; //流程是否运行中标志    
    uint8_t done; //正常结束标志
    uint8_t failed; //失败结束标志
    uint8_t clamp_upright_hold_dwell_started; /* 1=已进入直立保持并开始计驻留 */
    uint32_t clamp_upright_hold_dwell_start_ms; /* 驻留起始 tick */
    ClampHeadState clamp_prev_state; //本周期 Run 前夹爪状态（边沿检测用）
    odom_nav_goto_target_t target; //导航目标
    odom_nav_goto_err_t last_nav_rc; //上次 odom_nav_goto_run 返回值
} app_zone1_ctx_t;  

static app_zone1_ctx_t g_app_zone1_ctx; //一区流程上下文    

static uint8_t app_zone1_flow_state_depends_on_nav_odom(app_zone1_state_t st) //状态依赖于导航里程计是否可靠
{
    return (uint8_t)((st == app_zone1_state_nav_to_fixed_point) ||
                     (st == app_zone1_state_nav_to_forward2) ||
                     (st == app_zone1_state_nav_to_step_start));
}

static uint8_t app_zone1_flow_nav_odom_trustworthy(void) //导航里程计是否可靠    
{
    if (rc_odom_is_valid() == 0U) //导航里程计是否有效      
    {
        return 0U;
    }
    if (g_app_zone1_cfg.nav_odom_max_age_ms > 0U) //导航里程计最大允许年龄大于0    
    {
        if (rc_get_odom_age_ms() > g_app_zone1_cfg.nav_odom_max_age_ms) //导航里程计年龄大于最大允许年龄            
        {
            return 0U; //不可靠        
        }
    }
    return 1U; //可靠    
}

/** 导航前只释放 Vy/Vw，Vx 留给航向（与二区 z2_exec_release_chassis_for_nav 一致） */
static void app_zone1_flow_release_for_nav(void)
{
    Process_Flow_ClearChassisOverrideAxes(APP_ZONE1_CHASSIS_AXES_NAV);
}

static void app_zone1_flow_clear_motion_override(void)
{
    Process_Flow_ClearChassisOverrideAxesByPriority(
        (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY | PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
        APP_ZONE1_CHASSIS_PRIO_MOTION);
}

static void app_zone1_flow_apply_chassis_axes(uint8_t axis_mask, float vx_cmd, float vy_cmd, float vw_cmd)
{
    if (Process_Flow_ChassisOverrideCanWrite(axis_mask, APP_ZONE1_CHASSIS_PRIO_MOTION) == 0U)
    {
        return;
    }
    Process_Flow_SetChassisOverrideAxes(axis_mask, APP_ZONE1_CHASSIS_PRIO_MOTION, vx_cmd, vy_cmd, vw_cmd);
}

static uint8_t app_zone1_flow_post_nav_turn(app_zone1_nav_turn_t turn)
{
    if (turn == app_zone1_nav_turn_90)
    {
#if APP_ZONE2_RED_SIDE
        return YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_right_90);
#else
        return YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_left_90);
#endif
    }
    if (turn == app_zone1_nav_turn_180)
    {
        return YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_180);
    }
    return 1U;
}

/** 导航段到点且航向跟踪结束（Vx 由 yaw_heading_ctrl 占权，Vy/Vw 由 odom 占权，可并行） */
static uint8_t app_zone1_flow_nav_leg_complete(odom_nav_goto_err_t nav_rc)
{
    return (uint8_t)((nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) && (YawHeadingCtrl_IsBusy() == 0U));
}

static float app_zone1_flow_get_chassis_rpm_abs_avg(void) //获取四轮转速绝对值均值      
{
    float rpm_sum = 0.0f; //四轮转速绝对值均值    
    rpm_sum += fabsf((float)chassis_motor1.speed_rpm); //底盘电机1转速绝对值    
    rpm_sum += fabsf((float)chassis_motor2.speed_rpm); //底盘电机2转速绝对值    
    rpm_sum += fabsf((float)chassis_motor3.speed_rpm); //底盘电机3转速绝对值    
    rpm_sum += fabsf((float)chassis_motor4.speed_rpm); //底盘电机4转速绝对值    
    return rpm_sum * 0.25f; //四轮转速绝对值均值    
}

static uint8_t app_zone1_flow_limit_hit_detect(float cmd_abs, float meas_abs, uint32_t now_ms) //限位检测    
{
    if ((cmd_abs >= g_app_zone1_cfg.limit_cmd_thr) &&
        (meas_abs <= g_app_zone1_cfg.limit_meas_rpm_thr)) //指令绝对值大于等于限位检测阈值且轮速绝对值小于等于限位检测阈值        
    {
        if (g_app_zone1_ctx.limit_detect_start_ms == 0U)
        {
            g_app_zone1_ctx.limit_detect_start_ms = now_ms;
        }
        if ((now_ms - g_app_zone1_ctx.limit_detect_start_ms) >= g_app_zone1_cfg.limit_debounce_ms)
        {
            return 1U;
        }
        return 0U;
    }
    g_app_zone1_ctx.limit_detect_start_ms = 0U;
    return 0U; //限位检测失败    
}

static void app_zone1_set_nav_target(float x_m, float y_m)
{
    odom_nav_goto_set_target(x_m, y_m);
    g_app_zone1_ctx.target.x_m = x_m;
    g_app_zone1_ctx.target.y_m = y_m;
    g_app_zone1_ctx.target.session_id = odom_nav_target.session_id;
}

/* 与二区一致：新航段 disarm → 设目标 → 只放 Vy/Vw；可选同时 Post 转向（与导航并行） */
static void app_zone1_flow_nav_leg_begin(float x_m, float y_m, app_zone1_nav_turn_t turn)
{
    app_zone1_flow_release_for_nav();
    odom_nav_goto_disarm();
    app_zone1_set_nav_target(x_m, y_m);
    g_app_zone1_ctx.yaw_cmd_issued = 0U;
    if (turn != app_zone1_nav_turn_none)
    {
        if (app_zone1_flow_post_nav_turn(turn) == 0U)
        {
            g_app_zone1_ctx.yaw_cmd_issued = 0U;
            return;
        }
        g_app_zone1_ctx.yaw_cmd_issued = 1U;
    }
}

static odom_nav_goto_err_t app_zone1_flow_nav_peek(void)
{
    odom_nav_goto_err_t nav_rc = odom_nav_goto_peek_last_run_result();

    if (g_app_zone1_ctx.target.session_id != odom_nav_target.session_id)
    {
        return ODOM_NAV_GOTO_ERR_DISARMED;
    }
    return nav_rc;
}

static void app_zone1_flow_enter_state(app_zone1_state_t state, uint32_t now_ms)
{
    g_app_zone1_ctx.state = state;
    g_app_zone1_ctx.state_enter_ms = now_ms;
    g_app_zone1_ctx.limit_detect_start_ms = 0U; //限位检测开始时间    
    if (state == app_zone1_state_shift_right_clamp_wait) //右移夹爪等待状态
    {
        g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U; //直立保持驻留未开始
        g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms = 0U; //直立保持驻留起始时间
        g_app_zone1_ctx.grab_was_active_in_clamp_wait = 0U;
    }
}
static void app_zone1_flow_clamp_wait_exit_to_forward2(uint32_t now_ms) //夹爪等待结束进入第二导航点    
{
    app_zone1_flow_clear_motion_override();
    app_zone1_flow_nav_leg_begin(g_app_zone1_cfg.forward2_target_x_m,
                                 g_app_zone1_cfg.forward2_target_y_m,
                                 app_zone1_nav_turn_180);
    if (g_app_zone1_ctx.yaw_cmd_issued == 0U)
    {
        app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
        return;
    }
    app_zone1_flow_enter_state(app_zone1_state_nav_to_forward2, now_ms);
}


void AppZone1_Reset(void) //重置流程    
{
    Process_Flow_ClearChassisOverride(); //清除底盘覆盖    
    odom_nav_goto_disarm(); //卸权导航，避免 Reset/Init 后默认点被 service_tick 推进
    g_app_zone1_ctx.state = app_zone1_state_idle; //空闲状态    
    g_app_zone1_ctx.state_enter_ms = 0U; //状态进入时间    
    g_app_zone1_ctx.limit_detect_start_ms = 0U; //限位检测开始时间    
    g_app_zone1_ctx.r1_wait_start_ms = 0U; //等待 R1 释放指令开始时间    
    g_app_zone1_ctx.r1_pending = 0U; //等待 R1 释放指令标志    
    g_app_zone1_ctx.yaw_cmd_issued = 0U; //转向指令已发出标志    
    g_app_zone1_ctx.grab_latched = 0U; //抓取触发已锁底盘标志
    g_app_zone1_ctx.grab_retry_count = 0U;
    g_app_zone1_ctx.grab_was_active_in_clamp_wait = 0U;
    g_app_zone1_ctx.active = 0U; //流程是否运行中标志
    g_app_zone1_ctx.done = 0U; //正常结束标志    
    g_app_zone1_ctx.failed = 0U; //失败结束标志    
    g_app_zone1_ctx.clamp_prev_state = clamp_head_state_idle; //本周期 Run 前夹爪状态（边沿检测用）    
    g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U; //直立保持驻留未开始    
    g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms = 0U; //直立保持驻留起始时间    
    g_app_zone1_ctx.last_nav_rc = ODOM_NAV_GOTO_ERR_OK_ARRIVED; //上次 odom_nav_goto_run 返回值     
    g_app_zone1_ctx.target.x_m = g_app_zone1_cfg.forward_target_x_m; //第一导航点 x    
    g_app_zone1_ctx.target.y_m = g_app_zone1_cfg.forward_target_y_m; //第一导航点 y
    g_app_zone1_ctx.target.session_id = 0U; //会话ID
}

void AppZone1_Init(void) //初始化流程    
{
    AppZone1_Reset(); //重置流程       
}

void AppZone1_Start(void) //启动流程    
{
    uint32_t now_ms = osKernelGetTickCount(); //当前时间    

    AppZone1_Reset(); //重置流程    
    ClampHeadCtrl_Init(); //夹爪初始化    

    g_app_zone1_ctx.active = 1U; //流程是否运行中标志    
    g_app_zone1_ctx.done = 0U; //正常结束标志    
    g_app_zone1_ctx.failed = 0U; //失败结束标志    
    g_app_zone1_ctx.grab_latched = 0U; //抓取触发已锁底盘标志    
    g_app_zone1_ctx.r1_pending = 0U; //等待 R1 释放指令标志    
    app_zone1_flow_nav_leg_begin(g_app_zone1_cfg.forward_target_x_m,
                                 g_app_zone1_cfg.forward_target_y_m,
                                 app_zone1_nav_turn_90);
    if (g_app_zone1_ctx.yaw_cmd_issued == 0U)
    {
        g_app_zone1_ctx.active = 0U;
        g_app_zone1_ctx.failed = 1U;
        return;
    }
    app_zone1_flow_enter_state(app_zone1_state_nav_to_fixed_point, now_ms);
}

uint8_t AppZone1_GetConfig(AppZone1Config *out) //获取配置    
{
    if (out == 0)
    {
        return 0U;
    }
    *out = g_app_zone1_cfg;
    return 1U;
}

uint8_t AppZone1_SetConfig(const AppZone1Config *cfg) //设置配置    
{
    if (app_zone1_cfg_validate(cfg) == 0U)
    {
        return 0U;
    }
    g_app_zone1_cfg = *cfg;
    return 1U;
}

uint8_t AppZone1_SetForwardTarget(float x_m, float y_m) //设置第一导航点    
{
    AppZone1Config cfg = g_app_zone1_cfg; //配置    
    cfg.forward_target_x_m = x_m; //第一导航点 x    
    cfg.forward_target_y_m = y_m; //第一导航点 y    
    return AppZone1_SetConfig(&cfg); //设置配置    
}

uint8_t AppZone1_SetForward2Target(float x_m, float y_m) //设置第二导航点      
{
    AppZone1Config cfg = g_app_zone1_cfg; //配置    
    cfg.forward2_target_x_m = x_m; //第二导航点 x    
    cfg.forward2_target_y_m = y_m; //第二导航点 y    
    return AppZone1_SetConfig(&cfg); //设置配置    
}


void AppZone1_NotifyR1Release(void) //通知 R1 释放指令    
{
    g_app_zone1_ctx.r1_pending = 1U; //等待 R1 释放指令标志    1=已通知 R1 释放指令     
}

uint8_t AppZone1_IsBusy(void) //流程是否运行中    
{
    return g_app_zone1_ctx.active; //流程是否运行中标志    1=运行中    0=停止           
}

uint8_t AppZone1_IsDone(void) //流程是否完成    
{
    return g_app_zone1_ctx.done; //正常结束标志    1=完成    0=未完成           
}

uint8_t AppZone1_IsFailed(void) //流程是否失败    
{
    return g_app_zone1_ctx.failed; //失败结束标志    1=失败    0=未失败           
}

void AppZone1_Run(void) //运行流程    
{
    uint32_t now_ms; //当前时间    
    float meas_rpm_abs; //四轮转速绝对值均值    
    odom_nav_goto_err_t nav_rc; //导航返回码    
    ClampHeadState prev_s; //本周期 Run 前夹爪状态（边沿检测用）    
    ClampHeadState cur_s; //当前夹爪状态    

    if (g_app_zone1_ctx.active == 0U) //流程是否运行中标志    0=停止    1=运行中           
    {
        return; //流程未运行中，直接返回    
    }

    now_ms = osKernelGetTickCount(); //当前时间    

    if (app_zone1_flow_state_depends_on_nav_odom(g_app_zone1_ctx.state) != 0U) //状态依赖于导航里程计    0=不依赖    1=依赖           
    {
        if (app_zone1_flow_nav_odom_trustworthy() == 0U) //导航里程计是否可靠    0=不可靠    1=可靠           
        {
            Process_Flow_ClearChassisOverride(); //清除底盘覆盖    
            app_zone1_flow_enter_state(app_zone1_state_abort, now_ms); //进入中止状态    
            meas_rpm_abs = app_zone1_flow_get_chassis_rpm_abs_avg(); //四轮转速绝对值均值    
            return; //导航里程计不可靠，直接返回    
        }
    }

    meas_rpm_abs = app_zone1_flow_get_chassis_rpm_abs_avg(); //四轮转速绝对值均值    

    switch (g_app_zone1_ctx.state) //当前状态       
    {
        case app_zone1_state_nav_to_fixed_point: //导航+转向并行（航向在 manual_chassis_function 内 Run）
            nav_rc = app_zone1_flow_nav_peek();
            g_app_zone1_ctx.last_nav_rc = nav_rc;
            if (app_zone1_flow_nav_leg_complete(nav_rc) != 0U)
            {
                app_zone1_flow_release_for_nav();
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
                g_app_zone1_ctx.yaw_cmd_issued = 0U;
                app_zone1_flow_enter_state(app_zone1_state_back_slow_to_limit, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED))
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) &&
                     ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms))
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_back_slow_to_limit: //慢退顶限位状态        
            app_zone1_flow_apply_chassis_axes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY,
                                              0.0f,
                                              g_app_zone1_cfg.back_slow_cmd,
                                              0.0f);
            if (app_zone1_flow_limit_hit_detect(fabsf(g_app_zone1_cfg.back_slow_cmd), meas_rpm_abs, //限位检测
                                                now_ms) != 0U)
            {
                app_zone1_flow_clear_motion_override();
                g_app_zone1_ctx.clamp_prev_state = ClampHeadCtrl_GetState(); //本周期 Run 前夹爪状态（边沿检测用）
                g_app_zone1_ctx.grab_retry_count = 0U; /* 新一段右移：重试计数清零 */
                app_zone1_flow_enter_state(app_zone1_state_shift_right_monitor, now_ms); //进入右移监控状态
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.limit_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms); //进入中止状态       
            }
            break;

        case app_zone1_state_shift_right_monitor: //右移监控状态        
            prev_s = g_app_zone1_ctx.clamp_prev_state; //本周期 Run 前夹爪状态（边沿检测用）    
            cur_s = ClampHeadCtrl_GetState(); //当前夹爪状态    
            if ((prev_s == clamp_head_state_idle) && (cur_s == clamp_head_state_wait_close_delay)) //本周期 Run 前夹爪状态为空闲状态且当前夹爪状态为等待关闭延迟状态    
            {
                app_zone1_flow_clear_motion_override();
                g_app_zone1_ctx.grab_latched = 1U; //抓取触发已锁底盘标志    1=已锁底盘    0=未锁底盘           
                app_zone1_flow_enter_state(app_zone1_state_shift_right_clamp_wait, now_ms); //进入右移夹爪等待状态    
                g_app_zone1_ctx.clamp_prev_state = cur_s; //本周期 Run 前夹爪状态（边沿检测用）    
                break;
            }
#if APP_ZONE2_RED_SIDE
            app_zone1_flow_apply_chassis_axes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                        PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
                                              0.0f,
                                              g_app_zone1_cfg.shift_right_vy_comp_cmd,
                                              -g_app_zone1_cfg.shift_right_slow_cmd);
#else
            app_zone1_flow_apply_chassis_axes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                        PROCESS_FLOW_CHASSIS_OVERRIDE_VW),
                                              0.0f,
                                              g_app_zone1_cfg.shift_right_vy_comp_cmd,
                                              g_app_zone1_cfg.shift_right_slow_cmd);
#endif
            g_app_zone1_ctx.clamp_prev_state = cur_s;
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.limit_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_shift_right_clamp_wait: //右移夹爪等待状态
        {
            ClampHeadState clamp_cs;

 //夹爪运行
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.clamp_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms); //进入中止状态
                break;
            }

            clamp_cs = ClampHeadCtrl_GetState();
            if (clamp_cs != clamp_head_state_idle)
            {
                g_app_zone1_ctx.grab_was_active_in_clamp_wait = 1U;
            }

            if (clamp_cs == clamp_head_state_dock_ok) //对接完成：立即进入第二导航点
            {
                app_zone1_flow_clamp_wait_exit_to_forward2(now_ms);
                break;
            }

            if (clamp_cs == clamp_head_state_upright_hold) //直立保持：PE9 持续有物 + 驻留后才进入第二导航点
            {
                if (ClampHeadCtrl_IsObjectPresentRaw() == 0U)
                {
                    g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U;
                }
                else
                {
                    if (g_app_zone1_ctx.clamp_upright_hold_dwell_started == 0U) //驻留开始标志    0=未开始    1=已开始           
                    {
                        g_app_zone1_ctx.clamp_upright_hold_dwell_started = 1U; //驻留开始标志    1=已开始    0=未开始           
                        g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms = now_ms; //驻留开始时间    
                    }
                    if ((now_ms - g_app_zone1_ctx.clamp_upright_hold_dwell_start_ms) >=           
                        g_app_zone1_cfg.clamp_upright_hold_dwell_ms) //驻留时间大于等于配置的时间       
                    {
                        app_zone1_flow_clamp_wait_exit_to_forward2(now_ms); //进入第二导航点        
                    }
                }
                break;
            }

            /* 夹空：曾进入抓取子流程又回到 idle → 回右移监控重试（有次数上限） */
            if ((clamp_cs == clamp_head_state_idle) && (g_app_zone1_ctx.grab_was_active_in_clamp_wait != 0U))
            {
                if (g_app_zone1_ctx.grab_retry_count < APP_ZONE1_GRAB_RETRY_MAX)
                {
                    g_app_zone1_ctx.grab_retry_count++; //夹爪等待内“夹空→回右移”已发生次数增加                                                      
                    ClampHeadCtrl_Init();    //初始化夹爪       
                    g_app_zone1_ctx.grab_latched = 0U; //抓取触发已锁底盘标志    0=未锁底盘    1=已锁底盘           
                    g_app_zone1_ctx.grab_was_active_in_clamp_wait = 0U; //本段夹爪等待内曾进入过非 idle 子状态    0=未进入    1=已进入           
                    g_app_zone1_ctx.clamp_prev_state = ClampHeadCtrl_GetState(); //本周期 Run 前夹爪状态（边沿检测用）    
                    app_zone1_flow_enter_state(app_zone1_state_shift_right_monitor, now_ms); //进入右移监控状态    
                    break;
                }
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms); //进入中止状态    
                break;
            }

            /* 离开直立保持等：清驻留计时（避免误用旧时间） */
            g_app_zone1_ctx.clamp_upright_hold_dwell_started = 0U;
            break;
        }

        case app_zone1_state_nav_to_forward2: //导航+转180并行
            nav_rc = app_zone1_flow_nav_peek();
            g_app_zone1_ctx.last_nav_rc = nav_rc;
            if (app_zone1_flow_nav_leg_complete(nav_rc) != 0U)
            {
                app_zone1_flow_release_for_nav();
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
                g_app_zone1_ctx.yaw_cmd_issued = 0U;
                app_zone1_flow_enter_state(app_zone1_state_forward_slow_to_limit, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED))
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) &&
                     ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms))
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_forward_slow_to_limit: //慢进顶限位状态        
            app_zone1_flow_apply_chassis_axes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY,
                                              0.0f,
                                              g_app_zone1_cfg.forward_slow_cmd,
                                              0.0f);
            if (app_zone1_flow_limit_hit_detect(fabsf(g_app_zone1_cfg.forward_slow_cmd), meas_rpm_abs,
                                                now_ms) != 0U)
            {
                app_zone1_flow_clear_motion_override();
                g_app_zone1_ctx.r1_wait_start_ms = now_ms; //等待 R1 释放指令开始时间    
                app_zone1_flow_enter_state(app_zone1_state_wait_r1_release, now_ms); //直接进入等待 R1 释放指令状态        
                break;
            }
            if ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.limit_timeout_ms)
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_wait_r1_release: //等待 R1 释放指令状态        
            app_zone1_flow_clear_motion_override();
            if (g_app_zone1_ctx.r1_pending != 0U)
            {
                g_app_zone1_ctx.r1_pending = 0U;
                ClampHeadCtrl_NotifyDockOk();
                app_zone1_flow_nav_leg_begin(g_app_zone1_cfg.step_start_target_x_m,
                                             g_app_zone1_cfg.step_start_target_y_m,
                                             app_zone1_nav_turn_90);
                if (g_app_zone1_ctx.yaw_cmd_issued == 0U)
                {
                    app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
                    break;
                }
                app_zone1_flow_enter_state(app_zone1_state_nav_to_step_start, now_ms);
                break;
            }
#if APP_ZONE1_WAIT_R1_TIMEOUT_ENABLE
            if ((now_ms - g_app_zone1_ctx.r1_wait_start_ms) > g_app_zone1_cfg.r1_wait_timeout_ms)
            {
                app_zone1_flow_nav_leg_begin(g_app_zone1_cfg.step_start_target_x_m,
                                             g_app_zone1_cfg.step_start_target_y_m,
                                             app_zone1_nav_turn_90);
                if (g_app_zone1_ctx.yaw_cmd_issued == 0U)
                {
                    app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
                    break;
                }
                app_zone1_flow_enter_state(app_zone1_state_nav_to_step_start, now_ms);
            }
#endif
            break;

        case app_zone1_state_nav_to_step_start: //导航+转向并行
            nav_rc = app_zone1_flow_nav_peek();
            g_app_zone1_ctx.last_nav_rc = nav_rc;
            if (app_zone1_flow_nav_leg_complete(nav_rc) != 0U)
            {
                app_zone1_flow_release_for_nav();
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
                g_app_zone1_ctx.yaw_cmd_issued = 0U;
                app_zone1_flow_enter_state(app_zone1_state_done, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED))
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) &&
                     ((now_ms - g_app_zone1_ctx.state_enter_ms) > g_app_zone1_cfg.action_timeout_ms))
            {
                app_zone1_flow_enter_state(app_zone1_state_abort, now_ms);
            }
            break;

        case app_zone1_state_done: //完成状态        
            Process_Flow_ClearChassisOverride();
            g_app_zone1_ctx.active = 0U; //流程是否运行中标志    0=停止    1=运行中           
            g_app_zone1_ctx.done = 1U; //正常结束标志    1=完成    0=未完成           
            g_app_zone1_ctx.failed = 0U; //失败结束标志    1=失败    0=未失败           
            g_app_zone1_ctx.state = app_zone1_state_idle; //空闲状态    
            break;

        case app_zone1_state_abort: //中止状态        
            Process_Flow_ClearChassisOverride();
            g_app_zone1_ctx.active = 0U; //流程是否运行中标志    0=停止    1=运行中           
            g_app_zone1_ctx.done = 0U; //正常结束标志    1=完成    0=未完成           
            g_app_zone1_ctx.failed = 1U; //失败结束标志    1=失败    0=未失败           
            g_app_zone1_ctx.state = app_zone1_state_idle; //空闲状态    
            break;

        case app_zone1_state_idle: //空闲状态        
        default:
            g_app_zone1_ctx.active = 0U; //流程是否运行中标志    0=停止    1=运行中           
            g_app_zone1_ctx.state = app_zone1_state_idle; //空闲状态    
            break;
    }
}
