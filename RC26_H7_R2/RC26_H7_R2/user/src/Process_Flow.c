#include "Process_Flow.h"
#include "app_init.h"
#include "yaw_heading_ctrl.h"
#include "Motion_Task.h"
#include "lift.h"
#include "kfs.h"
#include "weapon.h"
#include "Sensor_Task.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"
#include "cmsis_os.h"
#include "common.h"
#include "sensor.h"
#include <math.h>

ProcessFlowChassisOverride process_flow_chassis_override = {0U, PROCESS_FLOW_OVERRIDE_PRIORITY_LOW, PROCESS_FLOW_OVERRIDE_PRIORITY_LOW, PROCESS_FLOW_OVERRIDE_PRIORITY_LOW, PROCESS_FLOW_OVERRIDE_PRIORITY_LOW, 0.0f, 0.0f, 0.0f};
UpstairsStep upstairs_step = upstairs_step_chassis_forward_pre;
DownstairsStep downstairs_step = downstairs_step_idle;
GetKfsStep get_kfs_step = get_kfs_step_idle;
PutKfsStep put_kfs_step = put_kfs_step_idle;
static uint8_t s_put_kfs_busy;
UpR1Step up_r1_step = up_r1_step_idle;
static uint8_t s_up_r1_busy;
volatile ProcessUpR1Tune g_process_up_r1_tune = {
    .raise_wait_ms = 800U,
    .fast_fwd_ms   = 800U,
    .fast_fwd_vy   = 80.0f,
    .slow_fwd_ms   = 600U,
    .slow_fwd_vy   = 20.0f,
    .fall_wait_ms  = 600U,
    .post_fwd_ms   = 500U,
    .post_fwd_vy   = 20.0f,
};
volatile ProcessFlowDebug process_flow_debug = {1U};

/** 1=Process_UpStairs mid-cycle (zone2 or flow_upstairs tick) */
static uint8_t s_upstairs_busy;
/** 1=Process_DownStairs mid-cycle */
static uint8_t s_downstairs_busy;
/** 下台阶俯仰检测 */
static float s_downstairs_pitch_abs_base = 0.0f;
static float s_downstairs_pitch_abs_peak = 0.0f;
static uint8_t s_downstairs_fall_confirm = 0U;
/** 1=Process_GetKFS mid-cycle */
static uint8_t s_get_kfs_busy;
/** 1=chassis_forward 已结束，后半段（spin_p1/吸盘/主轴/三轴）仍在跑 */
static uint8_t s_get_kfs_chassis_fwd_done;
/** 1=wait_after_sucker_off 阶段A 已完成（关吸盘） */
static uint8_t s_get_kfs_sucker_off_done;

/**上坡流程参数*/
volatile ProcessUpSlopeTune g_process_upslope_tune = {
    .p1_x_m = PROCESS_UPSLOPE_P1_X_M,
    .p1_y_m = PROCESS_UPSLOPE_P1_Y_M,
    .yaw_tol_deg = 1.0f,
    .vy_target = 60.0f,
    .wait_after_goto_ms = 1000U,
    .pitch_abs_rise_th_deg = 5.0f,
    .pitch_abs_fall_th_deg = 5.0f,
    .fall_confirm_cnt = 1U,
    .stage_timeout_ms = 60000U,
};

/**上台阶流程参数（2026-06-16 实车标定）*/
volatile ProcessUpstairsTune g_process_upstairs_tune = {
    .chassis_forward_pre_ms = 1000U,/* 抬升前底盘前进时间 */
    .vy_chassis_forward_pre = 30.0f,/* 抬升前底盘前进 vy */
    .wait_raise_done_ms = 800U,/* 上升等待时间 */
    .fast_before_fall_ms = 600U,/* 下降前快速前进时间(ms) */
    .vy_fast_before_fall = 120.0f,/* 下降前快速前进 vy */
    .wait_before_fall_ms = 500U,/* 下降前等待时间 */
    .wait_fall_done_ms = 500U,
    .vy_forward = 40.0f,/* 上台阶纵向速度 */
    .chassis_forward_post_ms = 0U,/* 落台等待结束后前进时间 */
    .vy_chassis_forward_post = 0.0f,/* 落台等待结束后前进 vy */	
};

/**放kfs流程参数*/
volatile ProcessPutKfsTune g_process_put_kfs_tune = {
    .wait_extend_ms = 2000U,
    .wait_retract_ms = 1000U,
};


/**下台阶流程参数（2026-06-16 实车标定）*/
volatile ProcessDownstairsTune g_process_downstairs_tune = {
    .vy_backward               = -65.0f,// 下台阶后退 vy
    .pitch_abs_rise_th_deg     = 5.0f,// 上台阶俯仰上升阈值
    .pitch_abs_fall_th_deg     = 5.0f,// 上台阶俯仰下降阈值
    .fall_confirm_cnt          = 1U,// 上台阶俯仰下降确认次数
    .pitch_rise_timeout_ms     = 2000U,// 上台阶俯仰上升超时
    .pitch_fall_timeout_ms     = 2000U,// 上台阶俯仰下降超时
    .wait_after_pitch_fall_ms  = 200U,// 上台阶俯仰下降后等待时间
    .vy_rev_fast               = -120.0f,
    .vy_rev_fast_ms            = 0U,
    .vy_rev                    = -40.0f,// 下台阶后退 vy
    .laser_rev_timeout_ms      = 1500U,// 下台阶后退激光超时
    .after_clear_before_fall_ms = 100U,// 下台阶后退清除障碍后等待时间
    .wait_fall_done_ms         = 300U,// 下台阶俯仰下降后等待时间
};
/**取kfs流程参数*/
volatile ProcessGetKfsTune g_process_get_kfs_tune = {
    .spin_front_to_p2_ms = 300U,/* 前臂到p2经过时间 */
    .chassis_forward_ms = 1800U,/* 底盘前进经过时间 */
    .wait_after_chassis_forward_ms = 200U,/* 底盘前进停止后等待时间 */
    .wait_before_sucker_off_ms = 800U,
    .wait_after_sucker_off_ms = 0U,
    .wait_after_close_s1_ms = 0U,/* 吸盘放松后前臂下掉时间 */
    .wait_front_p2_done_ms =1500U,/* 大风车旋转前计时 */
    .spin_back_to_p1_ms = 500U,
    .vy_chassis_forward = 10.0f,/* 底盘前进 vy */
};

typedef enum
{
    upslope_step_idle = 0,
    upslope_step_goto_p1,
    upslope_step_wait_after_goto,
    upslope_step_yaw_to_zero,
    upslope_step_wait_roll_rise,
    upslope_step_wait_roll_fall,
    upslope_step_done
} UpSlopeStep;

static UpSlopeStep s_upslope_step = upslope_step_idle;
static uint32_t s_upslope_stage_ms = 0U;
static float s_upslope_pitch_abs_base = 0.0f;
static float s_upslope_pitch_abs_peak = 0.0f;
static uint8_t s_upslope_fall_confirm = 0U;
static uint8_t s_upslope_goto_latched = 0U;
static uint8_t s_upslope_yaw_latched = 0U;
static uint32_t s_upslope_goto_session = 0U;


static void process_flow_update_chassis_priority(void)
{
    uint8_t priority = PROCESS_FLOW_OVERRIDE_PRIORITY_LOW;

    if (((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) != 0U) &&
        process_flow_chassis_override.priority_vx > priority)
    {
        priority = process_flow_chassis_override.priority_vx;
    }
    if (((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VY) != 0U) &&
        process_flow_chassis_override.priority_vy > priority)
    {
        priority = process_flow_chassis_override.priority_vy;
    }
    if (((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VW) != 0U) &&
        process_flow_chassis_override.priority_vw > priority)
    {
        priority = process_flow_chassis_override.priority_vw;
    }
    process_flow_chassis_override.priority = priority;
}

uint8_t Process_Flow_ChassisOverrideCanWrite(uint8_t axis_mask, uint8_t priority)
{
    if (((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) != 0U) &&
        ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) != 0U) &&
        process_flow_chassis_override.priority_vx > priority)
    {
        return 0U;
    }
    if (((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VY) != 0U) &&
        ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VY) != 0U) &&
        process_flow_chassis_override.priority_vy > priority)
    {
        return 0U;
    }
    if (((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VW) != 0U) &&
        ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VW) != 0U) &&
        process_flow_chassis_override.priority_vw > priority)
    {
        return 0U;
    }
    return 1U;
}

void Process_Flow_SetChassisOverrideAxes(uint8_t axis_mask, uint8_t priority, float vx, float vy, float vw)
{
    if (Process_Flow_ChassisOverrideCanWrite(axis_mask, priority) == 0U)
    {
        return;
    }

    if ((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) != 0U)
    {
        process_flow_chassis_override.axis_mask |= PROCESS_FLOW_CHASSIS_OVERRIDE_VX;
        process_flow_chassis_override.priority_vx = priority;
        process_flow_chassis_override.vx = vx;
    }
    if ((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VY) != 0U)
    {
        process_flow_chassis_override.axis_mask |= PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
        process_flow_chassis_override.priority_vy = priority;
        process_flow_chassis_override.vy = vy;
    }
    if ((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VW) != 0U)
    {
        process_flow_chassis_override.axis_mask |= PROCESS_FLOW_CHASSIS_OVERRIDE_VW;
        process_flow_chassis_override.priority_vw = priority;
        process_flow_chassis_override.vw = vw;
    }
    process_flow_update_chassis_priority();
}

void Process_Flow_ClearChassisOverrideAxesByPriority(uint8_t axis_mask, uint8_t max_priority)
{
    if (((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) != 0U) &&
        (((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) == 0U) ||
         process_flow_chassis_override.priority_vx <= max_priority))
    {
        process_flow_chassis_override.axis_mask &= (uint8_t)(~PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
        process_flow_chassis_override.priority_vx = PROCESS_FLOW_OVERRIDE_PRIORITY_LOW;
        process_flow_chassis_override.vx = 0.0f;
    }
    if (((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VY) != 0U) &&
        (((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VY) == 0U) ||
         process_flow_chassis_override.priority_vy <= max_priority))
    {
        process_flow_chassis_override.axis_mask &= (uint8_t)(~PROCESS_FLOW_CHASSIS_OVERRIDE_VY);
        process_flow_chassis_override.priority_vy = PROCESS_FLOW_OVERRIDE_PRIORITY_LOW;
        process_flow_chassis_override.vy = 0.0f;
    }
    if (((axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VW) != 0U) &&
        (((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VW) == 0U) ||
         process_flow_chassis_override.priority_vw <= max_priority))
    {
        process_flow_chassis_override.axis_mask &= (uint8_t)(~PROCESS_FLOW_CHASSIS_OVERRIDE_VW);
        process_flow_chassis_override.priority_vw = PROCESS_FLOW_OVERRIDE_PRIORITY_LOW;
        process_flow_chassis_override.vw = 0.0f;
    }
    process_flow_update_chassis_priority();
}

void Process_Flow_ClearChassisOverrideAxes(uint8_t axis_mask)
{
    Process_Flow_ClearChassisOverrideAxesByPriority(axis_mask, PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH);
}
void Process_Flow_DebugSnapshot(void)
{
    if (process_flow_debug.enable == 0U) return;

    process_flow_debug.seq++;
    process_flow_debug.now_tick = osKernelGetTickCount();

    process_flow_debug.upstairs_step = (uint32_t)upstairs_step;
    process_flow_debug.downstairs_step = (uint32_t)downstairs_step;
    process_flow_debug.get_kfs_step = (uint32_t)get_kfs_step;
    process_flow_debug.put_kfs_step = (uint32_t)put_kfs_step;
    process_flow_debug.upslope_step = (uint32_t)s_upslope_step;

    process_flow_debug.lift_has_stopped = (uint32_t)lift_has_stopped;
    process_flow_debug.r2_lift_mode = (uint32_t)r2_lift_mode;
    process_flow_debug.lift_rise_fast = (uint32_t)lift_rise_fast;
    process_flow_debug.lift_fall_fast = (uint32_t)lift_fall_fast;
    process_flow_debug.lift_stop_mode = (uint32_t)lift_stop_mode;
    process_flow_debug.lift_running = (uint32_t)lift_running;

    process_flow_debug.axis_mask = (uint32_t)process_flow_chassis_override.axis_mask;
    process_flow_debug.priority = (uint32_t)process_flow_chassis_override.priority;
    process_flow_debug.priority_vx = (uint32_t)process_flow_chassis_override.priority_vx;
    process_flow_debug.priority_vy = (uint32_t)process_flow_chassis_override.priority_vy;
    process_flow_debug.priority_vw = (uint32_t)process_flow_chassis_override.priority_vw;
    process_flow_debug.vx = process_flow_chassis_override.vx;
    process_flow_debug.vy = process_flow_chassis_override.vy;
    process_flow_debug.vw = process_flow_chassis_override.vw;
}
    
void Process_Flow_ClearChassisOverride(void)
{
    Process_Flow_ClearChassisOverrideAxes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VX |
                                                PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                PROCESS_FLOW_CHASSIS_OVERRIDE_VW));
}

void Process_Flow_ResetAll(void)
{
    /* 抬升到位锁存由 manual_lift / process_flow_lift_command 管理，勿在此每周期清除 */
    odom_nav_goto_disarm();
    Process_Flow_ClearChassisOverride();
    upstairs_step = upstairs_step_chassis_forward_pre;
    downstairs_step = downstairs_step_idle;
    get_kfs_step = get_kfs_step_idle;
    /* upslope: reset on estop/remote so next auto upslope starts from idle */
    s_upslope_step = upslope_step_idle;
    s_upslope_stage_ms = 0U;
    s_upslope_pitch_abs_base = 0.0f;
    s_upslope_pitch_abs_peak = 0.0f;
    s_upslope_fall_confirm = 0U;
    s_upslope_goto_latched = 0U;
    s_upslope_yaw_latched  = 0U;
    s_upslope_goto_session = 0U;
    s_upstairs_busy = 0U;
    s_downstairs_busy = 0U;
    s_downstairs_pitch_abs_base = 0.0f;
    s_downstairs_pitch_abs_peak = 0.0f;
    s_downstairs_fall_confirm = 0U;
    s_get_kfs_busy = 0U;
    s_get_kfs_chassis_fwd_done = 0U;
    s_get_kfs_sucker_off_done = 0U;
    s_put_kfs_busy = 0U;
    put_kfs_step = put_kfs_step_idle;
    up_r1_step = up_r1_step_idle;
    s_up_r1_busy = 0U;
}

/* 流程 busy 期间每周期 HIGH 占 VY，防 odom 等低优先级写 override */
static void process_flow_hold_vy_high(float vy)
{
    Process_Flow_SetChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY,
                                        PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH,
                                        0.0f, vy, 0.0f);
}

/** GetKFS 前顶结束后尾段不再占 VY，便于二区摆头回中/换桩导航（Vx 摆头与 Vy 分轴） */
static void get_kfs_hold_vy_if_pre_tail(float vy)
{
    if (s_get_kfs_chassis_fwd_done != 0U)
        return;
    process_flow_hold_vy_high(vy);
}

/* 流程下发抬升方向前清除到位锁存，避免半自动重复写同模式仍走刹车分支 */
static void process_flow_lift_command(R2_lift_mode mode)
{
    lift_clear_stop_latch();
    r2_lift_mode = mode;
}

void Process_UpStairs(void)
{
    static uint32_t now_ms = 0U;

    switch (upstairs_step)
    {
        case upstairs_step_chassis_forward_pre:
            s_upstairs_busy = 1U;
            process_flow_hold_vy_high(g_process_upstairs_tune.vy_chassis_forward_pre);
            now_ms = osKernelGetTickCount();
            upstairs_step = upstairs_step_wait_chassis_forward_pre;
            break;

        case upstairs_step_wait_chassis_forward_pre:
            process_flow_hold_vy_high(g_process_upstairs_tune.vy_chassis_forward_pre);
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.chassis_forward_pre_ms)
            {
                process_flow_hold_vy_high(0.0f);
                upstairs_step = upstairs_step_idle;
            }
            break;

        case upstairs_step_idle:
            process_flow_hold_vy_high(0.0f);
            process_flow_lift_command(raise);

            upstairs_step = upstairs_step_wait_raise_done;
            now_ms = osKernelGetTickCount();
            break;

        case upstairs_step_wait_raise_done:
            process_flow_hold_vy_high(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.wait_raise_done_ms)
            {
                process_flow_hold_vy_high(g_process_upstairs_tune.vy_fast_before_fall);
                now_ms = osKernelGetTickCount();
                upstairs_step = upstairs_step_wait_fast_before_fall;
            }
            break;

        case upstairs_step_wait_fast_before_fall:
            process_flow_hold_vy_high(g_process_upstairs_tune.vy_fast_before_fall);
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.fast_before_fall_ms)
            {
                process_flow_hold_vy_high(g_process_upstairs_tune.vy_forward);
                now_ms = osKernelGetTickCount();
                upstairs_step = upstairs_step_wait_before_fall;
            }
            break;

        case upstairs_step_wait_before_fall:
            process_flow_hold_vy_high(g_process_upstairs_tune.vy_forward);
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.wait_before_fall_ms)
            {
                process_flow_hold_vy_high(0.0f);
                process_flow_lift_command(fall);
                lift_fall_fast = 1U;
                upstairs_step = upstairs_step_wait_fall_done;
                now_ms = osKernelGetTickCount();
            }
            break;

        case upstairs_step_wait_fall_done:
            process_flow_hold_vy_high(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.wait_fall_done_ms)
            {
                upstairs_step = upstairs_step_chassis_forward_post;
            }
            break;

        case upstairs_step_chassis_forward_post:
            process_flow_hold_vy_high(g_process_upstairs_tune.vy_chassis_forward_post);
            now_ms = osKernelGetTickCount();
            upstairs_step = upstairs_step_wait_chassis_forward_post;
            break;

        case upstairs_step_wait_chassis_forward_post:
            process_flow_hold_vy_high(g_process_upstairs_tune.vy_chassis_forward_post);
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.chassis_forward_post_ms)
            {
                process_flow_hold_vy_high(0.0f);
                flow_mode = flow_none;
                s_upstairs_busy = 0U;
                Process_Flow_ClearChassisOverride();
                upstairs_step = upstairs_step_chassis_forward_pre;
            }
            break;

        default:
            s_upstairs_busy = 0U;
            Process_Flow_ClearChassisOverride();
            upstairs_step = upstairs_step_chassis_forward_pre;
            break;
    }
}

uint8_t Process_UpStairs_IsBusy(void)
{
    return s_upstairs_busy;
}

void Process_DownStairs(void)
{
    static uint32_t now_ms = 0U;
    const float pitch_abs = fabsf(g_sensor_task_data.imu.pitch_deg);
    const volatile ProcessDownstairsTune *pt = &g_process_downstairs_tune;

    switch (downstairs_step)
    {
        case downstairs_step_idle:
            Laser_ClearSuddenIncrease(&laser1);
            s_downstairs_busy = 1U;
            process_flow_lift_command(raise);
            lift_rise_fast = 1U;
            lift_fall_fast = 0U;
            process_flow_hold_vy_high(pt->vy_backward);
            s_downstairs_pitch_abs_base = pitch_abs;
            s_downstairs_pitch_abs_peak = pitch_abs;
            s_downstairs_fall_confirm = 0U;
            now_ms = osKernelGetTickCount();
            downstairs_step = downstairs_step_wait_pitch_rise;
            break;

        case downstairs_step_wait_pitch_rise:
            process_flow_hold_vy_high(pt->vy_backward);
            if (pitch_abs > s_downstairs_pitch_abs_peak)
            {
                s_downstairs_pitch_abs_peak = pitch_abs;
            }
            if ((pitch_abs - s_downstairs_pitch_abs_base) >= pt->pitch_abs_rise_th_deg)
            {
                s_downstairs_fall_confirm = 0U;
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_wait_pitch_fall;
            }
            else if ((osKernelGetTickCount() - now_ms) >= pt->pitch_rise_timeout_ms)
            {
                process_flow_hold_vy_high(0.0f);
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_wait_after_clear_before_fall;
            }
            break;

        case downstairs_step_wait_pitch_fall:
            process_flow_hold_vy_high(pt->vy_backward);
            if (pitch_abs > s_downstairs_pitch_abs_peak)
            {
                s_downstairs_pitch_abs_peak = pitch_abs;
                s_downstairs_fall_confirm = 0U;
            }
            else if ((s_downstairs_pitch_abs_peak - pitch_abs) >= pt->pitch_abs_fall_th_deg)
            {
                if (s_downstairs_fall_confirm < 0xFFU)
                {
                    s_downstairs_fall_confirm++;
                }
            }
            else
            {
                s_downstairs_fall_confirm = 0U;
            }
            if (s_downstairs_fall_confirm >= pt->fall_confirm_cnt)
            {
                process_flow_hold_vy_high(0.0f);
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_wait_after_pitch_fall;
            }
            else if ((osKernelGetTickCount() - now_ms) >= pt->pitch_fall_timeout_ms)
            {
                process_flow_hold_vy_high(0.0f);
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_wait_after_clear_before_fall;
            }
            break;

        case downstairs_step_wait_after_pitch_fall:
            process_flow_hold_vy_high(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= pt->wait_after_pitch_fall_ms)
            {
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_vy_rev_until_sudden;
            }
            break;

        case downstairs_step_vy_rev_until_sudden:
        {
            uint8_t sudden = Laser_GetSuddenIncrease(&laser1);
            uint32_t elapsed = osKernelGetTickCount() - now_ms;

            /* 快退阶段: vy = -120 */
            if (elapsed < pt->vy_rev_fast_ms)
                process_flow_hold_vy_high(pt->vy_rev_fast);
            /* 慢退阶段: vy = -40 */
            else
                process_flow_hold_vy_high(pt->vy_rev);

            if (sudden != 0U)
            {
                Laser_ClearSuddenIncrease(&laser1);
            }
            /* 激光突变 或 总超时 → 停车 → 清障 → fall */
            if ((sudden != 0U) || (elapsed >= pt->laser_rev_timeout_ms))
            {
                process_flow_hold_vy_high(0.0f);
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_wait_after_clear_before_fall;
            }
            break;
        }

        case downstairs_step_wait_after_clear_before_fall:
            process_flow_hold_vy_high(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= pt->after_clear_before_fall_ms)
            {
                process_flow_lift_command(fall);
                lift_fall_fast = 1U;
                lift_rise_fast = 0U;
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_wait_fall_done;
            }
            break;

        case downstairs_step_wait_fall_done:
            process_flow_hold_vy_high(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= pt->wait_fall_done_ms)
            {
                flow_mode = flow_none;
                s_downstairs_busy = 0U;
                Process_Flow_ClearChassisOverride();
                downstairs_step = downstairs_step_idle;
            }
            break;

        default:
            s_downstairs_busy = 0U;
            Process_Flow_ClearChassisOverride();
            downstairs_step = downstairs_step_idle;
            break;
    }
}

void Process_UpR1(void)
{
    static uint32_t now_ms = 0U;

    switch (up_r1_step)
    {
        case up_r1_step_idle:
            s_up_r1_busy = 1U;
            process_flow_lift_command(raise);
            now_ms = osKernelGetTickCount();
            up_r1_step = up_r1_step_wait_raise;
            break;

        case up_r1_step_wait_raise:
            if ((osKernelGetTickCount() - now_ms) >= g_process_up_r1_tune.raise_wait_ms)
            {
                process_flow_hold_vy_high(g_process_up_r1_tune.fast_fwd_vy);
                now_ms = osKernelGetTickCount();
                up_r1_step = up_r1_step_fast_fwd;
            }
            break;

        case up_r1_step_fast_fwd:
            process_flow_hold_vy_high(g_process_up_r1_tune.fast_fwd_vy);
            if ((osKernelGetTickCount() - now_ms) >= g_process_up_r1_tune.fast_fwd_ms)
            {
                process_flow_hold_vy_high(g_process_up_r1_tune.slow_fwd_vy);
                now_ms = osKernelGetTickCount();
                up_r1_step = up_r1_step_slow_fwd;
            }
            break;

        case up_r1_step_slow_fwd:
            process_flow_hold_vy_high(g_process_up_r1_tune.slow_fwd_vy);
            if ((osKernelGetTickCount() - now_ms) >= g_process_up_r1_tune.slow_fwd_ms)
            {
                process_flow_hold_vy_high(0.0f);
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY);
                lift_fall_fast = 1U;
                process_flow_lift_command(fall);
                now_ms = osKernelGetTickCount();
                up_r1_step = up_r1_step_fall;
            }
            break;

        case up_r1_step_fall:
            up_r1_step = up_r1_step_wait_fall;
            break;

        case up_r1_step_wait_fall:
            if ((osKernelGetTickCount() - now_ms) >= g_process_up_r1_tune.fall_wait_ms)
            {
                process_flow_hold_vy_high(g_process_up_r1_tune.post_fwd_vy);
                now_ms = osKernelGetTickCount();
                up_r1_step = up_r1_step_post_fwd;
            }
            break;

        case up_r1_step_post_fwd:
            process_flow_hold_vy_high(g_process_up_r1_tune.post_fwd_vy);
            if ((osKernelGetTickCount() - now_ms) >= g_process_up_r1_tune.post_fwd_ms)
            {
                process_flow_hold_vy_high(0.0f);
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY);
                up_r1_step = up_r1_step_done;
            }
            break;

        case up_r1_step_done:
            Process_Flow_ClearChassisOverride();
            flow_mode = flow_none;
            s_up_r1_busy = 0U;
            up_r1_step = up_r1_step_idle;
            break;

        default:
            Process_Flow_ClearChassisOverride();
            flow_mode = flow_none;
            s_up_r1_busy = 0U;
            up_r1_step = up_r1_step_idle;
            break;
    }
}

uint8_t Process_UpR1_IsBusy(void)
{
    return s_up_r1_busy;
}

uint8_t Process_DownStairs_IsBusy(void)
{
    return s_downstairs_busy;
}

static Main_lift_position process_get_kfs_main_lift_high(app_zone2_get_kfs_rel_t rel)
{
    if (rel == APP_ZONE2_GET_KFS_GROUND_HIGHEST)
        return main_lift_p4;
    if (rel == APP_ZONE2_GET_KFS_GROUND)
        return main_lift_p2;
    return main_lift_p3;
}

void Process_GetKFS(app_zone2_get_kfs_rel_t rel)
{
    static uint32_t now_ms = 0U;
    static uint8_t get_kfs_round = 0U; /* 0: first entry force p1; 1: normal */
    static Three_kfs_position start_three_pos = three_kfs_p1;

    switch (get_kfs_step)
    {
        case get_kfs_step_idle:
            s_get_kfs_busy = 1U;
            s_get_kfs_chassis_fwd_done = 0U;
            s_get_kfs_sucker_off_done = 0U;
            Laser_ClearSuddenIncrease(&laser1);
            get_kfs_hold_vy_if_pre_tail(0.0f);
            /* Only first entry forces p1; later entries keep current position */
            if (get_kfs_round == 0U)
            {
                three_kfs_position = three_kfs_p1;
            }

            start_three_pos = three_kfs_position;
            /* 高桩取低 p0；低桩取高 p3；地面预备最高档 p4 */
            if (rel == APP_ZONE2_GET_KFS_HIGH_TO_LOW)
            {
                main_lift_position = main_lift_p0;
            }
            else if (rel == APP_ZONE2_GET_KFS_GROUND_HIGHEST)
            {
                main_lift_position = main_lift_p4;
            }
            else if (rel == APP_ZONE2_GET_KFS_GROUND)
            {
                main_lift_position = main_lift_p2;
            }
            else
            {
                main_lift_position = main_lift_p3;
            }
            if (start_three_pos == three_kfs_p1)
            {
                sucker1_state = 1U;
                sucker2_state = 1U;
            }
            else if (start_three_pos == three_kfs_p2)
            {
                sucker1_state = 1U;
                sucker3_state = 1U;
            }
            else if (start_three_pos == three_kfs_p3)
            {
                sucker1_state = 1U;
                sucker4_state = 1U;
            }
            else if (start_three_pos == three_kfs_p4)
            {
                sucker2_state = 1U;
                sucker3_state = 1U;
                sucker4_state = 1U;
            }

            kfs_spin_position = kfs_spin_p2;
            kfs_below_position = kfs_below_cmd_p1;

            now_ms = osKernelGetTickCount();
            get_kfs_step = get_kfs_step_spin_front_to_p2;
            break;

        case get_kfs_step_spin_front_to_p2:
            get_kfs_hold_vy_if_pre_tail(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.spin_front_to_p2_ms)
            {
                process_flow_hold_vy_high(g_process_get_kfs_tune.vy_chassis_forward);
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_chassis_forward;
            }
            break;

        case get_kfs_step_chassis_forward:
            process_flow_hold_vy_high(g_process_get_kfs_tune.vy_chassis_forward);
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.chassis_forward_ms ||
                Laser_GetSuddenIncrease(&laser1) != 0U)
            {
                if (Laser_GetSuddenIncrease(&laser1) != 0U)
                    Laser_ClearSuddenIncrease(&laser1);
                process_flow_hold_vy_high(0.0f);
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY);
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_wait_after_chassis_forward;
            }
            break;

        case get_kfs_step_wait_after_chassis_forward:
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.wait_after_chassis_forward_ms)
            {
                /* 设位置: 旋转到P1、主升降、KFS下方 */
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY);
                kfs_spin_position = kfs_spin_p1;
                // s_get_kfs_chassis_fwd_done = 1U;
                if (rel == APP_ZONE2_GET_KFS_HIGH_TO_LOW)
                    main_lift_position = main_lift_p1;
                else
                    main_lift_position = process_get_kfs_main_lift_high(rel);
                kfs_below_position = kfs_below_cmd_p2;
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_wait_after_sucker_off;
            }
            break;

        case get_kfs_step_wait_after_sucker_off:
            /* 阶段A: 等320s 关吸盘 */
            if (s_get_kfs_sucker_off_done == 0U)
            {
                if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.wait_before_sucker_off_ms)
                {
                    sucker1_state = 0U;
                    s_get_kfs_sucker_off_done = 1U;
                    now_ms = osKernelGetTickCount();
                }
            }
            /* 阶段B: 再等230s 清override + 标记完成 */
            else
            {
                if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.wait_after_sucker_off_ms)
                {
                    Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY);
                    // s_get_kfs_chassis_fwd_done = 1U;
                    s_get_kfs_sucker_off_done = 0U;
                    now_ms = osKernelGetTickCount();
                    get_kfs_step = get_kfs_step_wait_after_close_s1;
                }
            }
            break;

        case get_kfs_step_wait_after_close_s1:
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.wait_after_close_s1_ms)
            {
                kfs_spin_position = kfs_spin_p3;
                s_get_kfs_chassis_fwd_done = 1U;
                if (rel != APP_ZONE2_GET_KFS_HIGH_TO_LOW && rel != APP_ZONE2_GET_KFS_LOW_TO_HIGH)
                    main_lift_position = process_get_kfs_main_lift_high(rel);
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_wait_front_p2_done;
            }
            break;


        case get_kfs_step_wait_front_p2_done:
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.wait_front_p2_done_ms)
            {
                if (start_three_pos == three_kfs_p1)
                {
                    three_kfs_position = three_kfs_p2;
                }
                else if (start_three_pos == three_kfs_p2)
                {
                    three_kfs_position = three_kfs_p3;
                }
                else if (start_three_pos == three_kfs_p3)
                {
                    three_kfs_position = three_kfs_p4;
                }
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_spin_back_to_p1;
            }
            break;

        case get_kfs_step_spin_back_to_p1:
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.spin_back_to_p1_ms)
            {
                kfs_spin_position = kfs_spin_p1;
                get_kfs_step = get_kfs_step_done;
            }
            break;

        case get_kfs_step_done:
            Process_Flow_ClearChassisOverride();
            kfs_below_position = kfs_below_cmd_stop;
            flow_mode = flow_none;
            s_get_kfs_busy = 0U;
            s_get_kfs_chassis_fwd_done = 0U;
            s_get_kfs_sucker_off_done = 0U;
            get_kfs_step = get_kfs_step_idle;
            get_kfs_round = 1U;
            break;

        default:
            Process_Flow_ClearChassisOverride();
            flow_mode = flow_none;
            s_get_kfs_busy = 0U;
            s_get_kfs_chassis_fwd_done = 0U;
            s_get_kfs_sucker_off_done = 0U;
            get_kfs_step = get_kfs_step_idle;
            break;
    }

    process_flow_debug.get_kfs_round = (uint32_t)get_kfs_round;
}

uint8_t Process_GetKFS_IsBusy(void)
{
    return s_get_kfs_busy;
}

uint8_t Process_GetKFS_IsChassisForwardDone(void)
{
    return (uint8_t)((s_get_kfs_busy != 0U) && (s_get_kfs_chassis_fwd_done != 0U));
}

uint8_t Process_PutKFS_IsBusy(void)
{
    return s_put_kfs_busy;
}

void Process_PutKFS(void)
{
    static uint32_t now_ms = 0U;

    switch (put_kfs_step)
    {
        case put_kfs_step_idle:
            s_put_kfs_busy = 1U;
            Process_Flow_ClearChassisOverride();

            /* kfs_above 伸出到 P3 的同时关吸盘 */
            kfs_above_position = kfs_above_cmd_p3;
            if (three_kfs_position == three_kfs_p1)
                sucker2_state = 0U;
            else if (three_kfs_position == three_kfs_p2)
                sucker3_state = 0U;
            else if (three_kfs_position == three_kfs_p3)
                sucker4_state = 0U;

            now_ms = osKernelGetTickCount();
            put_kfs_step = put_kfs_step_extend;
            break;

        case put_kfs_step_extend:
            /* 等 P3 伸出到位(默认2s)，同时底盘锁死 */
            Process_Flow_ClearChassisOverride();
            if ((osKernelGetTickCount() - now_ms) >= g_process_put_kfs_tune.wait_extend_ms)
            {
                kfs_above_position = kfs_above_cmd_p1;
                now_ms = osKernelGetTickCount();
                put_kfs_step = put_kfs_step_retract;
            }
            break;

        case put_kfs_step_retract:
            /* P1 缩回(默认1s)同时释放底盘，可导航回P1 */
            Process_Flow_ClearChassisOverride();
            if ((osKernelGetTickCount() - now_ms) >= g_process_put_kfs_tune.wait_retract_ms)
            {
                if (three_kfs_position > three_kfs_p1)
                    three_kfs_position = (Three_kfs_position)((uint8_t)three_kfs_position - 1U);
                put_kfs_step = put_kfs_step_done;
            }
            break;

        case put_kfs_step_done:
            Process_Flow_ClearChassisOverride();
            flow_mode = flow_none;
            s_put_kfs_busy = 0U;
            put_kfs_step = put_kfs_step_idle;
            break;

        default:
            Process_Flow_ClearChassisOverride();
            flow_mode = flow_none;
            s_put_kfs_busy = 0U;
            put_kfs_step = put_kfs_step_idle;
            break;
    }
}
void Process_PutKFS_AbortAndRollback(void)
{
    if (s_put_kfs_busy == 0U) return;

    if (put_kfs_step == put_kfs_step_retract
        || put_kfs_step == put_kfs_step_done)
    {
        if      (three_kfs_position == three_kfs_p1) sucker2_state = 1U;
        else if (three_kfs_position == three_kfs_p2) sucker3_state = 1U;
        else if (three_kfs_position == three_kfs_p3) sucker4_state = 1U;
    }

    kfs_above_position = kfs_above_cmd_p1;
    put_kfs_step = put_kfs_step_idle;
    s_put_kfs_busy = 0U;
    flow_mode = flow_none;
    Process_Flow_ClearChassisOverride();
}

void Process_UpSlope(void)
{
    const uint32_t now_ms = osKernelGetTickCount();//获取当前时间戳
        const float pitch_abs = fabsf(g_sensor_task_data.imu.pitch_deg); /* 上坡检测：俯仰角绝对值（度） */
    odom_nav_goto_err_t nav_rc;

    /* 如果当前步骤不是空闲状态且不是完成状态，且当前时间戳减去上次步骤开始时间大于阶段超时时间，则清除底盘覆盖并设置步骤为空闲状态，并设置自动模式为无自动模式 */
    if (s_upslope_step != upslope_step_idle &&
        s_upslope_step != upslope_step_done &&
        (now_ms - s_upslope_stage_ms) > g_process_upslope_tune.stage_timeout_ms)
    {
        Process_Flow_ClearChassisOverride();
        s_upslope_step = upslope_step_idle;
        flow_mode = flow_none;
        return;
    }

    switch (s_upslope_step)
    {
        case upslope_step_idle:
            s_upslope_pitch_abs_base = pitch_abs;
            s_upslope_pitch_abs_peak = pitch_abs;
            s_upslope_fall_confirm = 0U;/* 设置横滚角下降确认次数为0 */
            s_upslope_goto_latched = 0U;
            s_upslope_yaw_latched  = 0U; /* 转向只发一次 */
            s_upslope_goto_session = 0U;
            s_upslope_stage_ms = now_ms;
            s_upslope_step = upslope_step_goto_p1;/* 设置步骤为goto_p1 */
            break;

        case upslope_step_goto_p1:
            /* 到点阶段：主轴 p1 + 三轴 p4（每周期保持） */
            main_lift_position = main_lift_p2;
            kfs_below_position = kfs_below_cmd_p3;
            three_kfs_position = three_kfs_p4;
            if (s_upslope_goto_latched == 0U)
            {
                odom_nav_goto_set_target(g_process_upslope_tune.p1_x_m, g_process_upslope_tune.p1_y_m);
                s_upslope_goto_session = odom_nav_target.session_id;
                s_upslope_goto_latched = 1U;
                YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_FRONT);
                s_upslope_yaw_latched = 1U;
                break; /* 刚发目标，跳过本帧结果检查，避免读到旧导航的残留 ARRIVED */
            }
            nav_rc = odom_nav_goto_peek_last_run_result();
            if (odom_nav_target.session_id != s_upslope_goto_session)
            {
                nav_rc = ODOM_NAV_GOTO_ERR_DISARMED;
            }
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                if (YawHeadingCtrl_IsBusy() == 0U)
                {
                    Process_Flow_ClearChassisOverride();
                    s_upslope_pitch_abs_base = pitch_abs;
                    s_upslope_pitch_abs_peak = pitch_abs;
                    s_upslope_fall_confirm = 0U;
                    s_upslope_yaw_latched = 0U;
                    s_upslope_stage_ms = now_ms;
                    s_upslope_step = upslope_step_wait_roll_rise;
                }
                else
                {
                    Process_Flow_ClearChassisOverride();
                    s_upslope_step = upslope_step_yaw_to_zero;
                }
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED))
            {
                Process_Flow_ClearChassisOverride();
                odom_nav_goto_disarm();
                s_upslope_step = upslope_step_idle;
                flow_mode = flow_none;
            }
            break;

        case upslope_step_wait_after_goto:
            if ((now_ms - s_upslope_stage_ms) >= g_process_upslope_tune.wait_after_goto_ms)
            {
                s_upslope_stage_ms = now_ms;
                s_upslope_step = upslope_step_yaw_to_zero;
            }
            break;

        case upslope_step_yaw_to_zero:
            if (s_upslope_yaw_latched == 0U)
            {
                YawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_FRONT);
                s_upslope_yaw_latched = 1U;
                break; /* 刚发指令，跳过本帧检查 */
            }
            if (YawHeadingCtrl_IsBusy() == 0U)
            {
                Process_Flow_ClearChassisOverride();
                s_upslope_pitch_abs_base = pitch_abs;
                s_upslope_pitch_abs_peak = pitch_abs;
                s_upslope_fall_confirm = 0U;
                s_upslope_yaw_latched = 0U;
                s_upslope_stage_ms = now_ms;
                s_upslope_step = upslope_step_wait_roll_rise;
            }
            break;

        case upslope_step_wait_roll_rise:
            process_flow_hold_vy_high(g_process_upslope_tune.vy_target);
            if (pitch_abs > s_upslope_pitch_abs_peak) s_upslope_pitch_abs_peak = pitch_abs;
            if ((pitch_abs - s_upslope_pitch_abs_base) >= g_process_upslope_tune.pitch_abs_rise_th_deg)
            {
                s_upslope_stage_ms = now_ms;
                s_upslope_step = upslope_step_wait_roll_fall;
            }
            break;

        case upslope_step_wait_roll_fall:
            process_flow_hold_vy_high(g_process_upslope_tune.vy_target);
            if (pitch_abs > s_upslope_pitch_abs_peak)
            {
                s_upslope_pitch_abs_peak = pitch_abs;
                s_upslope_fall_confirm = 0U;
            }
            else if ((s_upslope_pitch_abs_peak - pitch_abs) >= g_process_upslope_tune.pitch_abs_fall_th_deg)
            {
                if (s_upslope_fall_confirm < 0xFFU) s_upslope_fall_confirm++;
            }
            else
            {
                s_upslope_fall_confirm = 0U;
            }
            if (s_upslope_fall_confirm >= g_process_upslope_tune.fall_confirm_cnt)
            {
                Process_Flow_ClearChassisOverride();
                s_upslope_stage_ms = now_ms;
                s_upslope_step = upslope_step_done;
            }
            break;

        case upslope_step_done:
            flow_mode = flow_none;
            s_upslope_step = upslope_step_idle;
            break;

        default:
            Process_Flow_ClearChassisOverride();
            flow_mode = flow_none;
            s_upslope_step = upslope_step_idle;
            break;
    }
}

uint8_t Process_UpSlope_IsBusy(void)
{
    return (uint8_t)(s_upslope_step != upslope_step_idle &&
                     s_upslope_step != upslope_step_done);
}

void Process_UpSlope_Reset(void)
{
    s_upslope_step = upslope_step_idle;
    s_upslope_stage_ms = 0U;
    s_upslope_pitch_abs_base = 0.0f;
    s_upslope_pitch_abs_peak = 0.0f;
    s_upslope_fall_confirm = 0U;
    s_upslope_goto_latched = 0U;
    s_upslope_yaw_latched  = 0U;
    s_upslope_goto_session = 0U;
}