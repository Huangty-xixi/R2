#include "Process_Flow.h"
#include "app_init.h"
#include "app_yaw_heading_ctrl.h"
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
volatile ProcessFlowDebug process_flow_debug = {1U};

/** 1=Process_UpStairs mid-cycle (zone2 or flow_upstairs tick) */
static uint8_t s_upstairs_busy;
/** 1=Process_DownStairs mid-cycle */
static uint8_t s_downstairs_busy;
/** Plan0 下台阶俯仰检测 */
static float s_downstairs_pitch_abs_base = 0.0f;
static float s_downstairs_pitch_abs_peak = 0.0f;
static uint8_t s_downstairs_fall_confirm = 0U;
/** 1=Process_GetKFS mid-cycle */
static uint8_t s_get_kfs_busy;
/** 1=chassis_forward 已结束，后半段（spin_p1/吸盘/主轴/三轴）仍在跑 */
static uint8_t s_get_kfs_chassis_fwd_done;

/**上坡流程参数*/
volatile ProcessUpSlopeTune g_process_upslope_tune = {
    .p1_x_m = 5.4f,
    .p1_y_m = 6.73f,
    .yaw_tol_deg = 1.0f,
    .vy_target = 80.0f,
    .wait_after_goto_ms = 1000U,
    .pitch_abs_rise_th_deg = 10.0f,
    .pitch_abs_fall_th_deg = 10.0f,
    .fall_confirm_cnt = 3U,
    .stage_timeout_ms = 60000U,
};

/**上台阶流程参数*/
volatile ProcessUpstairsTune g_process_upstairs_tune = {
    .chassis_forward_pre_ms = 1500U,/* 抬升前底盘前进时间 */
    .vy_chassis_forward_pre = 20.0f,/* 抬升前底盘前进 vy */
    .wait_raise_done_ms = 1500U,/* 上升等待时间 */
    .wait_before_fall_ms = 1750U,/* 下降前等待时间 */
    .wait_fall_done_ms = 1500U,
    .vy_forward = 50.0f,/* 上台阶纵向速度 */
    .chassis_forward_post_ms = 1500U,/* 落台等待结束后前进时间 */
    .vy_chassis_forward_post = 10.0f,/* 落台等待结束后前进 vy */
};

/**下台阶流程参数*/
volatile ProcessDownstairsTune g_process_downstairs_tune = {
    .fast_raise_back_ms = 1200U,/* 俯仰回落后再后退经过时间 */
    .stop_before_fall_ms = 1000U,/* 无用*/
    .wait_fall_done_ms = 100U,/* 无用*/
    .vy_backward = -50.0f,
    .pitch_abs_rise_th_deg = 10.0f,
    .pitch_abs_fall_th_deg = 10.0f,
    .fall_confirm_cnt = 3U,
    .wait_after_pitch_fall_ms = 500U,
    .vy_backward_after_pitch = -40.0f,
};

/** Plan B 下台阶：计时与 vy（Watch 在线改） */
volatile ProcessDownstairsPlanBTune g_process_downstairs_plan_b_tune = {
    .vy_rev_first_ms = 3050U,
    .wait_after_sudden_stop_ms = 500U,
    .raise_hold_ms = 1500U,
    .vy_rev_second_ms = 3500U,
    .after_clear_before_fall_ms = 1000U,
    .fall_hold_ms = 1000U,
    .vy_rev = -10.0f,
    .vy_rev_after_raise = -20.0f,
};

/** Plan C 下台阶：先前进再后退（timed），参数同早期 PlanB */
volatile ProcessDownstairsPlanCTune g_process_downstairs_plan_c_tune = {
    .vy_fwd_ms = 3000U,
    .vy_rev_first_ms = 3700U,
    .raise_hold_ms = 1500U,
    .vy_rev_second_ms = 3500U,
    .after_clear_before_fall_ms = 1000U,
    .fall_hold_ms = 1000U,
    .vy_fwd = 10.0f,
    .vy_rev = -10.0f,
    .vy_rev_after_raise = -20.0f,
};

/**取kfs流程参数*/
volatile ProcessGetKfsTune g_process_get_kfs_tune = {
    .spin_front_to_p2_ms = 1200U,/* 前臂到p2经过时间 */
    .chassis_forward_ms = 1500U,/* 底盘前进经过时间 */
    .spin_front_to_p1_ms = 1200U,/* 前臂到p1和吸盘吸kfs经过时间 */
    .wait_after_close_s1_ms = 2000U,/* 吸盘放松后前臂下掉时间 */
    .wait_main_lift_p1_ms = 1200U,/* 主轴到p3时间 */
    .wait_front_p2_done_ms = 200U,/* 无用 */
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
    lift_clear_stop_latch();
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
    s_upslope_goto_session = 0U;
    s_upstairs_busy = 0U;
    s_downstairs_busy = 0U;
    s_downstairs_pitch_abs_base = 0.0f;
    s_downstairs_pitch_abs_peak = 0.0f;
    s_downstairs_fall_confirm = 0U;
    s_get_kfs_busy = 0U;
    s_get_kfs_chassis_fwd_done = 0U;
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

#if (PROCESS_FLOW_DOWNSTAIRS_PLAN == 0)
    {
        const float pitch_abs = fabsf(g_sensor_task_data.imu.pitch_deg);

    switch (downstairs_step)
    {
        case downstairs_step_idle:
            s_downstairs_busy = 1U;
            process_flow_lift_command(raise);
            lift_rise_fast = 1U;
            lift_fall_fast = 0U;
            process_flow_hold_vy_high(g_process_downstairs_tune.vy_backward);
            s_downstairs_pitch_abs_base = pitch_abs;
            s_downstairs_pitch_abs_peak = pitch_abs;
            s_downstairs_fall_confirm = 0U;
            downstairs_step = downstairs_step_wait_pitch_rise;
            break;

        case downstairs_step_wait_pitch_rise:
            process_flow_hold_vy_high(g_process_downstairs_tune.vy_backward);
            if (pitch_abs > s_downstairs_pitch_abs_peak)
            {
                s_downstairs_pitch_abs_peak = pitch_abs;
            }
            if ((pitch_abs - s_downstairs_pitch_abs_base) >= g_process_downstairs_tune.pitch_abs_rise_th_deg)
            {
                s_downstairs_fall_confirm = 0U;
                downstairs_step = downstairs_step_wait_pitch_fall;
            }
            break;

        case downstairs_step_wait_pitch_fall:
            process_flow_hold_vy_high(g_process_downstairs_tune.vy_backward);
            if (pitch_abs > s_downstairs_pitch_abs_peak)
            {
                s_downstairs_pitch_abs_peak = pitch_abs;
                s_downstairs_fall_confirm = 0U;
            }
            else if ((s_downstairs_pitch_abs_peak - pitch_abs) >= g_process_downstairs_tune.pitch_abs_fall_th_deg)
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
            if (s_downstairs_fall_confirm >= g_process_downstairs_tune.fall_confirm_cnt)
            {
                process_flow_hold_vy_high(0.0f);
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_wait_after_pitch_fall;
            }
            break;

        case downstairs_step_wait_after_pitch_fall:
            process_flow_hold_vy_high(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.wait_after_pitch_fall_ms)
            {
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_fast_raise_back;
            }
            break;

        case downstairs_step_fast_raise_back:
            process_flow_hold_vy_high(g_process_downstairs_tune.vy_backward_after_pitch);
            if ((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.fast_raise_back_ms)
            {
                process_flow_hold_vy_high(0.0f);
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_stop_before_fall;
            }
            break;

        case downstairs_step_stop_before_fall:
            process_flow_hold_vy_high(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.stop_before_fall_ms)
            {
                process_flow_lift_command(fall);
                lift_fall_fast = 1U;
                downstairs_step = downstairs_step_wait_fall_done;
                now_ms = osKernelGetTickCount();
            }
            break;

        case downstairs_step_wait_fall_done:
            process_flow_hold_vy_high(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.wait_fall_done_ms)
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

#elif (PROCESS_FLOW_DOWNSTAIRS_PLAN == 2) /* Plan C：先前进 timed → 再后退 timed → 抬升 → 再退 → 快降 */

    {
        const volatile ProcessDownstairsPlanCTune *pc = &g_process_downstairs_plan_c_tune;

        switch (downstairs_step)
        {
            case downstairs_step_idle:
                s_downstairs_busy = 1U;
                process_flow_hold_vy_high(pc->vy_fwd);
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_c_vy_fwd;
                break;

            case downstairs_step_c_vy_fwd:
                process_flow_hold_vy_high(pc->vy_fwd);
                if ((osKernelGetTickCount() - now_ms) >= pc->vy_fwd_ms)
                {
                    process_flow_hold_vy_high(pc->vy_rev);
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_c_vy_rev_first;
                }
                break;

            case downstairs_step_c_vy_rev_first:
                process_flow_hold_vy_high(pc->vy_rev);
                if ((osKernelGetTickCount() - now_ms) >= pc->vy_rev_first_ms)
                {
                    process_flow_hold_vy_high(0.0f);
                    process_flow_lift_command(raise);
                    lift_rise_fast = 0U;
                    lift_fall_fast = 0U;
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_c_raise_hold;
                }
                break;

            case downstairs_step_c_raise_hold:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= pc->raise_hold_ms)
                {
                    process_flow_hold_vy_high(pc->vy_rev_after_raise);
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_c_vy_rev_second;
                }
                break;

            case downstairs_step_c_vy_rev_second:
                process_flow_hold_vy_high(pc->vy_rev_after_raise);
                if ((osKernelGetTickCount() - now_ms) >= pc->vy_rev_second_ms)
                {
                    process_flow_hold_vy_high(0.0f);
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_c_wait_before_fall;
                }
                break;

            case downstairs_step_c_wait_before_fall:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= pc->after_clear_before_fall_ms)
                {
                    process_flow_lift_command(fall);
                    lift_fall_fast = 0U;
                    lift_rise_fast = 0U;
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_c_fall_hold;
                }
                break;

            case downstairs_step_c_fall_hold:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= pc->fall_hold_ms)
                {
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_wait_fall_done;
                }
                break;

            case downstairs_step_wait_fall_done:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.wait_fall_done_ms)
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

#else /* Plan B：后退至测距突增停车 + 等待 + 抬升 + 再退 + 快降 */

    {
        const volatile ProcessDownstairsPlanBTune *pb = &g_process_downstairs_plan_b_tune;

        switch (downstairs_step)
        {
            case downstairs_step_idle:
                s_downstairs_busy = 1U;
                process_flow_hold_vy_high(pb->vy_rev);
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_b_vy_rev_until_sudden;
                break;

            case downstairs_step_b_vy_rev_until_sudden:
                process_flow_hold_vy_high(pb->vy_rev);
                if (Laser_GetSuddenIncrease(&laser1) != 0U)
                {
                    Laser_ClearSuddenIncrease(&laser1);
                    process_flow_hold_vy_high(0.0f);
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_b_wait_after_sudden_stop;
                }
                else if ((osKernelGetTickCount() - now_ms) >= pb->vy_rev_first_ms)
                {
                    process_flow_hold_vy_high(0.0f);
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_b_wait_after_sudden_stop;
                }
                break;

            case downstairs_step_b_wait_after_sudden_stop:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= pb->wait_after_sudden_stop_ms)
                {
                    process_flow_lift_command(raise);
                    lift_rise_fast = 0U;
                    lift_fall_fast = 0U;
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_b_raise_hold_15s;
                }
                break;

            case downstairs_step_b_raise_hold_15s:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= pb->raise_hold_ms)
                {
                    process_flow_hold_vy_high(pb->vy_rev_after_raise);
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_b_vy_rev_2s;
                }
                break;

            case downstairs_step_b_vy_rev_2s:
                process_flow_hold_vy_high(pb->vy_rev_after_raise);
                if ((osKernelGetTickCount() - now_ms) >= pb->vy_rev_second_ms)
                {
                    process_flow_hold_vy_high(0.0f);
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_b_wait_after_clear_before_fall;
                }
                break;

            case downstairs_step_b_wait_after_clear_before_fall:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= pb->after_clear_before_fall_ms)
                {
                    process_flow_lift_command(fall);
                    lift_fall_fast = 1U;
                    lift_rise_fast = 0U;
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_b_fall_hold_1s;
                }
                break;

            case downstairs_step_b_fall_hold_1s:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= pb->fall_hold_ms)
                {
                    now_ms = osKernelGetTickCount();
                    downstairs_step = downstairs_step_wait_fall_done;
                }
                break;

            case downstairs_step_wait_fall_done:
                process_flow_hold_vy_high(0.0f);
                if ((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.wait_fall_done_ms)
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

#endif /* PROCESS_FLOW_DOWNSTAIRS_PLAN */
}

uint8_t Process_DownStairs_IsBusy(void)
{
    return s_downstairs_busy;
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
            get_kfs_hold_vy_if_pre_tail(0.0f);
            /* Only first entry forces p1; later entries keep current position */
            if (get_kfs_round == 0U)
            {
                three_kfs_position = three_kfs_p1;
            }

            start_three_pos = three_kfs_position;
            /* 高桩取低 → 主轴低位 p0；低桩取高 → 主轴高位 p3 */
            if (rel == APP_ZONE2_GET_KFS_HIGH_TO_LOW)
            {
                main_lift_position = main_lift_p0;
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

            now_ms = osKernelGetTickCount();
            get_kfs_step = get_kfs_step_spin_front_to_p2;
            break;

        case get_kfs_step_spin_front_to_p2:
            get_kfs_hold_vy_if_pre_tail(0.0f);
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.spin_front_to_p2_ms)
            {
                process_flow_hold_vy_high(10.0f);
                now_ms = osKernelGetTickCount();
                kfs_spin_position = kfs_spin_p2;
                get_kfs_step = get_kfs_step_chassis_forward;
            }
            break;

        case get_kfs_step_chassis_forward:
            process_flow_hold_vy_high(10.0f);
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.chassis_forward_ms)
            {
                Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VY);
                kfs_spin_position = kfs_spin_p1;
                main_lift_position = main_lift_p1;
                s_get_kfs_chassis_fwd_done = 1U;
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_spin_front_to_p1;
            }
            break;

        case get_kfs_step_spin_front_to_p1:
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.spin_front_to_p1_ms)
            {
                sucker1_state = 0U;
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_wait_after_close_s1;
            }
            break;

        case get_kfs_step_wait_after_close_s1:
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.wait_after_close_s1_ms)
            {
                kfs_spin_position = kfs_spin_p2;
                main_lift_position = main_lift_p3;
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_main_lift_to_p1;
            }
            break;

        case get_kfs_step_main_lift_to_p1:
            main_lift_position = main_lift_p3;
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.wait_main_lift_p1_ms)
            {
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
                get_kfs_step = get_kfs_step_done;
            }
            break;

        case get_kfs_step_done:
            Process_Flow_ClearChassisOverride();
            flow_mode = flow_none;
            s_get_kfs_busy = 0U;
            s_get_kfs_chassis_fwd_done = 0U;
            get_kfs_step = get_kfs_step_idle;
            get_kfs_round = 1U;
            break;

        default:
            Process_Flow_ClearChassisOverride();
            flow_mode = flow_none;
            s_get_kfs_busy = 0U;
            s_get_kfs_chassis_fwd_done = 0U;
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

void Process_PutKFS(void)
{
    Process_Flow_ClearChassisOverride();
    flow_mode = flow_none;
    Process_Flow_DebugSnapshot();
}

void Process_UpSlope(void)
{
    const uint32_t now_ms = osKernelGetTickCount();//获取当前时间戳
    float yaw_now = g_sensor_task_data.imu.yaw_deg;//获取当前航向
    float yaw_err_abs = 0.0f;//航向误差绝对值
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
            s_upslope_goto_latched = 0U;/* 进入到点阶段时只下发一次目标 */
            s_upslope_goto_session = 0U;
            s_upslope_stage_ms = now_ms;
            s_upslope_step = upslope_step_goto_p1;/* 设置步骤为goto_p1 */
            break;

        case upslope_step_goto_p1:
            /* 到点阶段：主轴 p1 + 三轴 p4（每周期保持） */
            main_lift_position = main_lift_p1;
            three_kfs_position = three_kfs_p4;
            if (s_upslope_goto_latched == 0U)
            {
                odom_nav_goto_set_target(g_process_upslope_tune.p1_x_m, g_process_upslope_tune.p1_y_m);
                s_upslope_goto_session = odom_nav_target.session_id;
                s_upslope_goto_latched = 1U;
            }
            nav_rc = odom_nav_goto_peek_last_run_result();
            if (odom_nav_target.session_id != s_upslope_goto_session)
            {
                nav_rc = ODOM_NAV_GOTO_ERR_DISARMED;
            }
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                s_upslope_stage_ms = now_ms;
                s_upslope_step = upslope_step_wait_after_goto;
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
            AppYawHeadingCtrl_RunFieldDir(APP_ZONE2_FIELD_FRONT);
            yaw_err_abs = fabsf(wrap_deg_180(0.0f - yaw_now));
            if (yaw_err_abs <= g_process_upslope_tune.yaw_tol_deg)
            {
                Process_Flow_ClearChassisOverride();
                s_upslope_pitch_abs_base = pitch_abs;
                s_upslope_pitch_abs_peak = pitch_abs;
                s_upslope_fall_confirm = 0U;
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
