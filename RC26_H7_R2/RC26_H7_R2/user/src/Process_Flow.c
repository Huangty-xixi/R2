#include "Process_Flow.h"
#include "Motion_Task.h"
#include "lift.h"
#include "kfs.h"
#include "weapon.h"
#include "Sensor_Task.h"
#include "odom_nav_goto.h"
#include "upper_pc_protocol.h"
#include "cmsis_os.h"
#include "common.h"
#include <math.h>

ProcessFlowChassisOverride process_flow_chassis_override = {0U, PROCESS_FLOW_OVERRIDE_PRIORITY_LOW, 0.0f, 0.0f, 0.0f};
UpstairsStep upstairs_step = upstairs_step_idle;
DownstairsStep downstairs_step = downstairs_step_idle;
GetKfsStep get_kfs_step = get_kfs_step_idle;
volatile ProcessFlowDebug process_flow_debug = {1U};


/**上坡流程参数*/
volatile ProcessUpSlopeTune g_process_upslope_tune = {
    .p1_x_m = 1.0f,
    .p1_y_m = 0.0f,
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
    .wait_raise_done_ms = 700U,/* 上升等待时间 */
    .wait_before_fall_ms = 1700U,/* 下降前等待时间 */
    .wait_fall_done_ms = 200U,/* 无用 */
    .vy_forward = 50.0f,/* 上台阶纵向速度 */
};

/**下台阶流程参数*/
volatile ProcessDownstairsTune g_process_downstairs_tune = {
    .fast_raise_back_ms = 1600U,/* 快速上升并后退经过时间 */
    .stop_before_fall_ms = 1000U,/* 无用*/
    .wait_fall_done_ms = 100U,/* 无用*/
    .vy_backward = -50.0f,
};

/**取kfs流程参数*/
volatile ProcessGetKfsTune g_process_get_kfs_tune = {
    .spin_front_to_p2_ms = 1200U,/* 前臂到p2经过时间 */
    .chassis_forward_ms = 3000U,/* 底盘前进经过时间 */
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


static void (*s_upslope_goto_fn)(float x_m, float y_m) = NULL;
static void (*s_upslope_yaw_fn)(ProcessFlowYawRef yaw_ref) = NULL;

void Process_UpSlope_Init(void (*goto_fn)(float x_m, float y_m),
                          void (*yaw_fn)(ProcessFlowYawRef yaw_ref))
{
    if ((goto_fn == 0) || (yaw_fn == 0))
    {
        return;
    }
    s_upslope_goto_fn = goto_fn;
    s_upslope_yaw_fn = yaw_fn;
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
    process_flow_debug.vx = process_flow_chassis_override.vx;
    process_flow_debug.vy = process_flow_chassis_override.vy;
    process_flow_debug.vw = process_flow_chassis_override.vw;
}
    
void Process_Flow_ClearChassisOverride(void)
{
    process_flow_chassis_override.axis_mask = 0U;
    process_flow_chassis_override.priority = PROCESS_FLOW_OVERRIDE_PRIORITY_LOW;
    process_flow_chassis_override.vx = 0.0f;
    process_flow_chassis_override.vy = 0.0f;
    process_flow_chassis_override.vw = 0.0f;
}

void Process_Flow_ResetAll(void)
{
    Process_Flow_ClearChassisOverride();
    upstairs_step = upstairs_step_idle;
    downstairs_step = downstairs_step_idle;
    get_kfs_step = get_kfs_step_idle;
    /* upslope: reset on estop/remote so next auto upslope starts from idle */
    s_upslope_step = upslope_step_idle;
    s_upslope_stage_ms = 0U;
    s_upslope_pitch_abs_base = 0.0f;
    s_upslope_pitch_abs_peak = 0.0f;
    s_upslope_fall_confirm = 0U;
    s_upslope_goto_latched = 0U;
}

void Process_UpStairs(void)
{
    static uint32_t now_ms = 0U;

    switch (upstairs_step)
    {
        case upstairs_step_idle:
            r2_lift_mode = raise;

            upstairs_step = upstairs_step_wait_raise_done;
            now_ms = osKernelGetTickCount();
            break;

        case upstairs_step_wait_raise_done:
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.wait_raise_done_ms)
            {
                process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
                process_flow_chassis_override.priority = PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH;
                process_flow_chassis_override.vy = g_process_upstairs_tune.vy_forward;
                now_ms = osKernelGetTickCount();
                upstairs_step = upstairs_step_wait_before_fall;
            }
            break;

        case upstairs_step_wait_before_fall:
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.wait_before_fall_ms)
            {
                Process_Flow_ClearChassisOverride();
                r2_lift_mode = fall;
                lift_fall_fast = 1U;
                upstairs_step = upstairs_step_wait_fall_done;
                now_ms = osKernelGetTickCount();
            }
            break;

        case upstairs_step_wait_fall_done:
            if ((osKernelGetTickCount() - now_ms) >= g_process_upstairs_tune.wait_fall_done_ms)
            {
                full_auto_mode = full_auto_none;
                upstairs_step = upstairs_step_idle;
            }
            break;

        default:
            upstairs_step = upstairs_step_idle;
            break;
    }
}

void Process_DownStairs(void)
{
    static uint32_t now_ms = 0U;

    switch (downstairs_step)
    {
        case downstairs_step_idle:
            r2_lift_mode = raise;
            lift_rise_fast = 1U;
            lift_fall_fast = 0U;
            process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
            process_flow_chassis_override.priority = PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH;
            process_flow_chassis_override.vy = g_process_downstairs_tune.vy_backward;
            now_ms = osKernelGetTickCount();
            downstairs_step = downstairs_step_fast_raise_back;
            break;

        case downstairs_step_fast_raise_back:
            if ((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.fast_raise_back_ms)
            {
                Process_Flow_ClearChassisOverride();
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_stop_before_fall;
            }
            break;

        case downstairs_step_stop_before_fall:
            if ((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.stop_before_fall_ms)
            {
                r2_lift_mode = fall;
                lift_fall_fast = 1U;
                downstairs_step = downstairs_step_wait_fall_done;
                now_ms = osKernelGetTickCount();
            }
            break;

        case downstairs_step_wait_fall_done:
            if((osKernelGetTickCount() - now_ms) >= g_process_downstairs_tune.wait_fall_done_ms)    
            {
                full_auto_mode = full_auto_none;
                downstairs_step = downstairs_step_idle;
            }
            break;

        default:
            downstairs_step = downstairs_step_idle;
            break;
    }
}

void Process_GetKFS(app_zone2_get_kfs_rel_t rel)
{
    static uint32_t now_ms = 0U;
    static uint8_t get_kfs_round = 0U; /* 0: first entry force p1; 1: normal */
    static Three_kfs_position start_three_pos = three_kfs_p1;

    switch (get_kfs_step)
    {
        case get_kfs_step_idle:
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
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.spin_front_to_p2_ms)
            {
                process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
                process_flow_chassis_override.priority = PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH;
                process_flow_chassis_override.vy = 10.0f;
                now_ms = osKernelGetTickCount();
                kfs_spin_position = kfs_spin_p2;
                get_kfs_step = get_kfs_step_chassis_forward;
            }
            break;

        case get_kfs_step_chassis_forward:
            if ((osKernelGetTickCount() - now_ms) >= g_process_get_kfs_tune.chassis_forward_ms)
            {
                Process_Flow_ClearChassisOverride();
                kfs_spin_position = kfs_spin_p1;
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
            full_auto_mode = full_auto_none;
            get_kfs_step = get_kfs_step_idle;
            get_kfs_round = 1U;
            break;

        default:
            Process_Flow_ClearChassisOverride();
            full_auto_mode = full_auto_none;
            get_kfs_step = get_kfs_step_idle;
            break;
    }

    process_flow_debug.get_kfs_round = (uint32_t)get_kfs_round;
}

void Process_PutKFS(void)
{
    Process_Flow_ClearChassisOverride();
    full_auto_mode = full_auto_none;
    Process_Flow_DebugSnapshot();
}

void Process_UpSlope(void)
{
    const uint32_t now_ms = osKernelGetTickCount();//获取当前时间戳
    float yaw_now = g_sensor_task_data.imu.yaw_deg;//获取当前航向
    float yaw_err_abs = 0.0f;//航向误差绝对值
    const float pitch_abs = fabsf(g_sensor_task_data.imu.pitch_deg); /* 上坡检测：俯仰角绝对值（度） */

    /* 如果当前步骤不是空闲状态且不是完成状态，且当前时间戳减去上次步骤开始时间大于阶段超时时间，则清除底盘覆盖并设置步骤为空闲状态，并设置自动模式为无自动模式 */
    if (s_upslope_step != upslope_step_idle &&
        s_upslope_step != upslope_step_done &&
        (now_ms - s_upslope_stage_ms) > g_process_upslope_tune.stage_timeout_ms)
    {
        Process_Flow_ClearChassisOverride();
        s_upslope_step = upslope_step_idle;
        full_auto_mode = full_auto_none;
        return;
    }

    switch (s_upslope_step)
    {
        case upslope_step_idle:
            s_upslope_pitch_abs_base = pitch_abs;
            s_upslope_pitch_abs_peak = pitch_abs;
            s_upslope_fall_confirm = 0U;/* 设置横滚角下降确认次数为0 */
            s_upslope_goto_latched = 0U;/* 进入到点阶段时只下发一次目标 */
            s_upslope_stage_ms = now_ms;
            s_upslope_step = upslope_step_goto_p1;/* 设置步骤为goto_p1 */
            break;

        case upslope_step_goto_p1:
            /* 到点阶段：主轴 p1 + 三轴 p4（每周期保持） */
            main_lift_position = main_lift_p1;
            three_kfs_position = three_kfs_p4;
            if (s_upslope_goto_latched == 0U)
            {
                if (s_upslope_goto_fn == NULL)
                {
                    Process_Flow_ClearChassisOverride();
                    s_upslope_step = upslope_step_idle;
                    full_auto_mode = full_auto_none;
                    break;
                }
                s_upslope_goto_fn(g_process_upslope_tune.p1_x_m, g_process_upslope_tune.p1_y_m);
                s_upslope_goto_latched = 1U;
            }
            if (odom_nav_goto_run(&odom_nav_target, NULL) == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                s_upslope_stage_ms = now_ms;
                s_upslope_step = upslope_step_wait_after_goto;
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
            /* 如果yaw目标函数为空，则清除底盘覆盖并设置步骤为空闲状态，并设置自动模式为无自动模式 */
            if (s_upslope_yaw_fn == NULL)
            {
                Process_Flow_ClearChassisOverride();
                s_upslope_step = upslope_step_idle;
                full_auto_mode = full_auto_none;
                break;
            }
            s_upslope_yaw_fn(APP_ZONE2_FIELD_FRONT);
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
            process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
            process_flow_chassis_override.priority = PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH;
            process_flow_chassis_override.vy = g_process_upslope_tune.vy_target;
            if (pitch_abs > s_upslope_pitch_abs_peak) s_upslope_pitch_abs_peak = pitch_abs;
            if ((pitch_abs - s_upslope_pitch_abs_base) >= g_process_upslope_tune.pitch_abs_rise_th_deg)
            {
                s_upslope_stage_ms = now_ms;
                s_upslope_step = upslope_step_wait_roll_fall;
            }
            break;

        case upslope_step_wait_roll_fall:
            process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
            process_flow_chassis_override.priority = PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH;
            process_flow_chassis_override.vy = g_process_upslope_tune.vy_target;
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
            full_auto_mode = full_auto_none;/* 设置自动模式为无自动模式 */
            s_upslope_step = upslope_step_idle;/* 设置步骤为idle */
            break;

        default:
            Process_Flow_ClearChassisOverride();/* 清除底盘覆盖 */
            full_auto_mode = full_auto_none;
            s_upslope_step = upslope_step_idle;/* 设置步骤为idle */
            break;
    }
}
