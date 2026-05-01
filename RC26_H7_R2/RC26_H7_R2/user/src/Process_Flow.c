#include "Process_Flow.h"
#include "Motion_Task.h"
#include "lift.h"
#include "cmsis_os.h"

ProcessFlowChassisOverride process_flow_chassis_override = {0U, 0.0f, 0.0f, 0.0f};
UpstairsStep upstairs_step = upstairs_step_idle;
DownstairsStep downstairs_step = downstairs_step_idle;
volatile ProcessFlowDebug process_flow_debug = {1U};

void Process_Flow_DebugSnapshot(void)
{
    if (process_flow_debug.enable == 0U) return;

    process_flow_debug.seq++;
    process_flow_debug.now_tick = osKernelGetTickCount();

    process_flow_debug.upstairs_step = (uint32_t)upstairs_step;
    process_flow_debug.downstairs_step = (uint32_t)downstairs_step;

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

#define PROCESS_UPSTAIRS_FORWARD_VY   (50.0f)   /* 约等效 CH2 最大值 */
#define PROCESS_UPSTAIRS_FORWARD_MS   (2000U)    /* 抬升到位后前进阶段 */
#define PROCESS_DOWNSTAIRS_BACKWARD_VY  (-50.0f) /* 约等效 CH2 最小值（后退） */
#define PROCESS_DOWNSTAIRS_BACK_MS      (1500U)   /* 快速上抬并后退阶段 */
#define PROCESS_FLOW_STEP_GAP_MS        (500U)    /* 各步骤切换统一缓冲 */

static void Process_Flow_ClearChassisOverride(void)
{
    process_flow_chassis_override.axis_mask = 0U;
    process_flow_chassis_override.vx = 0.0f;
    process_flow_chassis_override.vy = 0.0f;
    process_flow_chassis_override.vw = 0.0f;
}

void Process_Flow_ResetAll(void)
{
    Process_Flow_ClearChassisOverride();
    upstairs_step = upstairs_step_idle;
    downstairs_step = downstairs_step_idle;
}

void Process_UpStairs(void)
{
    static uint32_t step_start_tick = 0U;

    switch (upstairs_step)
    {
        case upstairs_step_idle:
            lift_clear_stop_latch();
            r2_lift_mode = raise;
            Process_Flow_ClearChassisOverride();
            upstairs_step = upstairs_step_wait_raise_done;
            step_start_tick = osKernelGetTickCount();
            break;

        case upstairs_step_wait_raise_done:
            if ((lift_has_stopped != 0U) &&
                ((osKernelGetTickCount() - step_start_tick) >= PROCESS_FLOW_STEP_GAP_MS))
            {
                process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
                process_flow_chassis_override.vy = PROCESS_UPSTAIRS_FORWARD_VY;
                step_start_tick = osKernelGetTickCount();
                upstairs_step = upstairs_step_wait_before_fall;
            }
            break;

        case upstairs_step_wait_before_fall:
            if ((osKernelGetTickCount() - step_start_tick) >= (PROCESS_UPSTAIRS_FORWARD_MS))
            {
                lift_clear_stop_latch();
                Process_Flow_ClearChassisOverride();
                r2_lift_mode = fall;
                lift_fall_fast = 1U;
                upstairs_step = upstairs_step_wait_fall_done;
                step_start_tick = osKernelGetTickCount();
            }
            break;

        case upstairs_step_wait_fall_done:
            if ((lift_has_stopped != 0U) &&
                ((osKernelGetTickCount() - step_start_tick) >= PROCESS_FLOW_STEP_GAP_MS))
            {
                semi_auto_mode = semi_auto_none;
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
    static uint32_t step_start_tick = 0U;

    switch (downstairs_step)
    {
        case downstairs_step_idle:
            lift_clear_stop_latch();
            r2_lift_mode = raise;
            lift_rise_fast = 1U;
            lift_fall_fast = 0U;
            process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
            process_flow_chassis_override.vy = PROCESS_DOWNSTAIRS_BACKWARD_VY;
            step_start_tick = osKernelGetTickCount();
            downstairs_step = downstairs_step_fast_raise_back;
            break;

        case downstairs_step_fast_raise_back:
            if (((osKernelGetTickCount() - step_start_tick) >= (PROCESS_DOWNSTAIRS_BACK_MS)) &&
                (lift_has_stopped != 0U))
            {
                Process_Flow_ClearChassisOverride();
                step_start_tick = osKernelGetTickCount();
                downstairs_step = downstairs_step_stop_before_fall;
            }
            break;

        case downstairs_step_stop_before_fall:
            if ((osKernelGetTickCount() - step_start_tick) >= PROCESS_FLOW_STEP_GAP_MS)
            {
                lift_clear_stop_latch();
                Process_Flow_ClearChassisOverride();
                r2_lift_mode = fall;
                lift_fall_fast = 1U;
                downstairs_step = downstairs_step_wait_fall_done;
                step_start_tick = osKernelGetTickCount();
            }
            break;

        case downstairs_step_wait_fall_done:
            if ((lift_has_stopped != 0U) &&
                ((osKernelGetTickCount() - step_start_tick) >= PROCESS_FLOW_STEP_GAP_MS))
            {
                Process_Flow_ClearChassisOverride();
                semi_auto_mode = semi_auto_none;
                downstairs_step = downstairs_step_idle;
            }
            break;

        default:
            downstairs_step = downstairs_step_idle;
            break;
    }
}

void Process_GetKFS(void)
{
    Process_Flow_ClearChassisOverride();
    semi_auto_mode = semi_auto_none;
    Process_Flow_DebugSnapshot();
}

void Process_PutKFS(void)
{
    Process_Flow_ClearChassisOverride();
    semi_auto_mode = semi_auto_none;
    Process_Flow_DebugSnapshot();
}
