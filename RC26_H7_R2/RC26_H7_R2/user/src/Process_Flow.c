#include "Process_Flow.h"
#include "Motion_Task.h"
#include "lift.h"
#include "cmsis_os.h"

ProcessFlowChassisOverride process_flow_chassis_override = {0U, 0.0f, 0.0f, 0.0f};
UpstairsStep upstairs_step = upstairs_step_idle;
DownstairsStep downstairs_step = downstairs_step_idle;

#define PROCESS_UPSTAIRS_FORWARD_VY   (50.0f)   /* 约等效 CH2 最大值 */
#define PROCESS_UPSTAIRS_FORWARD_MS   (2000U)    /* 抬升到位后前进阶段 */
#define PROCESS_DOWNSTAIRS_BACKWARD_VY  (-50.0f) /* 约等效 CH2 最小值（后退） */
#define PROCESS_DOWNSTAIRS_BACK_MS      (1000U)   /* 快速上抬并后退阶段 */
#define PROCESS_DOWNSTAIRS_STOP_MS      (80U)     /* 停后退到切下降的缓冲 */

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
            r2_lift_mode = raise;
            Process_Flow_ClearChassisOverride();
            upstairs_step = upstairs_step_wait_raise_done;
            break;

        case upstairs_step_wait_raise_done:
            if (lift_has_stopped != 0U)
            {
                process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
                process_flow_chassis_override.vy = PROCESS_UPSTAIRS_FORWARD_VY;
                step_start_tick = osKernelGetTickCount();
                upstairs_step = upstairs_step_forward_on_raised;
            }
            break;

        case upstairs_step_forward_on_raised:
            if ((osKernelGetTickCount() - step_start_tick) >= PROCESS_UPSTAIRS_FORWARD_MS)
            {
                Process_Flow_ClearChassisOverride();
                r2_lift_mode = fall;
                lift_fall_fast = 1U;
                upstairs_step = upstairs_step_wait_fall_done;
            }
            break;

        case upstairs_step_wait_fall_done:
            if ((lift_has_stopped != 0U) && (lift_stop_mode == fall))
            {
                lift_fall_fast = 0U;
                Process_Flow_ClearChassisOverride();
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
    static DownstairsStep downstairs_step = downstairs_step_idle;
    static uint32_t step_start_tick = 0U;

    switch (downstairs_step)
    {
        case downstairs_step_idle:
            r2_lift_mode = raise;
            lift_rise_fast = 1U;
            lift_fall_fast = 0U;
            process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
            process_flow_chassis_override.vy = PROCESS_DOWNSTAIRS_BACKWARD_VY;
            step_start_tick = osKernelGetTickCount();
            downstairs_step = downstairs_step_fast_raise_back;
            break;

        case downstairs_step_fast_raise_back:
            if (((osKernelGetTickCount() - step_start_tick) >= PROCESS_DOWNSTAIRS_BACK_MS) &&
                (lift_has_stopped != 0U) &&
                (lift_stop_mode == raise))
            {
                Process_Flow_ClearChassisOverride();
                step_start_tick = osKernelGetTickCount();
                downstairs_step = downstairs_step_stop_before_fall;
            }
            break;

        case downstairs_step_stop_before_fall:
            if ((osKernelGetTickCount() - step_start_tick) >= PROCESS_DOWNSTAIRS_STOP_MS)
            {
                r2_lift_mode = fall;
                downstairs_step = downstairs_step_wait_fall_done;
            }
            break;

        case downstairs_step_wait_fall_done:
            if ((lift_has_stopped != 0U) && (lift_stop_mode == fall))
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
}

void Process_PutKFS(void)
{
    Process_Flow_ClearChassisOverride();
    semi_auto_mode = semi_auto_none;
}
