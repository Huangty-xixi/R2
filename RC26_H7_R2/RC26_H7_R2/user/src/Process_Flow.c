#include "Process_Flow.h"
#include "Motion_Task.h"
#include "lift.h"
#include "cmsis_os.h"

ProcessFlowChassisOverride process_flow_chassis_override = {0U, 0.0f, 0.0f, 0.0f};

#define PROCESS_UPSTAIRS_FORWARD_VY   (100.0f)   /* 约等效 CH2 最大值 */
#define PROCESS_UPSTAIRS_RAISE_MS     (1200U)    /* 上抬+前进阶段 */
#define PROCESS_UPSTAIRS_FASTFALL_MS  (700U)     /* 快速下降阶段 */

static void Process_Flow_ClearChassisOverride(void)
{
    process_flow_chassis_override.axis_mask = 0U;
    process_flow_chassis_override.vx = 0.0f;
    process_flow_chassis_override.vy = 0.0f;
    process_flow_chassis_override.vw = 0.0f;
}

void Process_UpStairs(void)
{
    typedef enum
    {
        upstairs_step_idle = 0,
        upstairs_step_raise_forward,
        upstairs_step_fast_fall
    } UpstairsStep;

    static UpstairsStep step = upstairs_step_idle;
    static uint32_t step_start_tick = 0U;

    switch (step)
    {
        case upstairs_step_idle:
            r2_lift_mode = raise;
            lift_fall_fast = 0U;
            lift_rise_fast = 0U;
            process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
            process_flow_chassis_override.vy = PROCESS_UPSTAIRS_FORWARD_VY;
            step_start_tick = osKernelGetTickCount();
            step = upstairs_step_raise_forward;
            break;

        case upstairs_step_raise_forward:
            if ((osKernelGetTickCount() - step_start_tick) >= PROCESS_UPSTAIRS_RAISE_MS)
            {
                r2_lift_mode = fall;
                lift_fall_fast = 1U;
                Process_Flow_ClearChassisOverride();
                step_start_tick = osKernelGetTickCount();
                step = upstairs_step_fast_fall;
            }
            break;

        case upstairs_step_fast_fall:
            if ((osKernelGetTickCount() - step_start_tick) >= PROCESS_UPSTAIRS_FASTFALL_MS)
            {
                lift_fall_fast = 0U;
                semi_auto_mode = semi_auto_none;
                step = upstairs_step_idle;
            }
            break;

        default:
            step = upstairs_step_idle;
            break;
    }
}

void Process_DownStairs(void)
{
    Process_Flow_ClearChassisOverride();
    semi_auto_mode = semi_auto_none;
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
