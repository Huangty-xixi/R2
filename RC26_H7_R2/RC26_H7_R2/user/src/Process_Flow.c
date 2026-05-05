#include "Process_Flow.h"
#include "Motion_Task.h"
#include "lift.h"
#include "kfs.h"
#include "weapon.h"
#include "cmsis_os.h"

ProcessFlowChassisOverride process_flow_chassis_override = {0U, 0.0f, 0.0f, 0.0f};
UpstairsStep upstairs_step = upstairs_step_idle;
DownstairsStep downstairs_step = downstairs_step_idle;
GetKfsStep get_kfs_step = get_kfs_step_idle;
volatile ProcessFlowDebug process_flow_debug = {1U};

void Process_Flow_DebugSnapshot(void)
{
    if (process_flow_debug.enable == 0U) return;

    process_flow_debug.seq++;
    process_flow_debug.now_tick = osKernelGetTickCount();

    process_flow_debug.upstairs_step = (uint32_t)upstairs_step;
    process_flow_debug.downstairs_step = (uint32_t)downstairs_step;
    process_flow_debug.get_kfs_step = (uint32_t)get_kfs_step;

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
            if ((osKernelGetTickCount() - now_ms) >= 500U)
            {
                process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
                process_flow_chassis_override.vy = 50.0f;
                now_ms = osKernelGetTickCount();
                upstairs_step = upstairs_step_wait_before_fall;
            }
            break;

        case upstairs_step_wait_before_fall:
            if ((osKernelGetTickCount() - now_ms) >= 2000U)
            {
                Process_Flow_ClearChassisOverride();
                r2_lift_mode = fall;
                lift_fall_fast = 1U;
                upstairs_step = upstairs_step_wait_fall_done;
                now_ms = osKernelGetTickCount();
            }
            break;

        case upstairs_step_wait_fall_done:
            if ((osKernelGetTickCount() - now_ms) >= 100U)
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
    static uint32_t now_ms = 0U;

    switch (downstairs_step)
    {
        case downstairs_step_idle:
            r2_lift_mode = raise;
            lift_rise_fast = 1U;
            lift_fall_fast = 0U;
            process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
            process_flow_chassis_override.vy = -50.0f;
            now_ms = osKernelGetTickCount();
            downstairs_step = downstairs_step_fast_raise_back;
            break;

        case downstairs_step_fast_raise_back:
            if ((osKernelGetTickCount() - now_ms) >= 1500U)
            {
                Process_Flow_ClearChassisOverride();
                now_ms = osKernelGetTickCount();
                downstairs_step = downstairs_step_stop_before_fall;
            }
            break;

        case downstairs_step_stop_before_fall:
            if ((osKernelGetTickCount() - now_ms) >= 500U)
            {
                r2_lift_mode = fall;
                lift_fall_fast = 1U;
                downstairs_step = downstairs_step_wait_fall_done;
                now_ms = osKernelGetTickCount();
            }
            break;

        case downstairs_step_wait_fall_done:
            if((osKernelGetTickCount() - now_ms) >= 100U)    
            {
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
            main_lift_position = main_lift_p1;
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
            else
            {
                sucker1_state = 1U;
                sucker4_state = 1U;
            }

            kfs_spin_position = kfs_spin_p2;
            now_ms = osKernelGetTickCount();
            get_kfs_step = get_kfs_step_spin_front_to_p2;
            break;

        case get_kfs_step_spin_front_to_p2:
            if ((osKernelGetTickCount() - now_ms) >= 1200U)
            {
                process_flow_chassis_override.axis_mask = PROCESS_FLOW_CHASSIS_OVERRIDE_VY;
                process_flow_chassis_override.vy = 10.0f;
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_chassis_forward;
            }
            break;

        case get_kfs_step_chassis_forward:
            if ((osKernelGetTickCount() - now_ms) >= 5000U)
            {
                Process_Flow_ClearChassisOverride();
                kfs_spin_position = kfs_spin_p1;
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_spin_front_to_p1;
            }
            break;

        case get_kfs_step_spin_front_to_p1:
            if ((osKernelGetTickCount() - now_ms) >= 1200U)
            {
                sucker1_state = 0U;
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_wait_after_close_s1;
            }
            break;

        case get_kfs_step_wait_after_close_s1:
            if ((osKernelGetTickCount() - now_ms) >= 1200U)
            {
                kfs_spin_position = kfs_spin_p2;
                now_ms = osKernelGetTickCount();
                get_kfs_step = get_kfs_step_wait_front_p2_done;
            }
            break;

        case get_kfs_step_wait_front_p2_done:
            if ((osKernelGetTickCount() - now_ms) >= 1200U)
            {
                if (start_three_pos == three_kfs_p1)
                {
                    three_kfs_position = three_kfs_p2;
                }
                else if (start_three_pos == three_kfs_p2)
                {
                    three_kfs_position = three_kfs_p3;
                }
                get_kfs_step = get_kfs_step_done;
            }
            break;

        case get_kfs_step_done:
            Process_Flow_ClearChassisOverride();
            semi_auto_mode = semi_auto_none;
            get_kfs_step = get_kfs_step_idle;
            get_kfs_round = 1U;
            break;

        default:
            Process_Flow_ClearChassisOverride();
            semi_auto_mode = semi_auto_none;
            get_kfs_step = get_kfs_step_idle;
            break;
    }

    process_flow_debug.get_kfs_round = (uint32_t)get_kfs_round;
}

void Process_PutKFS(void)
{
    Process_Flow_ClearChassisOverride();
    semi_auto_mode = semi_auto_none;
    Process_Flow_DebugSnapshot();
}
