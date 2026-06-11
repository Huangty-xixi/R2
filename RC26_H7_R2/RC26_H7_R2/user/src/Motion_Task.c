#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"
#include "Process_Flow.h"
#include "app_zone1.h"
#include "app_zone2.h"
#include "app_zone3.h"
#include "clamp_head_ctrl.h"
#include "yaw_heading_ctrl.h"

Control_mode control_mode;
Remote_mode remote_mode;
Flow_mode flow_mode = flow_none;
App_flow_mode app_flow_mode = app_flow_none;

static Control_mode s_motion_prev_control_mode = remote_control;
static uint8_t s_zone1_prev_run_cond = 0U;

static uint8_t rc_bit_minmax_decode(uint16_t ch_val)
{
    if (ch_val <= 500u) return 0u;
    if (ch_val >= 1500u) return 1u;
    return 2u;
}

void Motion_Task(void const * argument)
{
    for (;;)
    {
        uint8_t ch6_bit = rc_bit_minmax_decode(RCctrl.CH6);
        uint8_t ch7_bit = rc_bit_minmax_decode(RCctrl.CH7);
        uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);

        if (RCctrl.CH8 < 500)
        {
            control_mode = emergency_stop_mode;
        }
        else if (RCctrl.CH8 > 500 && RCctrl.CH8 < 1500)
        {
            control_mode = full_auto_control;
        }
        else
        {
            control_mode = remote_control;
        }

        if (((control_mode == remote_control) || (control_mode == emergency_stop_mode)) &&
            (s_motion_prev_control_mode == full_auto_control))
        {
            ClampHeadCtrl_Init();
        }
        s_motion_prev_control_mode = control_mode;

        {
            uint8_t zone1_run_cond = (uint8_t)((control_mode == full_auto_control) && (ch7_bit == 1u));

            if ((zone1_run_cond == 0U) && (s_zone1_prev_run_cond != 0U))
            {
                app_zone1_mission_clear();
                if (app_flow_mode == app_flow_zone1)
                {
                    app_flow_mode = app_flow_none;
                }
            }
            else if ((zone1_run_cond != 0U) && (s_zone1_prev_run_cond == 0U))
            {
#if MOTION_YAW_TUNE_CH5
                uint8_t z1_r_get_kfs = 0u;
                uint8_t z1_r_put_kfs = 0u;
#else
                uint8_t z1_ch5_bit = rc_bit_minmax_decode(RCctrl.CH5);
                uint8_t z1_r_get_kfs = (uint8_t)(z1_ch5_bit == 0u);
                uint8_t z1_r_put_kfs = (uint8_t)(z1_ch5_bit == 1u);
#endif
                uint8_t z1_r_z2 = (uint8_t)(ch6_bit == 1u);
                uint8_t z1_cmd_count = (uint8_t)(z1_r_z2 + z1_r_get_kfs + z1_r_put_kfs + 1u);

                if ((z1_cmd_count == 1u) && (app_flow_mode == app_flow_none))
                {
                    AppZone1_Start();
                    app_flow_mode = app_flow_zone1;
                    flow_mode = flow_none;
                }
            }
            s_zone1_prev_run_cond = zone1_run_cond;
        }

        switch (control_mode)
        {
        case remote_control:
            Process_Flow_ResetAll();
            flow_mode = flow_none;
            app_flow_mode = app_flow_none;
            app_zone2_mission_clear();
            app_zone1_mission_clear();
            AppZone3_Reset();
            if ((ch6_bit <= 1u) && (ch7_bit <= 1u))
            {
                switch (mode_code)
                {
                case 0u:
                    remote_mode = chassis_mode;
                    break;
                case 1u:
                    remote_mode = weapon_mode;
                    break;
                case 2u:
                    remote_mode = lift_mode;
                    break;
                case 3u:
                    remote_mode = kfs_mode;
                    break;
                default:
                    break;
                }
            }
            break;

        case emergency_stop_mode:
            Process_Flow_ResetAll();
            flow_mode = flow_none;
            app_flow_mode = app_flow_none;
            app_zone2_mission_clear();
            app_zone1_mission_clear();
            AppZone3_Reset();
            break;

        case full_auto_control:
        {
#if MOTION_YAW_TUNE_CH5
            uint8_t r_get_kfs = 0u;
            uint8_t r_put_kfs = 0u;
#else
            uint8_t ch5_bit = rc_bit_minmax_decode(RCctrl.CH5);
            /* CH5：低=取KFS，高=放KFS；CH7=一区 */
            uint8_t r_get_kfs = (uint8_t)(ch5_bit == 0u);
            uint8_t r_put_kfs = (uint8_t)(ch5_bit == 1u);
#endif
            uint8_t r_zone1 = (uint8_t)(ch7_bit == 1u);
            uint8_t r_z2 = (uint8_t)(ch6_bit == 1u);
            uint8_t cmd_count;

            remote_mode = chassis_mode;

#if MOTION_YAW_TUNE_CH5
            /* CH5: 转固定角度，>1500右转90°，<500左转90°（边沿触发） */
            {
                static uint16_t ch5_prev = 1024U;
                uint16_t ch5_now = RCctrl.CH5;

                if (ch5_now >= 1500U && ch5_prev < 1500U)
                {
                    YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_right_90);
                }
                else if (ch5_now <= 500U && ch5_prev > 500U)
                {
                    YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_left_90);
                }
                ch5_prev = ch5_now;
            }
#endif

            if (app_flow_mode == app_flow_zone2)
            {
                app_zone2_poll();
                if (app_zone2_is_done() != 0U)
                    app_flow_mode = app_flow_none;
            }
            else if (app_flow_mode == app_flow_zone1)
            {
                app_zone1_poll();
                if ((app_zone1_is_done() != 0U) || (app_zone1_is_failed() != 0U))
                {
                    app_flow_mode = app_flow_none;
                }
            }
            else if ((app_flow_mode == app_flow_zone3) || (AppZone3_IsActive() != 0U))
            {
                if (app_flow_mode != app_flow_zone3)
                    app_flow_mode = app_flow_zone3;
                AppZone3_Run();
                if ((AppZone3_IsActive() == 0U) &&
                    ((AppZone3_IsDone() != 0U) || (AppZone3_IsFailed() != 0U)))
                    app_flow_mode = app_flow_none;
            }
            else if (flow_mode == flow_none)
            {
                cmd_count = (uint8_t)(r_z2 + r_get_kfs + r_put_kfs + r_zone1);
                if (cmd_count == 1u)
                {
                    if (r_get_kfs != 0u)
                        flow_mode = flow_get_kfs_mode;
                    else if (r_put_kfs != 0u)
                        flow_mode = flow_put_kfs_mode;
                    else if (r_zone1 == 0u)
                        app_flow_mode = app_flow_zone2;
                }
            }
            break;
        }
        }

        osDelay(1);
    }
}