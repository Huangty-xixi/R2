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
#include "app_zone3_prep.h"
#include "app_init.h"

Control_mode control_mode;
Remote_mode remote_mode;
Flow_mode flow_mode = flow_none;
App_flow_mode app_flow_mode = app_flow_none;

/* 通道触发消抖：500ms窗口 */
static uint32_t s_trigger_settle_ms   = 0;
static uint8_t  s_trigger_settle_cmd  = 0;
static uint8_t  s_match_auto_active   = 0;  /* 比赛自动序列进行中 */

#if APP_MATCH_IS_ARENA || APP_MATCH_SKILL_Z12 || APP_MATCH_SKILL_Z3
static void app_flow_start_match(void)
{
    s_match_auto_active = 1U;

#if APP_MATCH_SKILL_Z12
    /* Z12技能赛：一区(双圈) → 二区(截断) */
    AppZone1_Start(); app_flow_mode = app_flow_zone1;
#elif APP_MATCH_SKILL_Z3
    /* Z3技能赛：预备阶段 → 三区(不走一区二区) */
    AppZone3Prep_Start();
    app_flow_mode = app_flow_zone3_prep;
#else /* APP_MATCH_IS_ARENA */
    /* 竞技赛：一区(单圈) → 二区(全程) → 三区(等R1) */
    AppZone1_Start(); app_flow_mode = app_flow_zone1;
#endif
}
#endif



static Control_mode s_motion_prev_control_mode = remote_control;
static uint8_t s_ch7_prev_high = 0U;

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
            uint8_t ch7_high = (uint8_t)((control_mode == full_auto_control) && (ch7_bit == 1u));

            if ((ch7_high == 0U) && (s_ch7_prev_high != 0U))
            {
                app_zone1_mission_clear();
                if (flow_mode == flow_camera_debug)
                {
                    flow_mode = flow_none;
                }
                if (app_flow_mode == app_flow_zone1)
                {
                    app_flow_mode = app_flow_none;
                }
            }
            else if ((ch7_high != 0U) && (s_ch7_prev_high == 0U))
            {
                if (flow_mode == flow_none && app_flow_mode == app_flow_none)
                {
                    app_flow_mode = app_flow_zone1;
                }
            }
            s_ch7_prev_high = ch7_high;
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
            AppZone3Prep_Reset();
            s_trigger_settle_ms = 0;
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
            AppZone3Prep_Reset();
            s_trigger_settle_ms = 0;
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
                {
#if APP_MATCH_IS_ARENA
                    /* 竞技赛：二区完 → 三区(等R1命令) */
                    AppZone3_Start();
                    app_flow_mode = app_flow_zone3;
#else
                    /* Z12技能赛：二区完 → 结束 */
                    s_match_auto_active = 0U;
                    app_flow_mode = app_flow_none;
#endif
                }
            }
            else if (app_flow_mode == app_flow_zone1)
            {
                app_zone1_poll();
                if ((AppZone1_IsBusy() == 0U) && (AppZone1_IsDone() == 0U) && (AppZone1_IsFailed() == 0U))
                {
                    AppZone1_Start();
                }
                AppZone1_Run();
                if ((AppZone1_IsBusy() == 0U) &&
                    ((AppZone1_IsDone() != 0U) || (AppZone1_IsFailed() != 0U)))
                {
                    if (s_match_auto_active != 0U)
                    {
                        /* 比赛自动序列：一区完 → 二区 */
                        app_flow_mode = app_flow_zone2;
                    }
                    else
                    {
                        app_flow_mode = app_flow_none;
                    }
                }
            }
            else if ((app_flow_mode == app_flow_zone3_prep) || (AppZone3Prep_IsActive() != 0U))
            {
                if (app_flow_mode != app_flow_zone3_prep)
                    app_flow_mode = app_flow_zone3_prep;
                AppZone3Prep_Run();
                if ((AppZone3Prep_IsActive() == 0U) &&
                    ((AppZone3Prep_IsDone() != 0U) || (AppZone3Prep_IsFailed() != 0U)))
                    app_flow_mode = app_flow_none;
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

                if (cmd_count == 0u)
                {
                    s_trigger_settle_ms = 0;
                }
                else if (cmd_count == 1u)
                {
                    /* 单通道：立刻执行 */
                    s_trigger_settle_ms = 0;
                    if (r_get_kfs != 0u)
                        flow_mode = flow_get_kfs_mode;
                    else if (r_put_kfs != 0u)
                        flow_mode = flow_put_kfs_mode;
                    else if (r_zone1 != 0u)
                        app_flow_mode = app_flow_zone1;
                    else
                        app_flow_mode = app_flow_zone2;
                }
#if APP_MATCH_IS_ARENA || APP_MATCH_SKILL_Z12 || APP_MATCH_SKILL_Z3
                else
                {
                    app_flow_start_match();
                }
#endif
            }
            break;
        }
        }

        osDelay(1);
    }
}