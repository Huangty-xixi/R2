/**
 * @file Motion_Task.c
 * @brief 遥控器通道解码任务：CH8=控制模式 / CH6+CH7=遥控子模式
 *
 * 纯遥控车——只保留 remote_control 和 emergency_stop_mode。
 * full_auto_control / flow_mode / app_flow_mode 全部删除。
 */

#include "Motion_Task.h"
#include "remote_control.h"
#include "cmsis_os.h"

Control_mode control_mode;
Remote_mode remote_mode;

static uint8_t rc_bit_minmax_decode(uint16_t ch_val)
{
    if (ch_val <= 500u) return 0u;
    if (ch_val >= 1500u) return 1u;
    return 2u;
}

/* CH6 + CH7 -> chassis / weapon / lift / kfs */
static void motion_remote_decode(uint8_t ch6_bit, uint8_t ch7_bit)
{
    uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);

    if ((ch6_bit <= 1u) && (ch7_bit <= 1u))
    {
        switch (mode_code)
        {
        case 0u: remote_mode = chassis_mode; break;
        case 1u: remote_mode = weapon_mode; break;
        case 2u: remote_mode = lift_mode;   break;
        case 3u: remote_mode = kfs_mode;    break;
        default: break;
        }
    }
}

void Motion_Task(void const * argument)
{
    for (;;)
    {
        uint8_t ch6_bit = rc_bit_minmax_decode(RCctrl.CH6);
        uint8_t ch7_bit = rc_bit_minmax_decode(RCctrl.CH7);

        /* CH8 -> 控制模式 */
        if (RCctrl.CH8 < 500)
        {
            control_mode = emergency_stop_mode;
        }
        else if (RCctrl.CH8 > 500 && RCctrl.CH8 < 1500)
        {
            control_mode = remote_control;
        }
        else
        {
            control_mode = remote_control;
        }

        /* 遥控模式：CH6+CH7 切子模式 */
        if (control_mode == remote_control)
        {
            motion_remote_decode(ch6_bit, ch7_bit);
        }

        osDelay(1);
    }
}
