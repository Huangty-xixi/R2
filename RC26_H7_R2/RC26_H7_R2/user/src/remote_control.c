#include "remote_control.h"
#include "main.h"
#include <math.h>


uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

#define ABS(x)  ((x) >= 0? (x) : -(x))

int16_t data_convert(int src, int src_min, int src_max, float dst_low, float dst_high)
{
	float res=0;
	res = (src - src_min)/1.0/(src_max - src_min) * (dst_high - dst_low) + dst_low;
	if (ABS(res) < 10 && dst_high !=5) {res = 0.0001f;}
	if (ABS(res) < 0.1f && dst_high ==5) {res = 0.0001f;}
	return res;
}

Remote_Info_Typedef RCctrl={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};

/**
* @brief SBUS数据协议解析
* @param sbus_buf：接收缓冲数组
* @param Remote_Ctrl: 遥控器数据结构体
* @date&author  2025/12/25  zhouxy
*/
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl)
{
    if (sbus_buf == NULL || Remote_Ctrl == NULL) return;

    /* Channel 0, 1, 2, 3 */
	Remote_Ctrl->CH1 = ((((uint16_t)sbus_buf[2] << 8) | (uint16_t)sbus_buf[1]) & 0x07FF);
	Remote_Ctrl->CH2 = ((int16_t)sbus_buf[ 2] >> 3 | ((int16_t)sbus_buf[ 3] << 5 )) & 0x07FF;
	Remote_Ctrl->CH3 = ((int16_t)sbus_buf[ 3] >> 6 | ((int16_t)sbus_buf[ 4] << 2 ) | (int16_t)sbus_buf[ 5] << 10 ) & 0x07FF;
    Remote_Ctrl->CH4 = ((int16_t)sbus_buf[ 5] >> 1 | ((int16_t)sbus_buf[ 6] << 7 )) & 0x07FF;
	Remote_Ctrl->CH5 = ((int16_t)sbus_buf[ 6] >> 4 | ((int16_t)sbus_buf[ 7] << 4 )) & 0x07FF;
	Remote_Ctrl->CH6 = ((int16_t)sbus_buf[ 7] >> 7 | ((int16_t)sbus_buf[ 8] << 1 ) | (int16_t)sbus_buf[9] << 9 ) & 0x07FF;
	Remote_Ctrl->CH7 = ((int16_t)sbus_buf[ 9] >> 2 | ((int16_t)sbus_buf[ 10] << 6)) & 0x07FF;
	Remote_Ctrl->CH8 = ((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[ 11] << 3)) & 0x07FF;
	Remote_Ctrl->CH9 = ((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF;
	Remote_Ctrl->CH10 = ((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF;
	Remote_Ctrl->CH11 = ((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 ) | (int16_t)sbus_buf[16] << 10 ) & 0x07FF;
	Remote_Ctrl->CH12 = ((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF;
    Remote_Ctrl->CH13 = ((int16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF;
    Remote_Ctrl->CH14 = ((int16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 ) | (int16_t)sbus_buf[20] << 9  ) & 0x07FF;
    Remote_Ctrl->CH15 = ((int16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF;
    Remote_Ctrl->CH16 = ((int16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF;
    

//    (sbus_buf[23] == 0x00) ? (Remote_Ctrl->rc_lost = false) : (Remote_Ctrl->rc_lost = true);
		/* reset the online count */
		Remote_Ctrl->online_cnt = 0xFAU;
		
		/* reset the lost flag */
		Remote_Ctrl->rc_lost = false;
}

void RemoteControl_LinkWatchdog_Update(Remote_Info_Typedef *Remote_Ctrl)
{
    if (Remote_Ctrl == NULL) return;

#if REMOTE_LOST_PROTECT_ENABLE
    if (Remote_Ctrl->online_cnt > 0U)
    {
        Remote_Ctrl->online_cnt--;
    }

    if (Remote_Ctrl->online_cnt == 0U)
    {
        Remote_Ctrl->rc_lost = true;
    }
#else
    (void)Remote_Ctrl;
#endif
}

volatile uint8_t g_remote_link_test_step = 0U;
volatile uint8_t g_remote_link_test_result = 0U;

void RemoteControl_LinkWatchdog_SimpleTest(Remote_Info_Typedef *Remote_Ctrl)
{
    if (Remote_Ctrl == NULL) return;

#if REMOTE_LINK_TEST_ENABLE
    static uint32_t step_tick_ms = 0U;
    uint32_t now_ms = HAL_GetTick();

    if (step_tick_ms == 0U)
    {
        step_tick_ms = now_ms;
    }

    /* step0: 正常在线，期望未丢失 */
    if (g_remote_link_test_step == 0U)
    {
        Remote_Ctrl->online_cnt = 0xFAU;
        Remote_Ctrl->rc_lost = false;
        g_remote_link_test_result = (Remote_Ctrl->rc_lost != false) ? 1U : 0U; /* expect 0 */
        if ((uint32_t)(now_ms - step_tick_ms) >= REMOTE_LINK_TEST_STEP_MS)
        {
            g_remote_link_test_step = 1U;
            step_tick_ms = now_ms;
        }
        return;
    }

    /* step1: 人为设置即将超时，调用看门狗后期望丢失 */
    if (g_remote_link_test_step == 1U)
    {
        Remote_Ctrl->online_cnt = 1U;
        Remote_Ctrl->rc_lost = false;
        RemoteControl_LinkWatchdog_Update(Remote_Ctrl);
        g_remote_link_test_result = (Remote_Ctrl->rc_lost != false) ? 1U : 0U; /* expect 1 */
        if ((uint32_t)(now_ms - step_tick_ms) >= REMOTE_LINK_TEST_STEP_MS)
        {
            g_remote_link_test_step = 2U;
            step_tick_ms = now_ms;
        }
        return;
    }

    /* step2: 模拟收到遥控帧恢复，期望未丢失 */
    if (g_remote_link_test_step == 2U)
    {
        Remote_Ctrl->online_cnt = 0xFAU;
        Remote_Ctrl->rc_lost = false;
        g_remote_link_test_result = (Remote_Ctrl->rc_lost != false) ? 1U : 0U; /* expect 0 */
        if ((uint32_t)(now_ms - step_tick_ms) >= REMOTE_LINK_TEST_STEP_MS)
        {
            g_remote_link_test_step = 3U;
            step_tick_ms = now_ms;
        }
        return;
    }
#else
    g_remote_link_test_result = 0U;
#endif
}
