#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "app_init.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

#define SBUS_RX_BUF_NUM 50u
#define RC_FRAME_LENGTH 25u
#define RC_CH_VALUE_OFFSET 1024U
#define KFS_AXIS_LIFT_MINPS -3.0f
#define KFS_AXIS_LIFT_MAXPOS 3.0f

/* REMOTE_LOST_PROTECT_ENABLE / REMOTE_LINK_TEST_ENABLE：默认值见 app_init.h */

/** 上电后禁止判丢的屏蔽时间（ms），避免接收机/DMA 未稳定误报 */
#ifndef REMOTE_LINK_GRACE_MS
#define REMOTE_LINK_GRACE_MS 400U
#endif

/** 距上次“有效 SBUS 帧”超过该时间（ms）则判链路丢失 */
#ifndef REMOTE_LINK_TIMEOUT_MS
#define REMOTE_LINK_TIMEOUT_MS 200U
#endif

/** 连续 failsafe/frame-lost 帧数达到该值才置 rc_lost */
#ifndef REMOTE_FAILSAFE_DEBOUNCE
#define REMOTE_FAILSAFE_DEBOUNCE 12U
#endif

/** 连续正常帧数达到该值才清除 rc_lost（恢复迟滞） */
#ifndef REMOTE_RECOVER_DEBOUNCE
#define REMOTE_RECOVER_DEBOUNCE 5U
#endif

/** 遥控链路看门狗自测每步间隔（ms） */
#define REMOTE_LINK_TEST_STEP_MS 5000U

//R_HORIZONTAL
#define CH1_LOW     192       //LEFT        
#define CH1_HIGH    1792        //RIGHT
#define CH1_MID     992        //MID
#define LR_TRANSLATION          data_convert(RCctrl.CH1, CH1_LOW, CH1_HIGH, -ACCEL, ACCEL)
//R_UPRIGHT
#define CH2_LOW     192         //DOWN
#define CH2_HIGH    1792        //UP
#define CH2_MID     992        //MID
#define FB_TRANSLATION          data_convert(RCctrl.CH2, CH2_LOW, CH2_HIGH, -ACCEL, ACCEL)
#define KFS_FLEXIBLE            data_convert(RCctrl.CH2, CH2_LOW, CH2_HIGH, -3, 3)
//L_UPRIGHT
#define CH3_LOW     192         //DOWN
#define CH3_HIGH    1792        //UP
#define CH3_MID     992        //MID
#define ACCEL                   data_convert(RCctrl.CH3, CH3_LOW, CH3_HIGH, 0, 100)
#define KFS_AXIS_LIFT           data_convert(RCctrl.CH3, CH3_MID, CH3_LOW, KFS_AXIS_LIFT_MINPS, KFS_AXIS_LIFT_MAXPOS)
//L_HORIZONTAL
#define CH4_LOW     192         //LEFT
#define CH4_HIGH    1792        //RIGHT
#define CH4_MID     992         //MID
#define ROTATION               40.2814f/4 * data_convert(RCctrl.CH4, CH4_LOW, CH4_HIGH, -5, 5)
//CHANNAL_C
#define CH5_LOW     192         //DOWN
#define CH5_HIGH    1792        //UP
#define CH5_MID     992        //MID
//CHANNEL_D
#define CH6_LOW     1792        //DOWN
#define CH6_HIGH    192         //UP
//CHANNEL_G
#define CH7_LOW     192         //BACK
#define CH7_HIGH    1792        //FRONT
#define CH7_MID     992        //MID
//CHANNEL_H
#define CH8_LOW     192         //PRESS
#define CH8_HIGH    1792        //RELEASE
//CHANNEL_F
#define CH9_LOW     192         //BACK
#define CH9_HIGH    1792        //FRONT
//CHANNEL_A
#define CH10_LOW    1792        //UP
#define CH10_HIGH   192         //DOWN

typedef  struct
{
    uint16_t CH1;  
    uint16_t CH2;  
    uint16_t CH3;  
    uint16_t CH4;
    uint16_t CH5;
    uint16_t CH6;
    uint16_t CH7;
    uint16_t CH8;
    uint16_t CH9;
    uint16_t CH10;
	uint16_t CH11;
	uint16_t CH12;
    uint16_t CH13;
    uint16_t CH14;
    uint16_t CH15;
    uint16_t CH16;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< 调试：连续正常帧计数（与恢复去抖相关） */
 } Remote_Info_Typedef;

/** 遥控链路调试计数（Keil Watch: g_rc_link_dbg） */
typedef struct
{
    uint32_t frame_ok;
    uint32_t frame_reject;
    uint32_t failsafe_frame;
    uint32_t timeout_lost;
} rc_link_dbg_t;

extern volatile rc_link_dbg_t g_rc_link_dbg;

extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
extern Remote_Info_Typedef RCctrl;
void RemoteControl_Link_Init(void);
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl);
int16_t data_convert(int src, int src_min, int src_max, float dst_low, float dst_high);
void RemoteControl_LinkWatchdog_Update(Remote_Info_Typedef *Remote_Ctrl);
void RemoteControl_LinkWatchdog_SimpleTest(Remote_Info_Typedef *Remote_Ctrl);
extern volatile uint8_t g_remote_link_test_step;
extern volatile uint8_t g_remote_link_test_result;

#endif
