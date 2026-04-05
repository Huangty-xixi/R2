#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

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
	uint8_t online_cnt;   /*!< online count */
 } Remote_Info_Typedef;

extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
extern Remote_Info_Typedef RCctrl;
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl);
int16_t data_convert(int src, int src_min, int src_max, float dst_low, float dst_high);

#endif
