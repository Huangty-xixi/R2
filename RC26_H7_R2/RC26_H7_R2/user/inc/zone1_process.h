#ifndef __ZONE1_PROCESS_H__
#define __ZONE1_PROCESS_H__

#include "main.h"

// ==================== 状态枚举 ====================
typedef enum {
    ZONE1_IDLE = 0,           // 空闲状态
    ZONE1_START_CLAMP,        // 步骤1：进入自动夹枪模式
    ZONE1_FORWARD_RADAR,      // 步骤2：底盘前进，雷达停
    ZONE1_ROTATE_LEFT_90,     // 步骤3：左旋转90度
    ZONE1_FORWARD_LIMIT,      // 步骤4：底盘继续前进，限位停
    ZONE1_LR_PHOTO_SWITCH,    // 步骤5：左右移动，光电触发夹枪
	  ZONE1_WAIT_CLAMP_COMPLETE,  // 新增：等待夹枪完成
    ZONE1_COMPLETE,           // 流程完成
    ZONE1_ERROR               // 错误状态
} Zone1_State_t;

// ==================== 调试用状态变量（只读）====================
extern Zone1_State_t zone1_state;
extern uint32_t zone1_tick_debug;
extern uint8_t zone1_moving_flag_debug;

// ==================== 可调参数（可修改）====================
extern uint32_t zone1_radar_stop_distance_mm;    // 雷达停止距离
extern float zone1_forward_speed1;               // 第一阶段前进速度
extern float zone1_forward_speed2;               // 第二阶段前进速度
extern float zone1_left_right_speed;             // 左右移动速度

// ==================== 接口函数 ====================
void Zone1_Init(void);               // 初始化
void Zone1_Start(void);              // 开始流程
void Zone1_Stop(void);               // 停止并重置
void Zone1_Process(void);            // 主循环（需在Can_Task里调用）

#endif