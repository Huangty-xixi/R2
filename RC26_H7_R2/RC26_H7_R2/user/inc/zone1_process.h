#ifndef __ZONE1_PROCESS_H__
#define __ZONE1_PROCESS_H__

#include "main.h"

// ==================== 状态枚举 ====================
typedef enum {
    ZONE1_IDLE = 0,           // 空闲状态
    ZONE1_START_CLAMP,        // 步骤1：进入自动夹枪模式
    ZONE1_FORWARD_RADAR,      // 步骤2：底盘前进，坐标判断停止
    ZONE1_ROTATE_LEFT_90,     // 步骤3：左旋转90度
    ZONE1_FORWARD_LIMIT,      // 步骤4：底盘继续前进，限位停
    ZONE1_RIGHT_MOVE_PHOTO,   // 步骤5：向右缓慢平移，光电检测物体停止
	  ZONE1_WAIT_CLAMP_COMPLETE,  // 步骤6：等待夹枪完成
    ZONE1_COMPLETE,           // 流程完成
    ZONE1_ERROR               // 错误状态
} Zone1_State_t;

// ==================== 调试用状态变量（只读，用于监控）====================
// 状态监控
extern Zone1_State_t zone1_current_state;  // 当前状态
extern Zone1_State_t zone1_last_state;     // 上一个状态
extern uint32_t zone1_elapsed_ms;          // 当前状态已运行时间(ms)

// 传感器数据监控
extern uint32_t zone1_laser_distance_mm;   // 激光雷达距离
extern uint8_t zone1_photo_switch_state;   // 光电开关状态
extern float zone1_chassis_speed_sum;      // 底盘4个电机速度之和

// 里程计数据监控
extern float zone1_odom_x;                 // 当前X坐标
extern float zone1_odom_y;                 // 当前Y坐标
extern float zone1_odom_yaw;               // 当前航向角

// 旋转90度监控
extern float zone1_rot_start_yaw;          // 旋转起始角度
extern float zone1_rot_target_yaw;         // 旋转目标角度
extern float zone1_rot_error;              // 旋转误差

// 步骤5向右移动监控
extern float zone1_right_move_distance;     // 步骤5已移动距离(米)

// ==================== 可调参数（可修改）====================
// ---------- 目标坐标参数 ----------
extern float zone1_target_x;               // I区入口X坐标(米)
extern float zone1_target_y;               // I区入口Y坐标(米)
extern float zone1_position_threshold;     // 位置到达阈值(米)

// ---------- 速度参数 ----------
extern float zone1_forward_speed1;         // 第一阶段前进速度(m/s)
extern float zone1_forward_speed2;         // 第二阶段前进速度(m/s)
extern float zone1_right_speed;           // 向右平移速度(m/s)

// ---------- 时间参数 ----------
extern uint32_t zone1_step1_delay_ms;      // 步骤1延时时间(ms)
extern uint32_t zone1_step2_timeout_ms;    // 步骤2超时时间(ms)
extern uint32_t zone1_step3_timeout_ms;    // 步骤3超时时间(ms)
extern uint32_t zone1_step4_timeout_ms;    // 步骤4超时时间(ms)
extern uint32_t zone1_step5_timeout_ms;    // 步骤5超时时间(ms)

// ---------- 限位检测参数 ----------
extern float zone1_limit_speed_start;      // 限位检测：开始移动速度阈值(rpm)
extern float zone1_limit_speed_stop;       // 限位检测：停止速度阈值(rpm)

// ==================== 接口函数 ====================
void Zone1_Init(void);               // 初始化
void Zone1_Start(void);              // 开始流程
void Zone1_Stop(void);               // 停止并重置
void Zone1_Process(void);            // 主循环（需在Can_Task里调用）

#endif
