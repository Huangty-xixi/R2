#ifndef __CHASSIS_VEL_PID_H__
#define __CHASSIS_VEL_PID_H__

#include <stdint.h>

/**
 * @brief 底盘分轴速度PID结构体
 * @note  vy(前后,低摩擦)和vw(左右,高摩擦)分开调参
 */
typedef struct {
    volatile uint8_t enable;       /* 使能标志 */
    
    /* vy 前后通道 PID 参数 (低摩擦，小增益) */
    volatile float vy_kp;
    volatile float vy_ki;
    volatile float vy_kd;
    volatile float vy_i_limit;
    volatile float vy_out_limit;
    
    /* vw 左右通道 PID 参数 (高摩擦，大增益) */
    volatile float vw_kp;
    volatile float vw_ki;
    volatile float vw_kd;
    volatile float vw_i_limit;
    volatile float vw_out_limit;
    
    /* 内部状态 */
    float vy_i_term;
    float vy_last_err;
    float vy_output;
    float vw_i_term;
    float vw_last_err;
    float vw_output;
    uint32_t last_tick_ms;
    uint8_t inited;
} ChassisVelPID;

extern volatile ChassisVelPID g_chassis_vel_pid;


#endif