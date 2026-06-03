/**
 * @file chassis_vel_pid.c
 * @brief 底盘分轴速度PID参数 — Watch窗口在线调参用
 * @note  vy(前后,低摩擦)和vw(左右,高摩擦)分开调参
 *        速度PI逻辑在 odom_nav_goto.c 中实现（与位置环同级）
 */

#include "chassis_vel_pid.h"

/* 默认参数配置 */
volatile ChassisVelPID g_chassis_vel_pid = {
    .enable = 1U,
    
    /* vy 前后通道 — 低摩擦，小增益 */
    .vy_kp = 0.8f,
    .vy_ki = 0.05f,
    .vy_kd = 0.2f,
    .vy_i_limit = 5.0f,
    .vy_out_limit = 8.0f,
    
    /* vw 左右通道 — 高摩擦，大增益 */
    .vw_kp = 0.8f,
    .vw_ki = 0.15f,
    .vw_kd = 0.0f,
    .vw_i_limit = 10.0f,
    .vw_out_limit = 15.0f,
    
    /* 内部状态清零 */
    .vy_i_term = 0.0f,
    .vy_last_err = 0.0f,
    .vy_output = 0.0f,
    .vw_i_term = 0.0f,
    .vw_last_err = 0.0f,
    .vw_output = 0.0f,
    .last_tick_ms = 0U,
    .inited = 0U,
};