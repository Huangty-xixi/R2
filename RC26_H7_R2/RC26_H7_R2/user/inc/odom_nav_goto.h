/**
 * @file odom_nav_goto.h
 * @brief 基于里程计的平面点到点：世界系位置误差 → 车体系 **前后 Vy + 左右 Vw**，**Vx=0**（不控航向、不对准终端朝向）。
 *
 * 调参改 @ref g_odom_nav_goto_tune（volatile，可在线写）；其中 @c last_run_return 为最近一次 @ref odom_nav_goto_run 返回值（数值同 @ref odom_nav_goto_err_t）。
 *
 * @par 用法
 * - 每周期 @ref odom_nav_goto_run(&target, status_opt)。
 * - 换目标递增 @c target.session_id；改参后可 @ref odom_nav_goto_clear_state。
 *
 * @date&author 2026/5/4 Hty
 */
#ifndef ODOM_NAV_GOTO_H
#define ODOM_NAV_GOTO_H

#include <stdint.h>

#include "app_init.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 里程计导航到点错误枚举
 */
typedef enum {
    ODOM_NAV_GOTO_ERR_OK_ARRIVED = 0,//到达目标
    ODOM_NAV_GOTO_ERR_OK_MOVING = 1,//移动中
    ODOM_NAV_GOTO_ERR_NULL_POINTER = 2,//空指针
    ODOM_NAV_GOTO_ERR_BAD_CONFIG = 3,//配置错误
    ODOM_NAV_GOTO_ERR_ODOM_READ = 4,//里程计读取错误
    ODOM_NAV_GOTO_ERR_TIMEOUT = 5,//超时
} odom_nav_goto_err_t;

/**
 * @brief 里程计导航到点状态结构体
 */
typedef struct {
    float distance_to_target_m;//到目标距离
    float vy_cmd;//前后速度命令
    float vw_cmd;//左右速度命令
    uint8_t at_xy;//到目标标志
} odom_nav_goto_status_t;//到目标状态

/**
 * @brief 里程计导航到点目标结构体
 */
typedef struct {
    float x_m;//x坐标
    float y_m;//y坐标
    uint32_t session_id;//会话id    
} odom_nav_goto_target_t;//到目标目标

/* 跨文件目标：业务层直接改 x/y/session_id */
extern odom_nav_goto_target_t odom_nav_target;

/**
 * @brief 世界系平面 PID（ex/ey）+ 车体系前后/左右限幅 + 到位与超时
 */
typedef struct {
    volatile float kp_xy;//x轴比例增益
    volatile float ki_xy;//x轴积分增益
    volatile float kd_xy;//x轴微分增益

    volatile float vmax_forward;//最大前后速度
    volatile float vmax_strafe;//最大左右速度

    volatile float position_tolerance_m;//位置误差容差
    volatile uint32_t timeout_ms;//超时时间

    /** 世界系 ex/ey 积分限幅（各轴） */
    volatile float i_xy_limit;//积分限幅

    /** 最近一次 @ref odom_nav_goto_run 返回值，数值同 @ref odom_nav_goto_err_t（0xFFFFFFFF=尚未跑过 run） */
    volatile uint32_t last_run_return;
} odom_nav_goto_tune_t;

extern volatile odom_nav_goto_tune_t g_odom_nav_goto_tune;//里程计导航到点参数

#if ODOM_NAV_GOTO_WATCH_DEBUG
/** 调试到点：Watch @ref g_odom_nav_goto_dbg（半自动空闲 + poll 挂载） */
typedef struct {
    volatile uint8_t enable;
    volatile float target_x_m;
    volatile float target_y_m;
    volatile uint32_t fire;
    /** 最近一次 @ref odom_nav_goto_run 返回值，数值同 @ref odom_nav_goto_err_t（每轮 poll 且已 fire 时更新） */
    volatile uint32_t last_run_return; /* 0xFFFFFFFF=尚未在 debug 路径中跑过 run */
} odom_nav_goto_dbg_t;

extern volatile odom_nav_goto_dbg_t g_odom_nav_goto_dbg;

#endif

void odom_nav_goto_clear_state(void);//清零状态

/**
 * @brief 设置导航目标坐标并自动刷新会话号
 * @param x_m    世界系目标X（米）
 * @param y_m    世界系目标Y（米）
 */
void odom_nav_goto_set_target(float x_m, float y_m);




/**
 * @brief 周期执行一次“到指定坐标”控制
 * @param target 导航目标（世界系 x/y + session_id；换目标需递增 session_id）
 * @param status 可选输出；传 NULL 表示不关心状态
 * @return ODOM_NAV_GOTO_ERR_OK_MOVING   正在运动中
 *         ODOM_NAV_GOTO_ERR_OK_ARRIVED  已进入位置容差；同 session 内保持到位不再 PI（直至换目标/session）
 *         其余值为参数/配置/里程计/超时错误
 *
 * 行为：读取里程计位姿，计算世界系位置误差，PI(D)求速度，再按 yaw 旋到车体系 Vy/Vw，
 * 并通过 process_flow_chassis_override 下发（Vx 固定为 0）。
 */
odom_nav_goto_err_t odom_nav_goto_run(const odom_nav_goto_target_t *target, odom_nav_goto_status_t *status);



#if ODOM_NAV_GOTO_WATCH_DEBUG
/**
 * @brief 调试：在半自动且无楼梯/KFS 流程时周期调用（已由 manual_chassis_function 挂载）。
 * Watch：@c g_odom_nav_goto_dbg.enable=1，写 target_x_m/target_y_m，再将 fire 加 1 触发新一轮；
 * 每周期 @c g_odom_nav_goto_dbg.last_run_return 为本次 @ref odom_nav_goto_run 返回值。
 */
void odom_nav_goto_poll_debug(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ODOM_NAV_GOTO_H */
