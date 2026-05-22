/**
 * @file odom_nav_goto.h
 * @brief 基于里程计的平面点到点：世界系位置误差 → 车体系 **前后 Vy + 左右 Vw**，**Vx=0**（不控航向、不对准终端朝向）。
 *
 * 调参改 @ref g_odom_nav_goto_tune（volatile，可在线写）；其中 @c last_run_return 为最近一次 @ref odom_nav_goto_run 返回值（数值同 @ref odom_nav_goto_err_t）。
 *
 * @par 用法
 * - 业务层只调用 @ref odom_nav_goto_set_target 设点，再用 @ref odom_nav_goto_peek_last_run_result 读结果。
 * - 周期执行统一由 @ref odom_nav_goto_service_tick 调用 @ref odom_nav_goto_run，避免多任务双跑。
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
    ODOM_NAV_GOTO_ERR_DISARMED = 6,//已卸权，run 不写底盘；须 set_target 重新 arm
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
 * @brief 世界系远近分区双积分 PI(D) + 固定 vmax 限幅 + 到位与超时
 */
typedef struct {
    volatile float kp_far;   /* 远区 P */
    volatile float kp_near;  /* 近区 P */
    volatile float ki_far;   /* 远区 I 增益（仅远区积分器） */
    volatile float ki_near;  /* 近区 I 增益（仅近区积分器） */
    volatile float kd_xy;    /* D（共用） */

    volatile float vmax_forward; /* 世界系合速度 / Vy 上限 */
    volatile float vmax_strafe;

    /** 滞回：dist>far_enter 进远区；dist<near_enter 进近区（须 near_enter<far_enter） */
    volatile float zone_far_enter_m;
    volatile float zone_near_enter_m;
    volatile float i_far_limit;  /* 远区 ix/iy 限幅 */
    volatile float i_near_limit; /* 近区 ix/iy 限幅 */

    volatile float position_tolerance_m;
    volatile uint32_t arrival_confirm_cycles;
    volatile uint32_t timeout_ms;

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
    /** 由雷达 ODOM 换算的车心世界坐标；center_valid=0 时无效 */
    volatile float center_x_m;
    volatile float center_y_m;
    volatile uint8_t center_valid;
} odom_nav_goto_dbg_t;

extern volatile odom_nav_goto_dbg_t g_odom_nav_goto_dbg;

#endif

void odom_nav_goto_clear_state(void);//清零状态

/**
 * @brief 卸权：清底盘 override 与 PI/到位状态；disarm 后 run 返回 DISARMED 且不控 Vy/Vw
 */
void odom_nav_goto_disarm(void);

/** @return 1 已 arm（set_target 之后），0 已 disarm */
uint8_t odom_nav_goto_is_armed(void);

/**
 * @brief 设置导航目标坐标并自动刷新会话号（同时 arm）
 * @param x_m    世界系目标X（米）
 * @param y_m    世界系目标Y（米）
 */
void odom_nav_goto_set_target(float x_m, float y_m);




/**
 * @brief 周期执行一次“到指定坐标”控制
 * @param target 导航目标（世界系 x/y + session_id；换目标需递增 session_id）
 * @param status 可选输出；传 NULL 表示不关心状态
 * @return ODOM_NAV_GOTO_ERR_OK_MOVING   正在运动中
 *         ODOM_NAV_GOTO_ERR_OK_ARRIVED  连续 N 次在容差内到位后自动 disarm
 *         ODOM_NAV_GOTO_ERR_DISARMED    已 disarm，不控车
 *         其余值为参数/配置/里程计/超时错误
 *
 * 行为：远近滞回双积分器 + 抗饱和 + 固定 vmax；世界系 PI(D) 旋到车体系 Vy/Vw（Vx=0）。
 */
odom_nav_goto_err_t odom_nav_goto_run(const odom_nav_goto_target_t *target, odom_nav_goto_status_t *status);

/**
 * @brief 底盘周期唯一入口：全自动且流程未占 VY 时对 @ref odom_nav_target 执行一次 run
 * @note  二区/其它模块只 set_target + peek，不得再调 run
 */
void odom_nav_goto_service_tick(void);

/** 读取上一拍 service_tick（或最近一次 run）的返回值；尚未跑过则 DISARMED */
odom_nav_goto_err_t odom_nav_goto_peek_last_run_result(void);

const odom_nav_goto_status_t *odom_nav_goto_peek_last_status(void);

#if ODOM_NAV_GOTO_WATCH_DEBUG
/**
 * @brief 台架：fire 边沿写 target；实际 run 仅由 @ref odom_nav_goto_service_tick 执行
 */
void odom_nav_goto_poll_debug(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ODOM_NAV_GOTO_H */
