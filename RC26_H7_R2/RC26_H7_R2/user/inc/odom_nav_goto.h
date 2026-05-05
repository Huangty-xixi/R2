/**
 * @file odom_nav_goto.h
 * @brief 基于里程计的平面点到点：世界系位置误差 → 车体系 **前后 Vy + 左右 Vw**，**Vx=0**（不控航向、不对准终端朝向）。
 *
 * 调参改 @ref g_odom_nav_goto_tune（volatile，可在线写）。
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
} odom_nav_goto_tune_t;

extern volatile odom_nav_goto_tune_t g_odom_nav_goto_tune;//里程计导航到点参数

void odom_nav_goto_clear_state(void);//清零状态

odom_nav_goto_err_t odom_nav_goto_run(const odom_nav_goto_target_t *target, odom_nav_goto_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_NAV_GOTO_H */
