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

typedef enum {
    ODOM_NAV_GOTO_ERR_OK_ARRIVED = 0,
    ODOM_NAV_GOTO_ERR_OK_MOVING = 1,
    ODOM_NAV_GOTO_ERR_NULL_POINTER = 2,
    ODOM_NAV_GOTO_ERR_BAD_CONFIG = 3,
    ODOM_NAV_GOTO_ERR_ODOM_READ = 4,
    ODOM_NAV_GOTO_ERR_TIMEOUT = 5,
} odom_nav_goto_err_t;

typedef struct {
    float distance_to_target_m;
    float vy_cmd;
    float vw_cmd;
    uint8_t at_xy;
} odom_nav_goto_status_t;

typedef struct {
    float x_m;
    float y_m;
    uint32_t session_id;
} odom_nav_goto_target_t;

/**
 * @brief 世界系平面 PID（ex/ey）+ 车体系前后/左右限幅 + 到位与超时
 */
typedef struct {
    volatile float kp_xy;
    volatile float ki_xy;
    volatile float kd_xy;

    volatile float vmax_forward;
    volatile float vmax_strafe;

    volatile float position_tolerance_m;
    volatile uint32_t timeout_ms;

    /** 世界系 ex/ey 积分限幅（各轴） */
    volatile float i_xy_limit;
} odom_nav_goto_tune_t;

extern volatile odom_nav_goto_tune_t g_odom_nav_goto_tune;

void odom_nav_goto_clear_state(void);

odom_nav_goto_err_t odom_nav_goto_run(const odom_nav_goto_target_t *target, odom_nav_goto_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_NAV_GOTO_H */
