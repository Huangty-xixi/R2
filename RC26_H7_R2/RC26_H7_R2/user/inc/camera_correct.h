/** @file camera_correct.h
 * @brief 摄像头KFS横向纠偏 PID 控制器，导航到位后精调阶段调用
 *
 * 使用: CameraCorrect_Reset() 重置 → 每摄像头帧 CameraCorrect_Update(error_m) → Vw写底盘
 *        CameraCorrect_IsDone() 连续N帧在死区 → 精调完成
 *        CameraCorrect_IsTimeout() 超时兜底 → 跳过精调
 */
#ifndef CAMERA_CORRECT_H
#define CAMERA_CORRECT_H

#include <stdint.h>

typedef struct {
    volatile float kp;           /* 比例系数 */
    volatile float ki;           /* 积分系数 */
    volatile float kd;           /* 微分系数 */
    volatile float i_max;        /* 积分限幅，防饱和 */
    volatile float out_max;      /* Vw输出限幅 (m/s) */
    volatile float dead_zone;    /* 死区 (m)，|error|小于此值不纠偏，建议0.02 */
    volatile uint8_t stable_need;/* 连续多少帧在死区内算done，建议3 */
    volatile uint32_t timeout_ms;/* 精调总超时 (ms) */
    volatile uint32_t data_timeout_ms; /* 摄像头数据超时 (ms)，无新帧视为invalid */
} CameraCorrectCfg;

/** 重置PID状态 + 记录开始时间，进入精调阶段时调用 */
void CameraCorrect_Reset(void);

/** 每收到一帧摄像头数据时调用一次，返回Vw命令值(m/s) */
float CameraCorrect_Update(float error_m);

/** 精调是否完成 (连续 stable_need 帧 |error|<dead_zone) */
uint8_t CameraCorrect_IsDone(void);

/** 是否超时 (无摄像头帧或总时间超时) */
uint8_t CameraCorrect_IsTimeout(void);

/** 外部可调的参数实例 */

float camera_kfs_to_lateral_error(float x, float y, float z);
void CameraCorrect_DebugRun(void);
void CameraCorrect_DebugExit(void);
extern volatile CameraCorrectCfg g_camera_correct_cfg;
extern volatile uint8_t g_camera_heartbeat_enable;
#endif /* CAMERA_CORRECT_H */
