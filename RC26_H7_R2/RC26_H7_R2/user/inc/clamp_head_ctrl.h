#ifndef CLAMP_HEAD_CTRL_H
#define CLAMP_HEAD_CTRL_H

#include <stdint.h>

typedef enum
{
    clamp_head_state_idle = 0,              // 空闲
    clamp_head_state_wait_close_delay,      // 等待关闭延迟
    clamp_head_state_upright_hold,          // 直立保持
    clamp_head_state_dock_ok,               // 对接成功
} ClampHeadState;

/**
 * @brief 模块初始化（状态机复位、舵机回中、夹爪松开）
 */
void ClampHeadCtrl_Init(void);

/**
 * @brief 周期运行函数（放在控制循环中反复调用）
 */
void ClampHeadCtrl_Run(void);

/**
 * @brief 上位机通知对接成功：
 *        在直立保持状态下调用，会切换到“对接成功”状态（舵机保持直立 + 夹爪松开）。
 */
void ClampHeadCtrl_NotifyDockOk(void);

/**
 * @brief 读取当前状态（只读）
 * @return 当前状态枚举值
 */
ClampHeadState ClampHeadCtrl_GetState(void);

/** PE9 原始：1=有物（与 CLAMP_HEAD_OBJECT_PRESENT_LEVEL 一致） */
uint8_t ClampHeadCtrl_IsObjectPresentRaw(void);

#endif /* CLAMP_HEAD_CTRL_H */
