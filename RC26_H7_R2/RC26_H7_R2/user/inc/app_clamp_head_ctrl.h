#ifndef APP_CLAMP_HEAD_CTRL_H
#define APP_CLAMP_HEAD_CTRL_H

#include <stdint.h>

typedef enum
{
    app_clamp_head_state_idle = 0,              // 空闲
    app_clamp_head_state_wait_close_delay,      // 等待关闭延迟
    app_clamp_head_state_upright_hold,          // 直立保持
    app_clamp_head_state_dock_ok,               // 对接成功
} AppClampHeadState;

/**
 * @brief 模块初始化（状态机复位、舵机回中、夹爪松开）
 */
void AppClampHeadCtrl_Init(void);              // 初始化

/**
 * @brief 周期运行函数（放在控制循环中反复调用）
 */
void AppClampHeadCtrl_Run(void);              // 运行

/**
 * @brief 上位机通知对接成功：
 *        在直立保持状态下调用，会切换到“对接成功”状态（舵机保持直立 + 夹爪松开）。
 */
void AppClampHeadCtrl_NotifyDockOk(void);      // 通知对接成功

/**
 * @brief 读取当前状态（只读）
 * @return 当前状态枚举值
 */
AppClampHeadState AppClampHeadCtrl_GetState(void);  // 获取状态

/** PE9 原始：1=有物（与 APP_CLAMP_HEAD_OBJECT_PRESENT_LEVEL 一致） */
uint8_t AppClampHeadCtrl_IsObjectPresentRaw(void);

#endif /* APP_CLAMP_HEAD_CTRL_H */
