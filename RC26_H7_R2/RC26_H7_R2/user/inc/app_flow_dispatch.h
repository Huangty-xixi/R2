#ifndef APP_FLOW_DISPATCH_H
#define APP_FLOW_DISPATCH_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t enable;
    volatile uint32_t seq;
    volatile uint32_t now_ms;
    volatile uint32_t flow_state;
    volatile uint32_t flow_action;
    volatile uint32_t limit_debounce_ms;
    volatile float cmd_vy;
    volatile float cmd_vw;
    volatile float meas_chassis_rpm_abs;
    volatile float target_x_m;
    volatile float target_y_m;
} AppFlowDispatchDebug;

/**
 * @brief 业务流程调度模块初始化（注册默认函数实现并复位状态机）。
 */
void AppFlowDispatch_Init(void);

/**
 * @brief 业务流程调度循环函数（放在周期任务中反复调用）。
 */
void AppFlowDispatch_Run(void);

/**
 * @brief 外部对接成功通知（例如来自 CAN 指令）。
 */
void AppFlowDispatch_NotifyDockOk(void);

extern volatile AppFlowDispatchDebug g_app_flow_dispatch_debug;

#endif /* APP_FLOW_DISPATCH_H */
