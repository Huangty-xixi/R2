#ifndef APP_FLOW_DISPATCH_H
#define APP_FLOW_DISPATCH_H

/**
 * @brief 业务流程调度模块初始化（注册默认函数实现并复位状态机）。
 */
void AppFlowDispatch_Init(void);

/**
 * @brief 业务流程调度循环函数（放在周期任务中反复调用）。
 */
void AppFlowDispatch_Run(void);

#endif /* APP_FLOW_DISPATCH_H */
