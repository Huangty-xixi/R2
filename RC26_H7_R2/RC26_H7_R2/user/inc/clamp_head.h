#ifndef __CLAMP_HEAD_H__
#define __CLAMP_HEAD_H__

#include "main.h"
#include "tim.h"

// ==================== 状态枚举 ====================
typedef enum {
    CLAMP_HEAD_IDLE = 0,          // 空闲状态
    CLAMP_HEAD_WAIT_CLOSE,        // 等待夹爪闭合
    CLAMP_HEAD_WAIT_DELAY         // 等待延时后直立
} ClampHeadState;

// ==================== 外部状态变量（供其他模块查询）====================
extern ClampHeadState clamp_head_state;

// ==================== 接口函数 ====================
void clamp_head_init(void);
void clamp_head_auto_process(void);

// ==================== zone1专用接口 ====================
void ClampHead_StartSequence(void);  // zone1步骤1：启动夹爪序列（打开夹爪+舵机水平）
void ClampHead_TriggerClose(void);   // zone1步骤5：触发夹爪闭合（光电开关触发后调用）

#endif