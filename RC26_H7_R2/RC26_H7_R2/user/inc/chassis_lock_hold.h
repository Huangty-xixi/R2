/**
 * @file chassis_lock_hold.h
 * @brief 四轮底盘速度环锁死：目标 rpm=0，用反馈抵抗外力（仅 CAN1 底盘轮）
 */
#ifndef CHASSIS_LOCK_HOLD_H
#define CHASSIS_LOCK_HOLD_H

#include <stdint.h>

typedef struct
{
    float rpm_deadband;   /* |rpm| 低于此值时输出置 0，减抖 */
} ChassisLockHoldCfg;

extern volatile ChassisLockHoldCfg g_chassis_lock_hold_cfg;

/** 四轮 target_rpm=0，PID 输出发 CAN1；不碰导轮 CAN2 */
void ChassisLockHold_Run(void);

/** 退出锁死时清四轮速度环积分/输出 */
void ChassisLockHold_Reset(void);

#endif /* CHASSIS_LOCK_HOLD_H */
