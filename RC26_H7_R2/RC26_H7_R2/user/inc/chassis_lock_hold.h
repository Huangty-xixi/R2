/**
 * @file chassis_lock_hold.h
 * @brief 四轮底盘速度环锁死：目标 rpm=0，独立 PID 抗外力（仅 CAN1 底盘轮）
 *
 * 不负责：航向保持、Chassis_Calc 混控/override。
 * 锁死时导轮 CAN2 清零（Chassis_Can2_PublishGuideZero）。
 *
 * === 业务调用链 ===
 * Can_Task(full_auto) / 遥控 chassis_mode
 *   -> manual_chassis_function()              [chassis.c]
 *   -> chassis_run_auto_output()
 *   -> Chassis_ServiceTick()
 *   -> ChassisLockHold_ShouldRun()
 *   -> [上升沿] ChassisLockHold_OnActivate()   // 清走场/锁死 PID，防积分残留
 *   -> [锁死] ChassisLockHold_Run()
 *        -> f_PID_Calculate(lock_pid, 0, speed_rpm)  // 用 g_chassis_lock_hold_cfg
 *        -> DJIset_motor_data(CAN1 0x200)
 *        -> Chassis_Can2_PublishGuideZero()
 *   -> [下降沿] ChassisLockHold_Reset()
 *   -> [正常] Chassis_Calc() + CAN1 + Chassis_Can2_PublishGuide()
 *
 * === Keil Watch 调试 ===
 * 1. g_chassis_lock_hold_dbg.force_enable = 1（需 CHASSIS_LOCK_HOLD_DBG_FORCE=1）
 * 2. 调 kp/ki/kd 改 g_chassis_lock_hold_cfg（实时生效，勿改 chassis_motor*_pid_param）
 * 3. 观察 g_chassis_lock_hold_dbg.rpm[0..3] / out[0..3]
 * 4. 正式比赛：CHASSIS_LOCK_HOLD_DBG_FORCE 置 0
 */
#ifndef CHASSIS_LOCK_HOLD_H
#define CHASSIS_LOCK_HOLD_H

#include <stdint.h>

#ifndef CHASSIS_LOCK_HOLD_DBG_FORCE
#define CHASSIS_LOCK_HOLD_DBG_FORCE 1U
#endif

#ifndef CHASSIS_LOCK_HOLD_KP
#define CHASSIS_LOCK_HOLD_KP            12.0f
#endif
#ifndef CHASSIS_LOCK_HOLD_KI
#define CHASSIS_LOCK_HOLD_KI            0.15f
#endif
#ifndef CHASSIS_LOCK_HOLD_KD
#define CHASSIS_LOCK_HOLD_KD            0.35f
#endif
#ifndef CHASSIS_LOCK_HOLD_LIMIT_I
#define CHASSIS_LOCK_HOLD_LIMIT_I       800.0f
#endif
#ifndef CHASSIS_LOCK_HOLD_LIMIT_OUT
#define CHASSIS_LOCK_HOLD_LIMIT_OUT     10000.0f
#endif
#ifndef CHASSIS_LOCK_HOLD_RPM_DEADBAND
#define CHASSIS_LOCK_HOLD_RPM_DEADBAND  1.0f
#endif

typedef struct
{
    float rpm_deadband;      /* |rpm| 低于此值：清该轮锁死 PID 并输出 0 */
    float kp;
    float ki;
    float kd;
    float limit_integral;
    float limit_output;
} ChassisLockHoldCfg;

typedef struct
{
    uint8_t force_enable; /* Watch 强制锁死（需 CHASSIS_LOCK_HOLD_DBG_FORCE） */
    uint8_t active;       /* 当前是否在锁死 */
    int16_t rpm[4];       /* 四轮反馈 rpm（与 chassis_motor1~4 一致） */
    int16_t out[4];       /* 四轮 CAN1 电流输出 */
} ChassisLockHoldDbg;

extern volatile ChassisLockHoldCfg g_chassis_lock_hold_cfg;
extern volatile ChassisLockHoldDbg g_chassis_lock_hold_dbg;

uint8_t ChassisLockHold_ShouldRun(void);

/** 进入锁死上升沿：清走场/锁死/导轮 PID 状态 */
void ChassisLockHold_OnActivate(void);

/** 锁死周期：hold0 + CAN1 + 导轮清零 */
void ChassisLockHold_Run(void);

/** 退出锁死下降沿：清锁死 PID，恢复走场前干净状态 */
void ChassisLockHold_Reset(void);

#endif /* CHASSIS_LOCK_HOLD_H */
