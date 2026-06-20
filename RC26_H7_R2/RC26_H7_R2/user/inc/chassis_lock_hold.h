/**
 * @file chassis_lock_hold.h
 * @brief 四轮底盘速度环锁死：目标 rpm=0，用反馈输出反向电流抵抗外力（仅 CAN1 底盘轮）
 *
 * 不负责：航向保持、导轮 CAN2、Chassis_Calc 混控/override。
 *
 * === 业务调用链 ===
 * Can_Task(full_auto) / 遥控 chassis_mode
 *   -> manual_chassis_function()              [chassis.c]
 *   -> chassis_run_auto_output()
 *   -> Chassis_ServiceTick()                 // odom/yaw 状态更新，锁死时仍跑
 *   -> ChassisLockHold_ShouldRun()
 *        |-- AppZone3_IsOnR1() == 1         // 三区上楼完成后 on_r1=1
 *        |-- g_chassis_lock_hold_dbg.force_enable == 1  // debug（需 CHASSIS_LOCK_HOLD_DBG_FORCE）
 *   -> [锁死] ChassisLockHold_Run()
 *        -> Motor_PID_Calculate(motor, 0)    // 反馈 speed_rpm
 *        -> DJIset_motor_data(CAN1 0x200)
 *   -> [正常] Chassis_Calc() + DJIset_motor_data + Chassis_Can2_PublishGuide()
 *   -> 下降沿 ChassisLockHold_Reset()        // 退出锁死清 PID，防切回跳变
 *
 * === API ===
 * | 函数                         | 调用者              | 作用                          |
 * | ChassisLockHold_ShouldRun()  | chassis_run_auto    | on_r1 或 debug force 时为 1   |
 * | ChassisLockHold_Run()        | chassis_run_auto    | 四轮 hold0 + 发 CAN1          |
 * | ChassisLockHold_Reset()      | 锁死下降沿          | 清四轮速度环积分/输出         |
 *
 * === Keil Watch 调试 ===
 * 1. 全自动或遥控 chassis_mode 下测试
 * 2. g_chassis_lock_hold_dbg.force_enable = 1  （需 app_init.h CHASSIS_LOCK_HOLD_DBG_FORCE=1）
 * 3. 观察 active、rpm[0..3]、out[0..3]；手推时 rpm 偏离 0、out 反向制动
 * 4. 调 g_chassis_lock_hold_cfg.rpm_deadband（默认 3，太小抖、太大抗力弱）
 * 5. 抗力不足 -> 调 chassis_motor*_pid_param 的 Kp / limitOutput
 * 6. force_enable = 0 且 AppZone3_IsOnR1()=0 -> 恢复 Chassis_Calc
 * 7. 正式比赛：CHASSIS_LOCK_HOLD_DBG_FORCE 置 0，禁止 Watch 强制锁死
 */
#ifndef CHASSIS_LOCK_HOLD_H
#define CHASSIS_LOCK_HOLD_H

#include <stdint.h>

typedef struct
{
    float rpm_deadband;   /* |rpm| 低于此值时输出置 0，减抖 */
} ChassisLockHoldCfg;

typedef struct
{
    uint8_t force_enable; /* 写：Watch 强制锁死测试（需 CHASSIS_LOCK_HOLD_DBG_FORCE） */
    uint8_t active;       /* 读：当前是否在锁死 */
    int16_t rpm[4];       /* 读：四轮反馈 rpm（LF,RF,RR,LR 与 chassis_motor1~4 一致） */
    int16_t out[4];       /* 读：四轮 CAN1 输出电流 */
} ChassisLockHoldDbg;

extern volatile ChassisLockHoldCfg g_chassis_lock_hold_cfg;
extern volatile ChassisLockHoldDbg g_chassis_lock_hold_dbg;

/** on_r1 或 debug force 时为 1 */
uint8_t ChassisLockHold_ShouldRun(void);

/** 四轮 target_rpm=0，PID 输出发 CAN1；不碰导轮 CAN2 */
void ChassisLockHold_Run(void);

/** 退出锁死时清四轮速度环积分/输出 */
void ChassisLockHold_Reset(void);

#endif /* CHASSIS_LOCK_HOLD_H */
