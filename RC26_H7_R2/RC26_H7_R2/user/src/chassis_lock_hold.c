/**
 * @file chassis_lock_hold.c
 * @brief 上 R1 后四轮底盘主动抱死（速度环 hold 0）
 *
 * 算法：每轮 target_rpm=0，Motor_PID_Calculate(motor, 0)，反馈为 CAN 回传的 speed_rpm。
 * |rpm| < rpm_deadband 时输出 0 减抖；否则 PID 输出反向电流抵抗外力。
 * 复用 chassis_motor*_pid_param，不单独维护一套 PID 参数。
 *
 * 触发：AppZone3_IsOnR1()（up_r1_run 完成置 on_r1=1）或 debug force_enable。
 */

#include "chassis_lock_hold.h"

#include "app_init.h"
#include "app_zone3.h"
#include "chassis.h"
#include "dji_motor.h"
#include "fdcan.h"
#include "pid.h"

#include <math.h>

volatile ChassisLockHoldCfg g_chassis_lock_hold_cfg = {
    .rpm_deadband = 3.0f,
};

volatile ChassisLockHoldDbg g_chassis_lock_hold_dbg;

static void chassis_lock_hold_reset_motor(DJI_MotorModule *motor)
{
    if (motor == NULL || motor->pid_spd.PID_Calc_Clear == NULL)
    {
        return;
    }

    motor->pid_spd.PID_Calc_Clear(&motor->pid_spd);
}

static float chassis_lock_hold_run_motor(DJI_MotorModule *motor)
{
    float out;

    if (motor == NULL)
    {
        return 0.0f;
    }

    if (fabsf((float)motor->speed_rpm) < g_chassis_lock_hold_cfg.rpm_deadband)
    {
        motor->pid_spd.Output = 0.0f;
        return 0.0f;
    }

    out = motor->PID_Calculate(motor, 0.0f);
    VAL_LIMIT(out, -motor->pid_spd.param.limitOutput, motor->pid_spd.param.limitOutput);
    motor->pid_spd.Output = out;
    return out;
}

uint8_t ChassisLockHold_ShouldRun(void)
{
    if (AppZone3_IsOnR1() != 0U)
    {
        return 1U;
    }

#if CHASSIS_LOCK_HOLD_DBG_FORCE
    if (g_chassis_lock_hold_dbg.force_enable != 0U)
    {
        return 1U;
    }
#endif

    return 0U;
}

void ChassisLockHold_Reset(void)
{
    chassis_lock_hold_reset_motor(&chassis_motor1);
    chassis_lock_hold_reset_motor(&chassis_motor2);
    chassis_lock_hold_reset_motor(&chassis_motor3);
    chassis_lock_hold_reset_motor(&chassis_motor4);

    g_chassis_lock_hold_dbg.active = 0U;
}

void ChassisLockHold_Run(void)
{
    int16_t out1;
    int16_t out2;
    int16_t out3;
    int16_t out4;

    out1 = (int16_t)chassis_lock_hold_run_motor(&chassis_motor1);
    out2 = (int16_t)chassis_lock_hold_run_motor(&chassis_motor2);
    out3 = (int16_t)chassis_lock_hold_run_motor(&chassis_motor3);
    out4 = (int16_t)chassis_lock_hold_run_motor(&chassis_motor4);

    g_chassis_lock_hold_dbg.active = 1U;
    g_chassis_lock_hold_dbg.rpm[0] = chassis_motor1.speed_rpm;
    g_chassis_lock_hold_dbg.rpm[1] = chassis_motor2.speed_rpm;
    g_chassis_lock_hold_dbg.rpm[2] = chassis_motor3.speed_rpm;
    g_chassis_lock_hold_dbg.rpm[3] = chassis_motor4.speed_rpm;
    g_chassis_lock_hold_dbg.out[0] = out1;
    g_chassis_lock_hold_dbg.out[1] = out2;
    g_chassis_lock_hold_dbg.out[2] = out3;
    g_chassis_lock_hold_dbg.out[3] = out4;

    (void)DJIset_motor_data(&hfdcan1, 0x200, out1, out2, out3, out4);
}
