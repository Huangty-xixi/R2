/**
 * @file chassis_lock_hold.c
 * @brief 上 R1 后四轮底盘主动抱死（速度环 hold 0）
 */

#include "chassis_lock_hold.h"

#include "chassis.h"
#include "dji_motor.h"
#include "fdcan.h"
#include "pid.h"

#include <math.h>

volatile ChassisLockHoldCfg g_chassis_lock_hold_cfg = {
    .rpm_deadband = 3.0f,
};

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

void ChassisLockHold_Reset(void)
{
    chassis_lock_hold_reset_motor(&chassis_motor1);
    chassis_lock_hold_reset_motor(&chassis_motor2);
    chassis_lock_hold_reset_motor(&chassis_motor3);
    chassis_lock_hold_reset_motor(&chassis_motor4);
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

    (void)DJIset_motor_data(&hfdcan1, 0x200, out1, out2, out3, out4);
}
