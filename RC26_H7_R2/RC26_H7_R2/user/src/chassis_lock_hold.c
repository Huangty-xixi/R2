/**
 * @file chassis_lock_hold.c
 * @brief 上 R1 后四轮底盘主动抱死（独立速度环 PID，目标 rpm=0）
 *
 * 算法：每轮 target_rpm=0，f_PID_Calculate(lock_pid, 0, speed_rpm)。
 * |rpm| < rpm_deadband 时清该轮锁死 PID 并输出 0；否则输出反向电流抵抗外力。
 */

#include "chassis_lock_hold.h"

#include "app_zone3.h"
#include "chassis.h"
#include "dji_motor.h"
#include "fdcan.h"
#include "pid.h"

#include <math.h>
#include <string.h>

volatile ChassisLockHoldCfg g_chassis_lock_hold_cfg = {
    .rpm_deadband = CHASSIS_LOCK_HOLD_RPM_DEADBAND,
    .kp = CHASSIS_LOCK_HOLD_KP,
    .ki = CHASSIS_LOCK_HOLD_KI,
    .kd = CHASSIS_LOCK_HOLD_KD,
    .limit_integral = CHASSIS_LOCK_HOLD_LIMIT_I,
    .limit_output = CHASSIS_LOCK_HOLD_LIMIT_OUT,
};

volatile ChassisLockHoldDbg g_chassis_lock_hold_dbg;

static PID_Info_TypeDef s_lock_pid[4];
static uint8_t s_lock_pid_inited;

static DJI_MotorModule *const s_lock_motor[4] = {
    &chassis_motor1,
    &chassis_motor2,
    &chassis_motor3,
    &chassis_motor4,
};

static float s_lock_pid_param_buf[PID_PARAMETER_NUM];

static void chassis_lock_hold_build_pid_param(void)
{
    s_lock_pid_param_buf[0] = g_chassis_lock_hold_cfg.kp;
    s_lock_pid_param_buf[1] = g_chassis_lock_hold_cfg.ki;
    s_lock_pid_param_buf[2] = g_chassis_lock_hold_cfg.kd;
    s_lock_pid_param_buf[3] = 0.0f;
    s_lock_pid_param_buf[4] = g_chassis_lock_hold_cfg.limit_integral;
    s_lock_pid_param_buf[5] = g_chassis_lock_hold_cfg.limit_output;
}

static void chassis_lock_hold_sync_pid_params(PID_Info_TypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->param.kp = g_chassis_lock_hold_cfg.kp;
    pid->param.ki = g_chassis_lock_hold_cfg.ki;
    pid->param.kd = g_chassis_lock_hold_cfg.kd;
    pid->param.limitIntegral = g_chassis_lock_hold_cfg.limit_integral;
    pid->param.limitOutput = g_chassis_lock_hold_cfg.limit_output;
}

static void chassis_lock_hold_init_lock_pids(void)
{
    uint8_t i;

    if (s_lock_pid_inited != 0U)
    {
        return;
    }

    chassis_lock_hold_build_pid_param();
    for (i = 0U; i < 4U; i++)
    {
        PID_Init(&s_lock_pid[i], PID_POSITION, s_lock_pid_param_buf);
    }
    s_lock_pid_inited = 1U;
}

static void chassis_lock_hold_clear_drive_motor(DJI_MotorModule *motor)
{
    if (motor == NULL || motor->pid_spd.PID_Calc_Clear == NULL)
    {
        return;
    }

    motor->pid_spd.PID_Calc_Clear(&motor->pid_spd);
}

static void chassis_lock_hold_clear_lock_pid(PID_Info_TypeDef *pid)
{
    if (pid == NULL || pid->PID_Calc_Clear == NULL)
    {
        return;
    }

    pid->PID_Calc_Clear(pid);
}

static void chassis_lock_hold_clear_all_lock_pids(void)
{
    uint8_t i;

    chassis_lock_hold_init_lock_pids();
    for (i = 0U; i < 4U; i++)
    {
        chassis_lock_hold_clear_lock_pid(&s_lock_pid[i]);
    }
}

static float chassis_lock_hold_run_wheel(uint8_t idx)
{
    DJI_MotorModule *motor;
    PID_Info_TypeDef *pid;
    float out;
    float rpm;

    if (idx >= 4U)
    {
        return 0.0f;
    }

    motor = s_lock_motor[idx];
    pid = &s_lock_pid[idx];
    if (motor == NULL)
    {
        return 0.0f;
    }

    chassis_lock_hold_init_lock_pids();
    chassis_lock_hold_sync_pid_params(pid);

    rpm = (float)motor->speed_rpm;
    if (fabsf(rpm) < g_chassis_lock_hold_cfg.rpm_deadband)
    {
        /* 输出置 0，保留 Integral/Err，避免边界反复清零振荡 */
        return 0.0f;
    }

    out = f_PID_Calculate(pid, 0.0f, rpm);
    VAL_LIMIT(out, -pid->param.limitOutput, pid->param.limitOutput);
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

void ChassisLockHold_OnActivate(void)
{
    uint8_t i;

    chassis_lock_hold_clear_all_lock_pids();
    for (i = 0U; i < 4U; i++)
    {
        chassis_lock_hold_clear_drive_motor(s_lock_motor[i]);
    }

    guide_motor1.pid_spd.Output = 0.0f;
    guide_motor2.pid_spd.Output = 0.0f;
    if (guide_motor1.pid_spd.PID_Calc_Clear != NULL)
    {
        guide_motor1.pid_spd.PID_Calc_Clear(&guide_motor1.pid_spd);
    }
    if (guide_motor2.pid_spd.PID_Calc_Clear != NULL)
    {
        guide_motor2.pid_spd.PID_Calc_Clear(&guide_motor2.pid_spd);
    }

    Chassis_Can2_PublishGuideZero();
    g_chassis_lock_hold_dbg.active = 0U;
}

void ChassisLockHold_Reset(void)
{
    uint8_t i;

    chassis_lock_hold_clear_all_lock_pids();
    for (i = 0U; i < 4U; i++)
    {
        chassis_lock_hold_clear_drive_motor(s_lock_motor[i]);
    }

    g_chassis_lock_hold_dbg.active = 0U;
}

void ChassisLockHold_Run(void)
{
    int16_t out1;
    int16_t out2;
    int16_t out3;
    int16_t out4;

    out1 = (int16_t)chassis_lock_hold_run_wheel(0U);
    out2 = (int16_t)chassis_lock_hold_run_wheel(1U);
    out3 = (int16_t)chassis_lock_hold_run_wheel(2U);
    out4 = (int16_t)chassis_lock_hold_run_wheel(3U);

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
    Chassis_Can2_PublishGuideZero();
}
