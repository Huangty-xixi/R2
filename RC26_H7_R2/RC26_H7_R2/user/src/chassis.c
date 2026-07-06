#include "chassis.h"
#include "Motion_Task.h"
#include "master_control.h"
#include "Sensor_Task.h"
#include "chassis_heading_hold.h"
#include "odom_nav_goto.h"
#include "nav_goto_dingdian_debug.h"
#include "Process_Flow.h"
#include "yaw_heading_ctrl.h"
#include "chassis_vel_pid.h"
#include "chassis_lock_hold.h"
#include <math.h>

Chassis_Module Chassis;

DJI_MotorModule chassis_motor1;
DJI_MotorModule chassis_motor2;
DJI_MotorModule chassis_motor3;
DJI_MotorModule chassis_motor4;

DJI_MotorModule guide_motor1;
DJI_MotorModule guide_motor2;

uint16_t switch_state;

volatile ChassisDebugSnapshot g_chassis_dbg = {0};
volatile chassis_speed_rpm_t g_chassis_speed;  /* 底盘四轮转速，每 tick 刷新 */

static void chassis_control_resolve_cmd(Chassis_Module *chassis, ChassisControlCmd *cmd_out)
{
    if (chassis == 0 || cmd_out == 0) return;

    if ((control_mode == remote_control && remote_mode == chassis_mode) ||
        (control_mode == full_auto_control && remote_mode == chassis_mode))
    {
        chassis->param.Accel = ACCEL;
        cmd_out->vw_cmd = LR_TRANSLATION;
        cmd_out->vy_cmd = FB_TRANSLATION;
        cmd_out->vx_cmd = ROTATION;
    }

    if (control_mode == full_auto_control)
    {
        if ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) != 0U)
        {
            cmd_out->vx_cmd = process_flow_chassis_override.vx;
        }
        if ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VY) != 0U)
        {
            cmd_out->vy_cmd = process_flow_chassis_override.vy;
        }
        if ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VW) != 0U)
        {
            cmd_out->vw_cmd = process_flow_chassis_override.vw;
        }
    }
}

void ChassisControl_RunPipeline(Chassis_Module *chassis, const ChassisControlCmd *cmd_in, const ChassisControlFeedback *fb)
{
    float vx = 0.0f;
    float vy = 0.0f;
    float vw = 0.0f;
    float heading_hold_comp = 0.0f;
    float transient_comp = 0.0f;
    float odom_vy_comp = 0.0f;
    float odom_vw_comp = 0.0f;

    if (chassis == 0 || cmd_in == 0 || fb == 0) return;

    vx = cmd_in->vx_cmd;
    vy = cmd_in->vy_cmd;
    vw = cmd_in->vw_cmd;

    g_chassis_dbg.rotation_cmd_raw = cmd_in->vx_cmd;
    g_chassis_dbg.vx_in_raw = vx;
    g_chassis_dbg.vy_in_raw = vy;
    g_chassis_dbg.vw_in_raw = vw;
    g_chassis_dbg.yaw_body_deg = fb->yaw_body_deg;

    ChassisDecouple_Apply(vx, &vy, &vw);
    g_chassis_dbg.vy_after_decouple = vy;
    g_chassis_dbg.vw_after_decouple = vw;

    ChassisOdomDriftComp_Update(fb->yaw_body_deg, vx, vy, vw, &odom_vy_comp, &odom_vw_comp);
    vy += odom_vy_comp;
    vw += odom_vw_comp;
    g_chassis_dbg.odom_vy_comp = odom_vy_comp;
    g_chassis_dbg.odom_vw_comp = odom_vw_comp;

    heading_hold_comp = ChassisHeadingHold_TranslationHoldStep((ChassisHeadingHold *)&g_heading_hold,
                                                               fb->yaw_body_deg,
                                                               vx,
                                                               vy,
                                                               vw);
    vx += heading_hold_comp;
    g_chassis_dbg.heading_hold_vx_comp = heading_hold_comp;

    transient_comp = ChassisTransientComp_Update(vx, vy, vw);
    vx += transient_comp;
    g_chassis_dbg.transient_vx_comp = transient_comp;

    vy = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vy_limiter, vy);
    vw = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vw_limiter, vw);
    vx = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vx_limiter, vx);
    g_chassis_dbg.vx_after_limit = vx;
    g_chassis_dbg.vy_after_limit = vy;
    g_chassis_dbg.vw_after_limit = vw;

    chassis->param.Vx_in = vx;
    chassis->param.Vy_in = vy;
    chassis->param.Vw_in = vw;

    chassis->param.V_out[0] = vx + vy + vw;
    chassis->param.V_out[1] = vx - vy + vw;
    chassis->param.V_out[2] = vx + vy - vw;
    chassis->param.V_out[3] = vx - vy - vw;
    g_chassis_dbg.v_out0 = chassis->param.V_out[0];
    g_chassis_dbg.v_out1 = chassis->param.V_out[1];
    g_chassis_dbg.v_out2 = chassis->param.V_out[2];
    g_chassis_dbg.v_out3 = chassis->param.V_out[3];
}

void Chassis_Calc(Chassis_Module *chassis)
{
    ChassisControlCmd cmd = {0.0f, 0.0f, 0.0f};
    ChassisControlFeedback fb = {0.0f};

    chassis_control_resolve_cmd(chassis, &cmd);
    fb.yaw_body_deg = g_sensor_task_data.imu.yaw_deg;
    ChassisControl_RunPipeline(chassis, &cmd, &fb);

    chassis_motor1.PID_Calculate(&chassis_motor1, 50*Chassis.param.V_out[0]);
	chassis_motor2.PID_Calculate(&chassis_motor2, 50*Chassis.param.V_out[1]);
	chassis_motor3.PID_Calculate(&chassis_motor3, 50*Chassis.param.V_out[2]);
	chassis_motor4.PID_Calculate(&chassis_motor4, 50*Chassis.param.V_out[3]);

	guide_motor1.PID_Calculate(&guide_motor1, 200*Chassis.param.V_out[0]);
	guide_motor2.PID_Calculate(&guide_motor2, 200*Chassis.param.V_out[1]);
}

void Chassis_Can2_PublishGuide(void)
{
    DJIset_motor_data(&hfdcan2, 0X200,
                      guide_motor1.pid_spd.Output,
                      guide_motor2.pid_spd.Output,
                      0.0f,
                      0.0f);
}

void Chassis_Can2_PublishGuideZero(void)
{
    DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
}

void Chassis_EmergencyBrakeRun(Chassis_Module *chassis)
{
    if (chassis == 0)
    {
        return;
    }

    Chassis_Calc(chassis);

    DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,
                      chassis_motor3.pid_spd.Output, chassis_motor4.pid_spd.Output);
}

void Chassis_Stop(Chassis_Module *chassis)
{
    chassis->param.Vx_in = 0.0f;
    chassis->param.Vy_in = 0.0f;
    chassis->param.Vw_in = 0.0f;
    chassis->param.V_out[0] = 0.0f;
    chassis->param.V_out[1] = 0.0f;
    chassis->param.V_out[2] = 0.0f;
    chassis->param.V_out[3] = 0.0f;

    chassis_motor1.pid_spd.Output = 0.0f;
    chassis_motor2.pid_spd.Output = 0.0f;
    chassis_motor3.pid_spd.Output = 0.0f;
    chassis_motor4.pid_spd.Output = 0.0f;

    guide_motor1.pid_spd.Output = 0.0f;
    guide_motor2.pid_spd.Output = 0.0f;
}

static volatile uint8_t s_em_brake_active = 0U;
static volatile chassis_pid_tune_t s_pid_backup;

volatile motor_pid_tune_t s_em_tune = {
    15.0f, 0.0f, 0.15f, 0, 500.0f, 10000.0f
};

void Chassis_EmergencyBrake_Engage(void)
{
    if (!s_em_brake_active)
    {
        s_pid_backup = g_chassis_pid;
        s_em_brake_active = 1U;
    }
    g_chassis_pid.m1 = s_em_tune;
    g_chassis_pid.m2 = s_em_tune;
    g_chassis_pid.m3 = s_em_tune;
    g_chassis_pid.m4 = s_em_tune;
}

void Chassis_EmergencyBrake_DisengageIfStopped(void)
{
    if (!s_em_brake_active) return;
    if (fabsf(g_chassis_speed.m1) > 5.0f || fabsf(g_chassis_speed.m2) > 5.0f ||
        fabsf(g_chassis_speed.m3) > 5.0f || fabsf(g_chassis_speed.m4) > 5.0f)
        return;
    g_chassis_pid = s_pid_backup;
    s_em_brake_active = 0U;
}

void Chassis_EmergencyBrake_Disengage(void)
{
    if (!s_em_brake_active) return;
    g_chassis_pid = s_pid_backup;
    s_em_brake_active = 0U;
}

volatile chassis_pid_tune_t g_chassis_pid = {
    .m1 = {6.0f, 0.1f, 0.35f, 1, 500.0f, 10000.0f},
    .m2 = {6.0f, 0.1f, 0.35f, 1, 500.0f, 10000.0f},
    .m3 = {6.0f, 0.1f, 0.35f, 1, 500.0f, 10000.0f},
    .m4 = {6.0f, 0.1f, 0.35f, 1, 500.0f, 10000.0f},
};

volatile float guide_motor1_pid_param[PID_PARAMETER_NUM] = {3.0f,0.1f,0.2f,1,500.0f,10000.0f};
volatile float guide_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

/**
 * 底盘统一输出：ServiceTick -> 锁死或正常二选一 -> CAN1（每周期只发一次）
 * 锁死与 Chassis_Calc 互斥；上升/下降沿 Reset/OnActivate，详见 chassis_lock_hold.h。
 */
static void chassis_run_auto_output(void)
{
    static uint8_t s_lock_active = 0U;
    uint8_t lock_hold;

    Chassis_ServiceTick();
    lock_hold = ChassisLockHold_ShouldRun();

    if ((lock_hold != 0U) && (s_lock_active == 0U))
    {
        ChassisLockHold_OnActivate();
    }
    else if ((s_lock_active != 0U) && (lock_hold == 0U))
    {
        ChassisLockHold_Reset();
    }
    s_lock_active = lock_hold;

    if (lock_hold != 0U)
    {
        ChassisLockHold_Run();
        return;
    }

    Chassis.Chassis_Calc(&Chassis);
    DJIset_motor_data(&hfdcan1, 0X200,
                      chassis_motor1.pid_spd.Output,
                      chassis_motor2.pid_spd.Output,
                      chassis_motor3.pid_spd.Output,
                      chassis_motor4.pid_spd.Output);
    Chassis_Can2_PublishGuide();
}

/** Can_Task(full_auto) 与遥控 chassis_mode 共用入口 */
void manual_chassis_function(void)
{
    chassis_run_auto_output();
}

/** odom 导航 tick + 航向控制；锁死时仍调用，但不走 Chassis_Calc */
/* 每个tick把volatile数组刷进PID结构体，让Keil Watch可实时调参 */
/* 刷新一个电机的PID参数: volatile数组 -> PID结构体 */
static void chassis_motor_pid_refresh_one(DJI_MotorModule *m, volatile motor_pid_tune_t *p)
{
    m->pid_spd.param.kp = p->kp;
    m->pid_spd.param.ki = p->ki;
    m->pid_spd.param.kd = p->kd;
    m->pid_spd.param.Deadband = p->Deadband;
    m->pid_spd.param.limitIntegral = p->limitIntegral;
    m->pid_spd.param.limitOutput = p->limitOutput;
}

static void chassis_motor_pid_refresh(void)
{
    chassis_motor_pid_refresh_one(&chassis_motor1, (volatile motor_pid_tune_t *)&g_chassis_pid.m1);
    chassis_motor_pid_refresh_one(&chassis_motor2, (volatile motor_pid_tune_t *)&g_chassis_pid.m2);
    chassis_motor_pid_refresh_one(&chassis_motor3, (volatile motor_pid_tune_t *)&g_chassis_pid.m3);
    chassis_motor_pid_refresh_one(&chassis_motor4, (volatile motor_pid_tune_t *)&g_chassis_pid.m4);
    chassis_motor_pid_refresh_one(&guide_motor1, (volatile motor_pid_tune_t *)guide_motor1_pid_param);
    chassis_motor_pid_refresh_one(&guide_motor2, (volatile motor_pid_tune_t *)guide_motor2_pid_param);
}

void Chassis_ServiceTick(void)
{
#if ODOM_NAV_GOTO_DINGDIAN_DEBUG
    nav_goto_dingdian_debug_poll();
#elif ODOM_NAV_GOTO_WATCH_DEBUG
    odom_nav_goto_poll_debug();
#endif

    chassis_motor_pid_refresh();
    g_chassis_speed.m1 = (float)chassis_motor1.speed_rpm;
    g_chassis_speed.m2 = (float)chassis_motor2.speed_rpm;
    g_chassis_speed.m3 = (float)chassis_motor3.speed_rpm;
    g_chassis_speed.m4 = (float)chassis_motor4.speed_rpm;
    odom_nav_goto_service_tick();
    YawHeadingCtrl_Run();
}
