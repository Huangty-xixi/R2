#include "chassis.h"
#include "Motion_Task.h"
#include "lift.h"
#include "master_control.h"
#include "Sensor_Task.h"
#include "chassis_heading_hold.h"
#include "Process_Flow.h"
#include <math.h>

Chassis_Module Chassis;


// 底盘电机
DJI_MotorModule chassis_motor1;  // 左前
DJI_MotorModule chassis_motor2;  // 右前
DJI_MotorModule chassis_motor3;  // 左后
DJI_MotorModule chassis_motor4;  // 右后

// 导轮电机
DJI_MotorModule guide_motor1;  // 左
DJI_MotorModule guide_motor2;  // 右
 
uint16_t switch_state;//光电开关（PE9）


volatile ChassisDebugSnapshot g_chassis_dbg = {0};

/**
  * @brief 底盘控制命令解析
  * @param chassis 底盘模块
  * @param cmd_out 输出命令
  */
static void chassis_control_resolve_cmd(Chassis_Module *chassis, ChassisControlCmd *cmd_out)
{
    if (chassis == 0 || cmd_out == 0) return;

    cmd_out->vx_cmd = chassis->param.Vx_in;
    cmd_out->vy_cmd = chassis->param.Vy_in;
    cmd_out->vw_cmd = chassis->param.Vw_in;

    /* 遥控与半自动都允许先拿 RC 原始三轴作为底座输入 */
    if ((control_mode == remote_control && remote_mode == chassis_mode) ||
        (control_mode == semi_auto_control && remote_mode == chassis_mode))
    {
        chassis->param.Accel = ACCEL;
        cmd_out->vw_cmd = LR_TRANSLATION;
        cmd_out->vy_cmd = FB_TRANSLATION;
        cmd_out->vx_cmd = ROTATION;
    }

    /* 半自动模式可叠加按轴覆盖：包含流程控制与导航写入 */
    if (control_mode == semi_auto_control)
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
/**
  * @brief 底盘控制运行管道
  * @param chassis 底盘模块
  * @param cmd_in 输入命令
  * @param fb 反馈
  */
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

    /* 平面解耦：减少前后<->左右串扰（结构/负载变化导致） */
    ChassisDecouple_Apply(vx, &vy, &vw);
    g_chassis_dbg.vy_after_decouple = vy;
    g_chassis_dbg.vw_after_decouple = vw;

    /* 里程计交叉漂移补偿：三种输入来源共用 */
    ChassisOdomDriftComp_Update(fb->yaw_body_deg, vx, vy, vw, &odom_vy_comp, &odom_vw_comp);
    vy += odom_vy_comp;
    vw += odom_vw_comp;
    g_chassis_dbg.odom_vy_comp = odom_vy_comp;
    g_chassis_dbg.odom_vw_comp = odom_vw_comp;

    /* 平移时角度保持 */
    heading_hold_comp = ChassisHeadingHold_TranslationHoldStep((ChassisHeadingHold *)&g_heading_hold,
                                                               fb->yaw_body_deg,
                                                               vx,
                                                               vy,
                                                               vw);
    vx += heading_hold_comp;
    g_chassis_dbg.heading_hold_vx_comp = heading_hold_comp;

    /* 起步/停车瞬态补偿 */
    transient_comp = ChassisTransientComp_Update(vx, vy, vw);
    vx += transient_comp;
    g_chassis_dbg.transient_vx_comp = transient_comp;

    /* 逐轴限幅 */
    vy = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vy_limiter, vy);
    vw = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vw_limiter, vw);
    vx = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vx_limiter, vx);
    g_chassis_dbg.vx_after_limit = vx;
    g_chassis_dbg.vy_after_limit = vy;
    g_chassis_dbg.vw_after_limit = vw;

    chassis->param.Vx_in = vx;
    chassis->param.Vy_in = vy;
    chassis->param.Vw_in = vw;

    /* 输出到电机 */
    chassis->param.V_out[0] = vx + vy + vw;
    chassis->param.V_out[1] = vx - vy + vw;
    chassis->param.V_out[2] = vx + vy - vw;
    chassis->param.V_out[3] = vx - vy - vw;
    g_chassis_dbg.v_out0 = chassis->param.V_out[0];
    g_chassis_dbg.v_out1 = chassis->param.V_out[1];
    g_chassis_dbg.v_out2 = chassis->param.V_out[2];
    g_chassis_dbg.v_out3 = chassis->param.V_out[3];
}

/**
  * @brief 底盘计算
  * @param chassis 底盘模块
  */
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

	flexible_motor1.PID_Calculate(&flexible_motor1,flexible_motor_PID_input);
	flexible_motor2.PID_Calculate(&flexible_motor2,-flexible_motor_PID_input);			

}

/* 停止底盘 */
void Chassis_Stop(Chassis_Module *chassis)
{
    /* 将速度输入与输出清零 */
    chassis->param.Vx_in = 0.0f;
    chassis->param.Vy_in = 0.0f;
    chassis->param.Vw_in = 0.0f;
    chassis->param.V_out[0] = 0.0f;
    chassis->param.V_out[1] = 0.0f;
    chassis->param.V_out[2] = 0.0f;
    chassis->param.V_out[3] = 0.0f;

    /* PID输出清零，防止残留 */
    chassis_motor1.pid_spd.Output = 0.0f;
    chassis_motor2.pid_spd.Output = 0.0f;
    chassis_motor3.pid_spd.Output = 0.0f;
    chassis_motor4.pid_spd.Output = 0.0f;

    guide_motor1.pid_spd.Output = 0.0f;
    guide_motor2.pid_spd.Output = 0.0f;
}





float chassis_motor1_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.15f,1,500.0f,10000.0f};     //KP,KI,KD,DEADBAND,LIMITINTEGRAL,LIMITOUTPUT
float chassis_motor2_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor3_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor4_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.15f,1,500.0f,10000.0f};

float guide_motor1_pid_param[PID_PARAMETER_NUM] = {3.0f,0.1f,0.2f,1,500.0f,10000.0f};
float guide_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

/**
  * @brief 底盘运行逻辑
  */
void manual_chassis_function(void)
{
	//底盘运行模式下光电开关控制夹爪开合
	switch_state=HAL_GPIO_ReadPin(GPIOE ,GPIO_PIN_9); 
	if(switch_state ==1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	}
	else 
	{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	}

    if(control_mode == remote_control)
    {
        flexible_motor_update_command(RCctrl.CH5);
    }
    flexible_motor_state_machine_step();

	Chassis.Chassis_Calc(&Chassis);


	DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
	DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);
		
}


