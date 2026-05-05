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

///////////////////////////////////////////////////////////////////////


	Chassis.Chassis_Calc(&Chassis);


	DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
	DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);
		
//	DJIset_motor_data(&hfdcan1, 0X200,0,0,0,0);
//	DJIset_motor_data(&hfdcan2, 0X200,0,0,0,0);

}


void Chassis_Calc(Chassis_Module *chassis)
{
    float yaw_body_deg = 0.0f;
    float heading_hold_comp = 0.0f;
    float transient_comp = 0.0f;
		
    // 仅遥控模式从RC通道读取，master模式输入由chassis_apply_master_motion直接给定
    if ((control_mode == remote_control && remote_mode == chassis_mode) || (control_mode == semi_auto_control && remote_mode == chassis_mode)) {
        chassis->param.Accel = ACCEL;
        chassis->param.Vw_in = LR_TRANSLATION;
        chassis->param.Vy_in = FB_TRANSLATION;
        g_chassis_dbg.rotation_cmd_raw = ROTATION;
        chassis->param.Vx_in = g_chassis_dbg.rotation_cmd_raw;
    }

    if (control_mode == semi_auto_control)
    {
        if ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VX) != 0U)
        {
            chassis->param.Vx_in = process_flow_chassis_override.vx;
        }
        if ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VY) != 0U)
        {
            chassis->param.Vy_in = process_flow_chassis_override.vy;
        }
        if ((process_flow_chassis_override.axis_mask & PROCESS_FLOW_CHASSIS_OVERRIDE_VW) != 0U)
        {
            chassis->param.Vw_in = process_flow_chassis_override.vw;
        }
    }

    g_chassis_dbg.vx_in_raw = chassis->param.Vx_in;
    g_chassis_dbg.vy_in_raw = chassis->param.Vy_in;
    g_chassis_dbg.vw_in_raw = chassis->param.Vw_in;

    /* 平面解耦：减少前后<->左右串扰（结构/负载变化导致） */
    ChassisDecouple_Apply(chassis->param.Vx_in, &chassis->param.Vy_in, &chassis->param.Vw_in);
    g_chassis_dbg.vy_after_decouple = chassis->param.Vy_in;
    g_chassis_dbg.vw_after_decouple = chassis->param.Vw_in;

    /* 航向保持：逻辑在 chassis_heading_hold.c 内部实现 */   
    /* 平移时角度保持：逻辑在 chassis_heading_hold.c 内部实现 */
    yaw_body_deg = g_sensor_task_data.imu.yaw_deg;
    g_chassis_dbg.yaw_body_deg = yaw_body_deg;

    /* 平移时角度保持：逻辑在 chassis_heading_hold.c 内部实现 */
    heading_hold_comp = ChassisHeadingHold_TranslationHoldStep((ChassisHeadingHold *)&g_heading_hold,
                                                               yaw_body_deg,
                                                               chassis->param.Vx_in,
                                                               chassis->param.Vy_in,
                                                               chassis->param.Vw_in);
    chassis->param.Vx_in += heading_hold_comp;
    g_chassis_dbg.heading_hold_vx_comp = heading_hold_comp;

    /* 起步/停车瞬态补偿：在短时窗口抑制惯量扰动 */
    transient_comp = ChassisTransientComp_Update(chassis->param.Vx_in,
                                                 chassis->param.Vy_in,
                                                 chassis->param.Vw_in);
    chassis->param.Vx_in += transient_comp;
    g_chassis_dbg.transient_vx_comp = transient_comp;

    /* 逐轴限幅：限制指令变化率，降低起步/变向打滑导致的漂移 */
    chassis->param.Vy_in = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vy_limiter, chassis->param.Vy_in);
    chassis->param.Vw_in = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vw_limiter, chassis->param.Vw_in);
    chassis->param.Vx_in = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vx_limiter, chassis->param.Vx_in);
    g_chassis_dbg.vx_after_limit = chassis->param.Vx_in;
    g_chassis_dbg.vy_after_limit = chassis->param.Vy_in;
    g_chassis_dbg.vw_after_limit = chassis->param.Vw_in;
    
    /* 输出到电机 */
    chassis->param.V_out[0] = chassis->param.Vx_in + chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[1] = chassis->param.Vx_in - chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[2] = chassis->param.Vx_in + chassis->param.Vy_in - chassis->param.Vw_in;
    chassis->param.V_out[3] = chassis->param.Vx_in - chassis->param.Vy_in - chassis->param.Vw_in;
    g_chassis_dbg.v_out0 = chassis->param.V_out[0];
    g_chassis_dbg.v_out1 = chassis->param.V_out[1];
    g_chassis_dbg.v_out2 = chassis->param.V_out[2];
    g_chassis_dbg.v_out3 = chassis->param.V_out[3];

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


