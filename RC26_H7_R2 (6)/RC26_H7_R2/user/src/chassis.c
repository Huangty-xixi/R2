#include "chassis.h"
#include "Motion_Task.h"
#include "lift.h"

Chassis_Module Chassis;


//底盘
DJI_MotorModule chassis_motor1;  // （左前）
DJI_MotorModule chassis_motor2;  // （右前）
DJI_MotorModule chassis_motor3;  // （左后）
DJI_MotorModule chassis_motor4;  // （右后）

//导轮
DJI_MotorModule guide_motor1;  // （左）
DJI_MotorModule guide_motor2;  // （右）



//活动电机伸缩状态机状态
static uint8_t flexible_motor_has_stopped = 0;
static uint8_t flexible_motor_running = 0;



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
	//flexible_motor调用，通道五控制，
//		if(lift_stop_mode == raise)
//		{
//			flexible_motor_PID_input = 500.0f;
//			
//		}

	// if(RCctrl.CH5==192)//上拨
	// flexible_motor_state = stretch;  //伸出
	// else if(RCctrl.CH5==1792)//下拨
	// flexible_motor_state = retraction;   // 收回

//	flexible_motor_use();

flexible_motor_update_command(RCctrl.CH5);
flexible_motor_state_machine_step();

///////////////////////////////////////////////////////////////////////


	Chassis.Chassis_Calc(&Chassis);

	chassis_motor1.PID_Calculate(&chassis_motor1, 50*Chassis.param.V_out[0]);
	chassis_motor2.PID_Calculate(&chassis_motor2, 50*Chassis.param.V_out[1]);
	chassis_motor3.PID_Calculate(&chassis_motor3, 50*Chassis.param.V_out[2]);
	chassis_motor4.PID_Calculate(&chassis_motor4, 50*Chassis.param.V_out[3]);
	
	guide_motor1.PID_Calculate(&guide_motor1, 200*Chassis.param.V_out[0]);
	guide_motor2.PID_Calculate(&guide_motor2, 200*Chassis.param.V_out[1]);

	flexible_motor1.PID_Calculate(&flexible_motor1,flexible_motor_PID_input);
	flexible_motor2.PID_Calculate(&flexible_motor2,-flexible_motor_PID_input);			

	DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
	DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);
		
//	DJIset_motor_data(&hfdcan1, 0X200,0,0,0,0);
//	DJIset_motor_data(&hfdcan2, 0X200,0,0,0,0);

}


void Chassis_Calc(Chassis_Module *chassis)
{
    chassis->param.Accel = ACCEL;
    chassis->param.Vw_in = LR_TRANSLATION;
    chassis->param.Vy_in = FB_TRANSLATION;
    chassis->param.Vx_in = ROTATION;
    
    chassis->param.V_out[0] = chassis->param.Vx_in + chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[1] = chassis->param.Vx_in - chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[2] = chassis->param.Vx_in + chassis->param.Vy_in - chassis->param.Vw_in;
    chassis->param.V_out[3] = chassis->param.Vx_in - chassis->param.Vy_in - chassis->param.Vw_in;

}

void Chassis_Stop(Chassis_Module *chassis)
{
    chassis->param.V_out[0] = 0.f;
    chassis->param.V_out[1] = 0.f;
    chassis->param.V_out[2] = 0.f;
    chassis->param.V_out[3] = 0.f;
    
}

void chassis_use()
{
	// 底盘电机正常输出（始终运行）
	DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
	DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);

}
