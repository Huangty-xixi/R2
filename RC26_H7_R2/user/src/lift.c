#include "lift.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h>
#include "Motion_Task.h"
#include "dm_motor.h"


//抬升
Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;//（左）
DM_MotorModule R2_lift_motor_right;//（右）
Flexible_motor_state flexible_motor_state;

//收缩
DJI_MotorModule flexible_motor1;//（左）
DJI_MotorModule flexible_motor2;//（右）

//抬升状态机状态
static uint8_t lift_has_stopped = 0;   // 1=已触限位停机
static uint8_t lift_running = 0;
int    lift_stop_mode  = 0;     // 记录是上升停还是下降停，用于给刹车力矩

//活动电机伸缩状态机状态
uint8_t flexible_motor_has_stopped = 0;
uint8_t flexible_motor_running = 0;
int    flexible_motor_stop_mode  = 0;  
int    last_flexible_motor_state = -1;
float flexible_motor_PID_input;


float flexible_motor1_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};
float flexible_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

void lift_init()
{
    // 初始化默认状态：下降 fall
    r2_lift_mode = fall;
	flexible_motor_state = retraction;

    // 复位限位相关状态
    lift_has_stopped = 0;
    lift_running    = 0;
    lift_stop_mode  = 0;

	flexible_motor_has_stopped = 0;
	flexible_motor_running = 0;
	flexible_motor_stop_mode = 0;

}

void manual_lift_function(void)
{
	
	if(RCctrl.CH3==1792)
	r2_lift_mode = raise;  // 上升
	else if(RCctrl.CH3==192)
	r2_lift_mode = fall;   // 正常

	if(RCctrl.CH2==1792)
	flexible_motor_state = stretch;  //伸出
	else if(RCctrl.CH2==192)
	flexible_motor_state = retraction;   // 收回


	flexible_motor_use();

	flexible_motor1.PID_Calculate(&flexible_motor1,flexible_motor_PID_input);
	flexible_motor2.PID_Calculate(&flexible_motor2,-flexible_motor_PID_input);

	
	DJIset_motor_data(&hfdcan2, 0X200, 0.0f,0.0f,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);
				
				
	// ==================== 升降电机防掉负载修复 ====================
	static int last_r2_lift_mode = -1;

	// 模式切换 → 复位所有状态
	if(r2_lift_mode != last_r2_lift_mode)
	{
		last_r2_lift_mode = r2_lift_mode;
		lift_has_stopped = 0;
		lift_running = 0;
	}
	// 已经触底/触顶停止 → 输出刹车力矩，不掉落
	  if(lift_has_stopped)
	{
		
		if(lift_stop_mode == fall)
		{
			// 上升到顶：给微小向下力矩顶住不下滑
				R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f,  -0.5f);
				R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f, 0.8f);
		}
		else if(lift_stop_mode == raise)
		{
				 // 下降到底：给一个微小向上力矩顶住不回落
				R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f, 1.6f);
				R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f,  -2.5f);
		}
	}

	// 正常运行
	if(r2_lift_mode == fall)
	{
		flexible_motor_PID_input = 500.0f;//flexible_motor顶死

		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -2.0f, 0, 0.15f, -1.0f);
		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 2.0f, 0, 0.15f,  1.0f);

		if(fabsf(R2_lift_motor_left.speed_w) > 1.5f || fabsf(R2_lift_motor_right.speed_w) > 1.5f)
		{
				lift_running = 1;
		}

		// 触底停止
		if(lift_running && 
			 fabsf(R2_lift_motor_left.speed_w) < 0.5f && fabsf(R2_lift_motor_right.speed_w) < 0.5f)
		{
				lift_has_stopped = 1;
				lift_stop_mode = fall;  // 记录停止模式
		}
	}
	else if(r2_lift_mode == raise)
	{
		flexible_motor_PID_input = 500.0f;//flexible_motor顶死

		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0,  2.0f, 0, 0.15f,  3.0f);
		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, -2.5f, 0, 0.15f, -3.3f);

		if(fabsf(R2_lift_motor_left.speed_w) > 1.5f || fabsf(R2_lift_motor_right.speed_w) > 1.5f)
		{
				lift_running = 1;
		}

		// 触顶停止
		if(lift_running && 
			 fabsf(R2_lift_motor_left.speed_w) < 0.5f && fabsf(R2_lift_motor_right.speed_w) < 0.5f)
		{
				lift_has_stopped = 1;
				lift_stop_mode = raise; // 记录停止模式
		}
	}
}


void flexible_motor_use()
{
	// 模式切换 → 复位所有状态
	if(flexible_motor_state != last_flexible_motor_state)
	{
		last_flexible_motor_state = flexible_motor_state;
		flexible_motor_has_stopped = 0;
		flexible_motor_running = 0;
	}

	if(flexible_motor_state == stretch)
	{
		flexible_motor_PID_input = -500.0f;
			if(abs((int)flexible_motor1.speed_rpm) > 1 || abs((int)flexible_motor2.speed_rpm) > 1)
		{
				flexible_motor_running = 1;
		}

		// 触底停止
		if(flexible_motor_running && 
			abs((int)flexible_motor1.speed_rpm) < 1 && abs((int)flexible_motor2.speed_rpm) < 1)
		{
				flexible_motor_has_stopped = 1;
				flexible_motor_stop_mode = stretch;  // 记录停止模式
		}

	}

	else if(flexible_motor_state == retraction)
	{
		flexible_motor_PID_input = 500.0f;
			if(abs((int)flexible_motor1.speed_rpm) > 1 || abs((int)flexible_motor2.speed_rpm) > 1)
		{
				flexible_motor_running = 1;
		}

		// 触底停止
		if(flexible_motor_running && 
			abs((int)flexible_motor1.speed_rpm) < 1 && abs((int)flexible_motor2.speed_rpm) < 1)
		{
				flexible_motor_has_stopped = 1;
				flexible_motor_stop_mode = retraction;  // 记录停止模式
		}

	}


	if(lift_stop_mode == fall && flexible_motor_has_stopped && flexible_motor_running ==0)
	{
		flexible_motor_PID_input = 0.0f;//flexible_motor放松
	}

}