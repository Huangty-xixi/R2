#include "lift.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>
#include "Motion_Task.h"

//抬升
Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;//（左）
DM_MotorModule R2_lift_motor_right;//（右）

//收缩
DJI_MotorModule flexible_motor1;//（左）
DJI_MotorModule flexible_motor2;//（右）

static uint8_t lift_has_stopped = 0;   // 1=已触限位停机
static uint8_t lift_running = 0;
static int    lift_stop_mode  = 0;     // 记录是上升停还是下降停，用于给刹车力矩
static float flexible_motor_PID_input;


float flexible_motor1_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};
float flexible_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

void R2_lift()
{
	// ==================== 升降电机防掉负载修复 ====================
	static int last_mode = -1;

	// 模式切换 → 复位所有状态
	if(r2_lift_mode != last_mode)
	{
		last_mode = r2_lift_mode;
		lift_has_stopped = 0;
		lift_running = 0;
	}

	// 已经触底/触顶停止 → 输出刹车力矩，不掉落
	if(lift_has_stopped)
	{
		flexible_motor_PID_input = 0.0f;//flexible_motor放松

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

		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0,  2.0f, 0, 0.15f,  2.0f);
		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, -2.5f, 0, 0.15f, -2.3f);

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


/**
  * @brief 抬升运行逻辑
  */
void manual_lift_function(void)
{
	//空函数
}
