#include "chassis.h"
#include "Motion_Task.h"

Chassis_Module Chassis;

//底盘
DJI_MotorModule chassis_motor1;  // （左前）
DJI_MotorModule chassis_motor2;  // （右前）
DJI_MotorModule chassis_motor3;  // （左后）
DJI_MotorModule chassis_motor4;  // （右后）

//导轮
DJI_MotorModule guide_motor1;  // （左）
DJI_MotorModule guide_motor2;  // （右）
DJI_MotorModule flexible_motor1;//（左）
DJI_MotorModule flexible_motor2;//（右）

//抬升
Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;//（左）
DM_MotorModule R2_lift_motor_right;//（右）

static uint8_t lift_has_stopped = 0;   // 1=已触限位停机
static uint8_t lift_running = 0;
static int    lift_stop_mode  = 0;     // 记录是上升停还是下降停，用于给刹车力矩
static float flexible_motor_PID_input;

float chassis_motor1_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.15f,1,500.0f,10000.0f};     //KP,KI,KD,DEADBAND,LIMITINTEGRAL,LIMITOUTPUT
float chassis_motor2_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor3_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor4_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.15f,1,500.0f,10000.0f};

float guide_motor1_pid_param[PID_PARAMETER_NUM] = {3.0f,0.1f,0.2f,1,500.0f,10000.0f};
float guide_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

float flexible_motor1_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};
float flexible_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};



/**
  * @brief 底盘运行逻辑
  */
void manual_chassis_function(void)
{
	//空函数
		DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output,
                      chassis_motor2.pid_spd.Output,
                      chassis_motor3.pid_spd.Output,
                      chassis_motor4.pid_spd.Output);
    DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output,
                      guide_motor2.pid_spd.Output,
                      flexible_motor1.pid_spd.Output,
                      flexible_motor2.pid_spd.Output);

}





void Chassis_Calc(Chassis_Module *chassis)
{
    chassis->param.Accel = ACCEL;
    chassis->param.Vx_in =-LR_TRANSLATION;
    chassis->param.Vy_in = FB_TRANSLATION;
    chassis->param.Vw_in = ROTATION;
    
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

//void R2_lift()
//{
//	// ==================== 升降电机防掉负载修复 ====================
//	static int last_mode = -1;

//	// 模式切换 → 复位所有状态
//	if(r2_lift_mode != last_mode)
//	{
//		last_mode = r2_lift_mode;
//		lift_has_stopped = 0;
//		lift_running = 0;
//	}

//	// 已经触底/触顶停止 → 输出刹车力矩，不掉落
//	if(lift_has_stopped)
//	{
//		flexible_motor_PID_input = 0.0f;//flexible_motor放松

//		if(lift_stop_mode == fall)
//		{
//				// 上升到顶：给微小向下力矩顶住不下滑
//				R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f,  -0.5f);
//				R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f, 0.8f);
//		}
//		else if(lift_stop_mode == raise)
//		{
//				 // 下降到底：给一个微小向上力矩顶住不回落
//				R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f, 1.6f);
//				R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f,  -2.5f);
//		}
//	}

//	// 正常运行
//	if(r2_lift_mode == fall)
//	{
//		flexible_motor_PID_input = 500.0f;//flexible_motor顶死

//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -2.0f, 0, 0.15f, -1.0f);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 2.0f, 0, 0.15f,  1.0f);

//		if(fabsf(R2_lift_motor_left.speed_w) > 1.5f || 
//			 fabsf(R2_lift_motor_right.speed_w) > 1.5f)
//		{
//				lift_running = 1;
//		}

//		// 触底停止
//		if(lift_running && 
//			 fabsf(R2_lift_motor_left.speed_w) < 0.5f && 
//			 fabsf(R2_lift_motor_right.speed_w) < 0.5f)
//		{
//				lift_has_stopped = 1;
//				lift_stop_mode = fall;  // 记录停止模式
//		}
//	}
//	else if(r2_lift_mode == raise)
//	{
//		flexible_motor_PID_input = 500.0f;//flexible_motor顶死

//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0,  2.0f, 0, 0.15f,  2.0f);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, -2.5f, 0, 0.15f, -2.3f);

//		if(fabsf(R2_lift_motor_left.speed_w) > 1.5f || 
//			 fabsf(R2_lift_motor_right.speed_w) > 1.5f)
//		{
//				lift_running = 1;
//		}

//		// 触顶停止
//		if(lift_running && 
//			 fabsf(R2_lift_motor_left.speed_w) < 0.5f && 
//			 fabsf(R2_lift_motor_right.speed_w) < 0.5f)
//		{
//				lift_has_stopped = 1;
//				lift_stop_mode = raise; // 记录停止模式
//		}
//	}


//}


////抬升
//float Initpos[2] = {0};
//void Initpos_Get(void)
//{
//    float pre_pos[2];
//    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 5, 0, 0.3, 0);
//    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, 5, 0, 0.3, 0);
//    pre_pos[0] = R2_lift_motor_left.position;
//    pre_pos[1] = R2_lift_motor_right.position;
//    vTaskDelay(500);
//    while(((R2_lift_motor_left.position - pre_pos[0]) >= 0.05) || ((R2_lift_motor_right.position - pre_pos[1]) >= 0.05)){
//        R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 5, 0, 0.3, 0);
//        R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, 5, 0, 0.3, 0);
//        pre_pos[0] = R2_lift_motor_left.position;
//        pre_pos[1] = R2_lift_motor_right.position;
//        vTaskDelay(100);
//    }
//    Initpos[0] = R2_lift_motor_left.position;
//    Initpos[1] = R2_lift_motor_right.position;
//}

