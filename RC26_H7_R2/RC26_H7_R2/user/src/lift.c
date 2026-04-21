#include "lift.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h>
#include "Motion_Task.h"
#include "dm_motor.h"
#include "master_control.h"
#include "chassis.h"


//抬升
Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;//（左）
DM_MotorModule R2_lift_motor_right;//（右）

//收缩
DJI_MotorModule flexible_motor1;//（左）
DJI_MotorModule flexible_motor2;//（右）

//抬升状态机状态
static uint8_t lift_has_stopped = 0;   // 1=已触限位停机
static uint8_t lift_running = 0;
int    lift_stop_mode  = 0;     // 记录是上升停还是下降停，用于给刹车力矩
uint8_t lift_fall_fast = 0;
uint8_t lift_rise_fast = 0;


//活动电机状态
FlexibleMotorCmd flex_cmd = FLEX_CMD_NONE;               // 输入层生成的本周期命令
FlexibleMotorState4 flex_state4 = FLEX_ST_RETRACTED;     // 四状态状态机当前状态
uint16_t flex_input_prev = CH2_MID;                      // 上一周期输入值(用于边沿触发)
uint8_t flex_seen_move = 0;                              // 本次动作是否已经检测到“确实转动”
uint8_t flex_stop_cnt = 0;                               // 低速连续计数(判定到尽头)
float flexible_motor_PID_input;                         //活动电机伸缩PID输入



float flexible_motor1_pid_param[PID_PARAMETER_NUM] = {5.0f,0.4f,0.2f,1,500.0f,10000.0f};
float flexible_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.4f,0.2f,1,500.0f,10000.0f};

/* 快速上升（CH4最大值）在线调参变量 */
volatile float lift_rise_fast_left_v  = 3.0f;
volatile float lift_rise_fast_kp = 0.15f;
volatile float lift_rise_fast_kd = 0.15f;
volatile float lift_rise_fast_left_t  = 3.6f;

volatile float lift_rise_fast_right_v  = -3.4f;
volatile float lift_rise_fast_right_t  = -3.9f;

void lift_init()
{
    // 初始化默认状态：下降 fall
    r2_lift_mode = fall;



    // 复位限位相关状态
    lift_has_stopped = 0;
    lift_running    = 0;
    lift_stop_mode  = 0;
    lift_fall_fast  = 0;
    lift_rise_fast  = 0;


	// flexible_motor 新状态机上电初值
	flex_cmd = FLEX_CMD_NONE;
	flex_state4 = FLEX_ST_RETRACTED;
	flex_input_prev = CH2_MID;
	flex_seen_move = 0;
	flex_stop_cnt = 0;
	flexible_motor_PID_input = 0.0f;

}

void manual_lift_function(void)
{
	/* 遥控单模式下保持原行为；主控并行模式下不要抢停底盘 */
	if (control_mode == remote_control)
	{
		Chassis.Chassis_Stop(&Chassis);
		DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
	}
	
	static MasterLevelGate master_lift_flex_gate = {0U, 0U};
	static MasterLevelGate master_lift_updown_gate = {0U, 0U};

	if (control_mode == master_control)
	{
		uint8_t flex_level = ((master_lift_action_bits & MASTER_LIFT_FLEX_BIT) != 0U) ? 1U : 0U;
		uint8_t updown_level = ((master_lift_action_bits & MASTER_LIFT_UPDOWN_BIT) != 0U) ? 1U : 0U;
		uint8_t fall_fast_level = ((master_lift_action_bits & MASTER_LIFT_FALL_FAST_BIT) != 0U) ? 1U : 0U;
		uint8_t rise_fast_level = ((master_lift_action_bits & MASTER_LIFT_RISE_FAST_BIT) != 0U) ? 1U : 0U;

		/* master模式：
		 * bit0 抬升方向：1上升，0下降（按电平变化一次触发）
		 * bit1 快速下降：电平为1且当前为下降指令时置 lift_fall_fast（与遥控 CH4 按住一致）
		 * bit2 快速上升：电平为1且当前为上升指令时置 lift_rise_fast（与遥控 CH4 按住一致）
		 * bit3 伸缩方向：1伸出，0收回
		 */
		/* 抬升方向同样按“电平变化一次触发”处理 */
		if (master_level_gate_on_change(&master_lift_updown_gate, updown_level) != 0U)
		{
			r2_lift_mode = updown_level ? raise : fall;
		}

		/* 下降 + 快速位：与遥控分支里每周期写 lift_fall_fast=1 等效 */
		if (r2_lift_mode == fall && fall_fast_level != 0U)
		{
			lift_fall_fast = 1U;
		}
		if (r2_lift_mode == raise && rise_fast_level != 0U)
		{
			lift_rise_fast = 1U;
		}

		/* 即使持续发同一电平，也只在电平变化时触发一次伸缩命令 */
		if (master_level_gate_on_change(&master_lift_flex_gate, flex_level) != 0U)
		{
			/* master模式下直接按bit语义下发命令，避免CH2值映射带来的歧义 */
			flex_cmd = flex_level ? FLEX_CMD_EXTEND : FLEX_CMD_RETRACT;
		}
		else
		{
			/* 无新边沿时不重复触发 */
			flex_cmd = FLEX_CMD_NONE;
		}
	}

	 
	else if(control_mode == remote_control)
	{
		/* 遥控模式下将门控状态与“真实机构状态”对齐，避免下次切回master误触发
		 * 注意：到位后摇杆/指令可能回中位，但机构状态不会自动反向，因此不能用通道值做对齐依据
		 */
		{
			uint8_t flex_real_level = 0U;
			if (flex_state4 == FLEX_ST_EXTENDED) flex_real_level = 1U;
			else if (flex_state4 == FLEX_ST_RETRACTED) flex_real_level = 0U;
			/* 其他运动中状态保持默认即可 */
			master_level_gate_init(&master_lift_flex_gate, flex_real_level);
		}
		{
			uint8_t updown_real_level;
			/* 优先按“真实到位状态”对齐：
			 * - 已到位且 stop_mode=raise: 认为当前在上升侧
			 * - 已到位且 stop_mode=fall : 认为当前在下降侧
			 * 未到位时再回退到当前指令状态 r2_lift_mode
			 */
			if (lift_has_stopped != 0U)
			{
				updown_real_level = (lift_stop_mode == raise) ? 1U : 0U;
			}
			else
			{
				updown_real_level = (r2_lift_mode == raise) ? 1U : 0U;
			}
			master_level_gate_init(&master_lift_updown_gate, updown_real_level);
		}

		if(RCctrl.CH3==1792)
		r2_lift_mode = raise;  // 上升
		else if(RCctrl.CH3==192)
		r2_lift_mode = fall;   // 正常
		else if(RCctrl.CH4==192)
		{
			r2_lift_mode = fall;   // 正常
			lift_fall_fast = 1;
		}
		else if(RCctrl.CH4==1792)
		{
			r2_lift_mode = raise;
			lift_rise_fast = 1;
		}

		//控制flexible_motor伸缩
		flexible_motor_update_command(RCctrl.CH2);
	}


	flexible_motor_state_machine_step();

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
		lift_fall_fast = 0;
		lift_rise_fast = 0;
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
				R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f, 1.9f);
				R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f,  -2.8f);
		}
	}

	// 正常运行
	else if(r2_lift_mode == fall)
	{
		if (lift_fall_fast == 0)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -1.0f, 0, 0.30f, -1.1f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 1.0f, 0, 0.30f,  1.1f);
		}
		else if (lift_fall_fast != 0)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -2.0f, 0, 0.30f, -3.1f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 2.0f, 0, 0.30f, 3.1f);
		}


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
				lift_fall_fast = 0;
		}
	}
	else if(r2_lift_mode == raise)
	{
		if (lift_rise_fast == 0U)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0,  2.2f, 0, 0.15f,  3.6f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, -2.7f, 0, 0.15f, -3.9f);
		}
		else
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, lift_rise_fast_left_v, lift_rise_fast_kp, lift_rise_fast_kd, lift_rise_fast_left_t);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, lift_rise_fast_right_v, lift_rise_fast_kp, lift_rise_fast_kd, lift_rise_fast_right_t);
		}

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
				lift_rise_fast = 0;
		}
	}
}



void flexible_motor_update_command(uint16_t ch_value)
{
	// 每周期先清空命令，只有检测到边沿才产生一次命令
	flex_cmd = FLEX_CMD_NONE;

	// 高位边沿：发起“伸出”命令（保持高位不重复触发）
	if (ch_value == CH2_HIGH && flex_input_prev != CH2_HIGH)
	{
		flex_cmd = FLEX_CMD_RETRACT;
	}
	// 低位边沿：发起“收回”命令（保持低位不重复触发）
	else if (ch_value == CH2_LOW && flex_input_prev != CH2_LOW)
	{
		flex_cmd = FLEX_CMD_EXTEND;
	}

	// 记录当前输入用于下一周期的边沿比较
	flex_input_prev = ch_value;
}

void flexible_motor_state_machine_step(void)
{
	// 读取双电机当前速度（取绝对值用于“是否在动/是否停止”判定）
	int rpm1 = abs((int)flexible_motor1.speed_rpm);
	int rpm2 = abs((int)flexible_motor2.speed_rpm);

	// 命令层 -> 状态层：收到一次命令就进入对应“运动中”状态，并清零本次动作判定
	if (flex_cmd == FLEX_CMD_EXTEND)
	{
		flex_state4 = FLEX_ST_EXTENDING;
		flex_seen_move = 0;
		flex_stop_cnt = 0;
	}
	else if (flex_cmd == FLEX_CMD_RETRACT)
	{
		flex_state4 = FLEX_ST_RETRACTING;
		flex_seen_move = 0;
		flex_stop_cnt = 0;
	}

	switch (flex_state4)
	{
	case FLEX_ST_EXTENDING:
		// 伸出中：持续给伸出驱动
		flexible_motor_PID_input = FLEX_CMD_EXTEND_PWM;
		if (rpm1 > FLEX_RUN_THR_RPM || rpm2 > FLEX_RUN_THR_RPM)
			flex_seen_move = 1;

		// 先“动过”再判“连续低速”到尽头，避免刚切换时误判
		if (flex_seen_move && rpm1 < FLEX_STOP_THR_RPM && rpm2 < FLEX_STOP_THR_RPM)
		{
			if (++flex_stop_cnt >= FLEX_STOP_CNT_MAX)
			{
				// 伸到尽头：切稳态并清驱动
				flexible_motor_PID_input = 0.0f;
				flex_state4 = FLEX_ST_EXTENDED;
				flex_stop_cnt = 0;
			}
		}
		else
		{
			flex_stop_cnt = 0;
		}
		break;

	case FLEX_ST_RETRACTING:
		// 收回中：持续给收回驱动
		flexible_motor_PID_input = FLEX_CMD_RETRACT_PWM;
		if (rpm1 > FLEX_RUN_THR_RPM || rpm2 > FLEX_RUN_THR_RPM)
			flex_seen_move = 1;

		// 先“动过”再判“连续低速”到尽头，避免刚切换时误判
		if (flex_seen_move && rpm1 < FLEX_STOP_THR_RPM && rpm2 < FLEX_STOP_THR_RPM)
		{
			if (++flex_stop_cnt >= FLEX_STOP_CNT_MAX)
			{
				// 收到尽头：切稳态并清驱动
				flexible_motor_PID_input = 0.0f;
				flex_state4 = FLEX_ST_RETRACTED;
				flex_stop_cnt = 0;
			}
		}
		else
		{
			flex_stop_cnt = 0;
		}
		break;

	case FLEX_ST_EXTENDED:
		// 伸到尽头稳态：不再主动驱动
		flexible_motor_PID_input = 0.0f;
		break;

	case FLEX_ST_RETRACTED:
		// 收到尽头稳态：抬升上升阶段仍保持收回驱动
		if (r2_lift_mode == raise)
			flexible_motor_PID_input = 500.0f;
		else
			flexible_motor_PID_input = 0.0f;
		break;

	default:
		flexible_motor_PID_input = 0.0f;
		break;
	}
}