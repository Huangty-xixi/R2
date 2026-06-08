#include "kfs.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include <math.h>
#include "cmsis_os.h"
#include "Motion_Task.h"
#include "chassis.h"

Kfs_Module Kfs;

DJI_MotorModule kfs_above;  
DJI_MotorModule kfs_below;  

DM_MotorModule main_lift;
DM_MotorModule kfs_spin;
DM_MotorModule three_kfs;


Three_kfs_position three_kfs_position;
Kfs_spin_position kfs_spin_position;
Main_lift_position main_lift_position;

// 上电初始位置
float main_lift_Initpos = 0.2f;
float kfs_spin_Initpos = 0.0f;
float three_kfs_Initpos = -4.055f;

float kfs_above_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,9000.0f};
float kfs_below_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,9000.0f};

/* ==================== 伸缩位置环参数（above/below 共用，Watch 可在线改） ==================== */
volatile Kfs_Flex_PosCtrl_Param kfs_below_pos_param = {
    .pos_kp = 120.0f,
    .pos_ki = 0.0f,
    .pos_kd = 400.0f,
    .max_speed = 800.0f,
    .pos_rounds = {0.0f, -60.0f, 160.0f, 240.0f},
    .pos_i_limit = 50.0f,
};

volatile Kfs_Flex_PosCtrl_Param kfs_above_pos_param = {
    .pos_kp = 120.0f,
    .pos_ki = 0.0f,
    .pos_kd = 400.0f,
    .max_speed = 800.0f,
    .pos_rounds = {0.0f, 80.0f, 160.0f, 240.0f},
    .pos_i_limit = 50.0f,
};

/* kfs_below 控制模式状态（默认速度模式） */
volatile Flexible_Mode flexible_mode = flex_below_speed;
volatile Flex_TargetPos flex_target_pos = flex_pos0;

/* 全自动模式位置指令（类似 main_lift_position，auto 代码直接设） */
volatile Kfs_Below_Cmd kfs_below_cmd = kfs_below_cmd_stop;
volatile Kfs_Above_Cmd kfs_above_cmd = kfs_above_cmd_stop;

/* main_lift 分段计时(ms)，debugger 可实时改；pX_pY = pX->pY */
volatile Main_Lift_Timing_Param main_lift_timing_param = {
    .t_up_p0_p1   = 200U,
    .t_up_p1_p2   = 450U,
    .t_up_p2_p3   = 590U,
    .t_up_p3_p4   = 735U,
    .t_down_p0_p1 = 200U,
    .t_down_p1_p2 = 450U,
    .t_down_p2_p3 = 590U,
    .t_down_p3_p4 = 735U,
};

static uint32_t main_lift_up_ms_get(int32_t lvl)
{
	switch (lvl)
	{
	case 0: return main_lift_timing_param.t_up_p0_p1;
	case 1: return main_lift_timing_param.t_up_p1_p2;
	case 2: return main_lift_timing_param.t_up_p2_p3;
	case 3: return main_lift_timing_param.t_up_p3_p4;
	default: return 0U;
	}
}

static uint32_t main_lift_down_ms_get(int32_t lvl)
{
	switch (lvl)
	{
	case 0: return main_lift_timing_param.t_down_p0_p1;
	case 1: return main_lift_timing_param.t_down_p1_p2;
	case 2: return main_lift_timing_param.t_down_p2_p3;
	case 3: return main_lift_timing_param.t_down_p3_p4;
	default: return 0U;
	}
}

/* 位置环内部状态 */
static int32_t flex_pos_base_rounds = 0;   /* 切入位置模式时的基准圈数 */
static float   flex_pos_integral  = 0.0f;  /* 位置环积分 */
static float   flex_pos_last_error = 0.0f; /* 位置环上次误差 */
static uint8_t flex_pos_inited = 0U;       /* 位置环是否已初始化 */

/* ==================== 伸缩位置环 PID（above/below 共用，在速度环之上） ==================== */
static float kfs_flex_position_pid(Kfs_Flex_PosCtrl_Param volatile *p, float target_rounds, float current_rounds)
{
    float error = target_rounds - current_rounds;
    float derivative;
    float output;

    /* 积分累加 + 限幅 */
    flex_pos_integral += error;
    if (flex_pos_integral > p->pos_i_limit)
        flex_pos_integral = p->pos_i_limit;
    if (flex_pos_integral < -p->pos_i_limit)
        flex_pos_integral = -p->pos_i_limit;

    /* 微分 */
    derivative = error - flex_pos_last_error;
    flex_pos_last_error = error;

    /* PID 输出 */
    output = p->pos_kp * error
           + p->pos_ki * flex_pos_integral
           + p->pos_kd * derivative;

    /* 输出限幅（CH2 等效值，x200 后送入速度环） */
    if (output > p->max_speed)
        output = p->max_speed;
    if (output < -p->max_speed)
        output = -p->max_speed;

    return output;
}

// 初始化：读取上电初始位置
void kfs_three_kfs_spin_main_lift_pos_init(void)
{
//	main_lift.set_mit_data(&main_lift, MAIN_LIFT_OFFSET1, 0.0f, 0.2, 0.15f, -5.0f);
 	kfs_spin.set_mit_data(&kfs_spin, kfs_spin_Initpos + KFS_SPIN_OFFSET1, 0.0f, 6.5f, 2.0f, 0.0f);
	HAL_Delay(1000);
	three_kfs.set_mit_data(&three_kfs, three_kfs_Initpos, 0.0f, 5.0f, 0.2f, 0.2f);

	three_kfs_position = three_kfs_p1;
	main_lift_position = main_lift_p1; /* 开机初始化到p1 */
	kfs_spin_position  = kfs_spin_p1;
}

/**
  * @brief KFS运行逻辑
  */
void manual_kfs_function(void)
{
	/* 遥控单模式下保持原行为；主控并行模式下不抢停底盘 */
	if (control_mode == remote_control)
	{
		Chassis.Chassis_Stop(&Chassis);
		DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
	}
	
	static Control_mode last_control_mode = remote_control;

	/* ==================== 三档旋转 ==================== */
	// 通道一控制三档旋转KFS
	static uint16_t ch1_prev = 0;
	static int8_t three_kfs_pingpong_dir = 1; /* 1: p1->p4, -1: p4->p1 */
	
	if (control_mode == remote_control)
	{
		if (RCctrl.CH1 >=1500 && ch1_prev <=1500)
		{
			if (three_kfs_position == three_kfs_p1) three_kfs_pingpong_dir = 1;
			else if (three_kfs_position == three_kfs_p4) three_kfs_pingpong_dir = -1;

			if (three_kfs_pingpong_dir > 0)
			{
				if (three_kfs_position == three_kfs_p1) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p3;
				else if (three_kfs_position == three_kfs_p3) three_kfs_position = three_kfs_p4;
				else three_kfs_position = three_kfs_p3;
			}
			else
			{
				if (three_kfs_position == three_kfs_p4) three_kfs_position = three_kfs_p3;
				else if (three_kfs_position == three_kfs_p3) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p1;
				else three_kfs_position = three_kfs_p2;
			}
		}
		if (RCctrl.CH1 <=500 && ch1_prev >=500)
		{
			if (three_kfs_position == three_kfs_p1) three_kfs_pingpong_dir = 1;
			else if (three_kfs_position == three_kfs_p4) three_kfs_pingpong_dir = -1;

			if (three_kfs_pingpong_dir > 0)
			{
				if (three_kfs_position == three_kfs_p1) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p3;
				else if (three_kfs_position == three_kfs_p3) three_kfs_position = three_kfs_p4;
				else three_kfs_position = three_kfs_p3;
			}
			else
			{
				if (three_kfs_position == three_kfs_p4) three_kfs_position = three_kfs_p3;
				else if (three_kfs_position == three_kfs_p3) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p1;
				else three_kfs_position = three_kfs_p2;
			}
		}
		ch1_prev = RCctrl.CH1;
	}
	
	
	


	float tar_3k;
	const float kp_3k = 10.0f;
	const float kd_3k = 2.0f;
	const float tar_step_max_3k = 0.009f; 
	static float tar_3k_ramped = 0.0f;
	static uint8_t tar_3k_ramped_inited = 0U;
	
	switch(three_kfs_position)
	{
		case three_kfs_p1:
			tar_3k = THREE_KFS_OFFSET1;
			three_kfs.set_mit_data(&three_kfs, tar_3k_ramped, 0.0f, kp_3k, kd_3k, 0.0f);

		break;
		case three_kfs_p2:
			tar_3k = THREE_KFS_OFFSET2;
			three_kfs.set_mit_data(&three_kfs, tar_3k_ramped, 0.0f, kp_3k, kd_3k, 0.2f);

		break;
		case three_kfs_p3: 
			tar_3k = THREE_KFS_OFFSET3;
			three_kfs.set_mit_data(&three_kfs, tar_3k_ramped, 0.0f, kp_3k, kd_3k, 0.0f);

		break;
		case three_kfs_p4:
			tar_3k = THREE_KFS_OFFSET4;
			three_kfs.set_mit_data(&three_kfs, tar_3k_ramped, 0.0f, kp_3k, kd_3k, 0.0f);

		break;
		default: tar_3k = three_kfs_Initpos;
	}

	if (tar_3k_ramped_inited == 0U)
	{
		tar_3k_ramped = three_kfs.position;
		tar_3k_ramped_inited = 1U;
	}
	{
		float delta = tar_3k - tar_3k_ramped;
		if (delta > tar_step_max_3k) delta = tar_step_max_3k;
		else if (delta < -tar_step_max_3k) delta = -tar_step_max_3k;
		tar_3k_ramped += delta;
	}
	// three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.0f, 0.0f, 0.0f);
	
	/* ==================== 主轴抬升 ==================== */
	/* --- [输入层] 遥控CH3 -> 目标档位命令 main_lift_position --- */
	static uint8_t main_lift_busy = 0U; /* 供输入层读取的主轴忙标志 */
	
		/* 遥控：CH3边沿换挡（与CH4切挡风格一致） */
		if (control_mode == remote_control)
		{
			static uint16_t ch3_prev = 0;
			static uint8_t ch3_cmd_lock = 0U; /* 1=主轴动作执行中，忽略新换挡命令 */

			ch3_prev = RCctrl.CH3;
			/* 阈值边沿：避免摇杆值没精确到192/1792时触发不到换挡 */
			{
				static uint8_t ch3_zone_prev = 1U; /* 0=LOW,1=MID,2=HIGH */
				uint8_t ch3_zone = 1U;
				if (RCctrl.CH3 >= 1500) ch3_zone = 2U;
				else if (RCctrl.CH3 <= 500) ch3_zone = 0U;

				/* 遥控：在p0~p4循环；上拨=+1(循环)，下拨=-1(循环) */
				if (ch3_zone == 2U && ch3_zone_prev != 2U && ch3_cmd_lock == 0U)
				{
					main_lift_position = (Main_lift_position)(((int)main_lift_position + 1) % 5);
				}
				if (ch3_zone == 0U && ch3_zone_prev != 0U && ch3_cmd_lock == 0U)
				{
					main_lift_position = (Main_lift_position)(((int)main_lift_position - 1 + 5) % 5);
				}
				ch3_zone_prev = ch3_zone;
			}
			ch3_cmd_lock = main_lift_busy;

		}
		/* --- [状态层] 主轴抬升状态变量（上次目标/位置估计/运动标志） --- */
		/* --- [执行层总流程] 档位变化 -> 固定速度 + 分段计时 -> 到时停止 --- */
		{
			static Main_lift_position main_lift_cmd_prev = main_lift_p0;        /* 上一次已执行的目标档位 */
			static Main_lift_position main_lift_pos_est = main_lift_p0;         /* 当前位置估计档位（计时法估计） */
			static Main_lift_position main_lift_target_active = main_lift_p0;   /* 当前正在执行的目标档位 */
			static Main_lift_position main_lift_target_pending = main_lift_p0;  /* 运动中收到的新目标（待执行） */
			static uint8_t main_lift_pending_valid = 0U;                        /* 待执行目标是否有效：1有效/0无 */
			static uint8_t lift_moving = 0U;                                    /* 计时动作状态：1运动中/0停止 */
			static int8_t lift_dir = 0; /* +1上升，-1下降 */
			static uint32_t lift_move_end_tick = 0U;                            /* 本次动作结束时刻（tick） */
			const float v_up = -5.0f;                                           /* 上升固定速度 */
			const float v_down = 5.0f;
			//p0:000 p1:001 p2:010 p3:011 p4:100

			if(control_mode == remote_control || control_mode == full_auto_control)
			{
				/* --- [调度层] 目标仲裁：运动中缓存pending，空闲时切active --- */
				/* 统一调度锁：动作执行中不立即切目标，先缓存，等当前动作结束再切换 */
				if (lift_moving != 0U)
				{
					if (main_lift_position != main_lift_target_active)
					{
						main_lift_target_pending = main_lift_position;
						main_lift_pending_valid = 1U;
					}
				}
				else
				{
					if (main_lift_pending_valid != 0U)
					{
						main_lift_target_active = main_lift_target_pending;
						main_lift_pending_valid = 0U;
					}
					else
					{
						main_lift_target_active = main_lift_position;
					}
				}

				/* --- [计时层] 新目标触发：计算时长与方向，启动一次动作 --- */
				if (main_lift_target_active != main_lift_cmd_prev)
				{
					uint32_t duration = 0U;

					if ((int32_t)main_lift_target_active > (int32_t)main_lift_pos_est)
					{
						int32_t lvl = (int32_t)main_lift_pos_est;
						while (lvl < (int32_t)main_lift_target_active)
						{
							if (lvl >= 0 && lvl <= 3) duration += main_lift_up_ms_get(lvl);
							lvl++;
						}
						lift_dir = +1;
						if (duration > 0U)
						{
							lift_moving = 1U;
							lift_move_end_tick = osKernelGetTickCount() + duration;
						}
						else
						{
							lift_moving = 0U;
						}
					}
					else if ((int32_t)main_lift_target_active < (int32_t)main_lift_pos_est)
					{
						int32_t lvl = (int32_t)main_lift_pos_est;
						while (lvl > (int32_t)main_lift_target_active)
						{
							if (lvl >= 1 && lvl <= 4) duration += main_lift_down_ms_get(lvl - 1);
							lvl--;
						}
						lift_dir = -1;
						if (duration > 0U)
						{
							lift_moving = 1U;
							lift_move_end_tick = osKernelGetTickCount() + duration;
						}
						else
						{
							lift_moving = 0U;
						}
					}
					else
					{
						lift_moving = 0U;
						lift_dir = 0;
					}

					main_lift_cmd_prev = main_lift_target_active;
				}

				/* --- [执行层] 运动中发速度；到时后停机并更新位置估计 --- */
				if (lift_moving != 0U)
				{
					/* 运行中方向兜底：防止lift_dir偶发为0导致不进速度分支 */
					if (lift_dir == 0)
					{
						if ((int32_t)main_lift_cmd_prev > (int32_t)main_lift_pos_est) lift_dir = +1;
						else if ((int32_t)main_lift_cmd_prev < (int32_t)main_lift_pos_est) lift_dir = -1;
					}
					if ((int32_t)(lift_move_end_tick - osKernelGetTickCount()) <= 0)
					{
						lift_moving = 0U;
						main_lift_pos_est = main_lift_cmd_prev;
						lift_dir = 0;
						main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.3f, -1.0f);
					}
					else
					{
						if (lift_dir > 0) main_lift.set_mit_data(&main_lift, 0.0f, v_up, 0.0f, 0.3f, -1.0f);
						else if (lift_dir < 0) main_lift.set_mit_data(&main_lift, 0.0f, v_down, 0.0f, 0.3f, -1.0f);
						else main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.3f, -1.0f);
					}
				}
				else
				{
					main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.3f, -1.0f);
				}
				main_lift_busy = lift_moving;
			}
			else
			{
				main_lift_busy = 0U;
				main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			}
		}

	/* ==================== 前臂旋转 ==================== */


	static uint16_t ch4_prev = 0;
	static uint16_t ch2_pos_prev = 0; /* 位置模式下 CH2 档位切换边沿检测 */

		if (control_mode == remote_control)
		{
			if (RCctrl.CH4 >=1500 && ch4_prev <=1500)
			{
				kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position + 1) % 2);
			}
			if (RCctrl.CH4<=500 && ch4_prev >=500)
			{
				kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position - 1+2) % 2);
			}
			ch4_prev = RCctrl.CH4;
		}

float tar_spin;
	switch(kfs_spin_position)
	{
		case kfs_spin_p1:
			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET1;
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 11.0f, 2.6f, -4.0f);
		break;
		case kfs_spin_p2:
			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET2;
			// kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 6.8f, 2.2f, 0.0f);
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 12.0f, 2.5f, 0.0f);
		break;
	}


	
	
	/* ==================== 上下伸缩 速度/位置 四模式（CH5 循环切换） ==================== */

	/* --- CH5/CH2 遥控边沿处理（仅遥控模式） --- */
	if (control_mode == remote_control)
	{
		/* 从其他模式切回遥控时，同步上一拍输入，避免CH5/CH2边沿误触发 */
		if (last_control_mode != remote_control)
		{
			ch5_prev = RCctrl.CH5;
			ch2_pos_prev = RCctrl.CH2;
		}

		/* CH5 LOW 边沿触发：四模式循环 0->1->2->3->0 */
		if (RCctrl.CH5 == CH5_LOW && ch5_prev != CH5_LOW)
		{
			flexible_mode = (Flexible_Mode)(((int)flexible_mode + 1) % 4);
		}
		ch5_prev = RCctrl.CH5;

		/* 位置模式下：CH2 边沿切换目标档位 */
		if (flexible_mode == flex_below_position || flexible_mode == flex_above_position)
		{
			if (RCctrl.CH2 >= 1500 && ch2_pos_prev < 1500)
			{
				if (flex_target_pos < flex_pos3)
					flex_target_pos = (Flex_TargetPos)((int)flex_target_pos + 1);
			}
			if (RCctrl.CH2 <= 500 && ch2_pos_prev > 500)
			{
				if (flex_target_pos > flex_pos0)
					flex_target_pos = (Flex_TargetPos)((int)flex_target_pos - 1);
			}
		}
		ch2_pos_prev = RCctrl.CH2;
	}

	/* --- 电机执行（遥控 + 全自动 均可驱动） --- */
	if (control_mode == remote_control || control_mode == full_auto_control)
	{
		static Flexible_Mode flex_mode_prev = flex_below_speed;

		/* 全自动模式：根据 kfs_below_cmd / kfs_above_cmd 自动切换模式与档位 */
		if (control_mode == full_auto_control)
		{
			if (kfs_below_cmd != kfs_below_cmd_stop)
			{
				flexible_mode = flex_below_position;
				flex_target_pos = (Flex_TargetPos)((int)kfs_below_cmd - 1);
			}
			else if (kfs_above_cmd != kfs_above_cmd_stop)
			{
				flexible_mode = flex_above_position;
				flex_target_pos = (Flex_TargetPos)((int)kfs_above_cmd - 1);
			}
			else
			{
				flexible_mode = flex_below_speed; /* 无指令时停止 */
			}
		}

		/* 检测模式切换：切入位置模式时自动记录基准圈数并复位PID */
		if (flexible_mode != flex_mode_prev)
		{
			if (flexible_mode == flex_below_position)
			{
				flex_pos_base_rounds = kfs_below.round_cnt;
				flex_target_pos = flex_pos0;
				flex_pos_integral = 0.0f;
				flex_pos_last_error = 0.0f;
				flex_pos_inited = 1U;
			}
			else if (flexible_mode == flex_above_position)
			{
				flex_pos_base_rounds = kfs_above.round_cnt;
				flex_target_pos = flex_pos0;
				flex_pos_integral = 0.0f;
				flex_pos_last_error = 0.0f;
				flex_pos_inited = 1U;
			}
			flex_mode_prev = flexible_mode;
		}

		{
			float above_cmd = 0.0f;
			float below_cmd = 0.0f;

			switch (flexible_mode)
			{
			case flex_below_speed:
				/* below 速度模式：遥控CH2直驱；全自动模式下无动作 */
				if (control_mode == remote_control)
					below_cmd = (RCctrl.CH2 - 992) * 200;
				break;

			case flex_above_speed:
				/* above 速度模式：遥控CH2直驱（方向相反） */
				if (control_mode == remote_control)
					above_cmd = (992 - RCctrl.CH2) * 200;
				break;

			case flex_below_position:
			case flex_above_position:
			{
				/* 位置模式：遥控/全自动共用位置环PID */
				float target_rounds = (float)flex_pos_base_rounds
				                    + (flexible_mode == flex_below_position ? kfs_below_pos_param.pos_rounds[flex_target_pos] : -kfs_above_pos_param.pos_rounds[flex_target_pos]);

				int32_t fb_rounds = (flexible_mode == flex_below_position)
				                     ? kfs_below.round_cnt
				                     : kfs_above.round_cnt;
				Kfs_Flex_PosCtrl_Param volatile *pos_p = (flexible_mode == flex_below_position)
				                      ? &kfs_below_pos_param
				                      : &kfs_above_pos_param;
				float raw_cmd = kfs_flex_position_pid(pos_p, target_rounds, (float)fb_rounds);

				/* 输出 x200 送入速度环；above 方向相反 */
				if (flexible_mode == flex_below_position)
					below_cmd = raw_cmd * 200.0f;
				else
					above_cmd =  raw_cmd * 200.0f; /* above 方向已在 target 取反，输出不再取反 */
				break;
			}
			default:
				break;
			}

			kfs_above.PID_Calculate(&kfs_above, above_cmd);
			kfs_below.PID_Calculate(&kfs_below, below_cmd);
		}
	}
	else
	{
		/* 非遥控/非全自动模式：上下伸缩停止 */
		kfs_above.PID_Calculate(&kfs_above, 0);
		kfs_below.PID_Calculate(&kfs_below, 0);
		flex_pos_inited = 0U;
	}

	last_control_mode = control_mode;



 	DJIset_motor_data(&hfdcan3, 0X200, kfs_above.pid_spd.Output,kfs_below.pid_spd.Output,0.0f,0.0f);

}