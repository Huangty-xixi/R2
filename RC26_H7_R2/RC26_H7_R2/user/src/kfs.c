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

float kfs_above_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};
float kfs_below_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

// 初始化：读取上电初始位置
void kfs_three_kfs_spin_main_lift_pos_init(void)
{
	three_kfs.set_mit_data(&three_kfs, three_kfs_Initpos, 0.0f, 5.0f, 0.2f, 0.2f);
//	main_lift.set_mit_data(&main_lift, MAIN_LIFT_OFFSET1, 0.0f, 0.2, 0.15f, -5.0f);
 	kfs_spin.set_mit_data(&kfs_spin, kfs_spin_Initpos + KFS_SPIN_OFFSET1, 0.0f, 6.5f, 2.0f, 0.0f);

	three_kfs_position = three_kfs_p1;
	main_lift_position = main_lift_p1; /* 开机初始化到p1（后续遥控仅在p1~p4循环） */
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
	
	int16_t master_kfs_above_spd_cmd = 0;
	int16_t master_kfs_below_spd_cmd = 0;
	static Control_mode last_control_mode = remote_control;

	/* master模式：KFS使用单字节8位动作（master_kfs_action_bits_0）
	 * bit0~1: 三档旋转 00/01/10，11预留
	 * bit2   : 前臂二档 1/0
	 * bit3~5 : 主轴抬升状态编码 0~6，7预留
	 * bit6~7 : 伸缩杆两位置 00/01，10/11预留=停止
	 */
	if (control_mode == master_control)
	{
		uint16_t kfs_action_word = (uint16_t)master_kfs_action_bits_0 |
		                           ((uint16_t)master_kfs_action_bits_1 << 8);
		uint8_t action = (uint8_t)(kfs_action_word & 0xFFU);

		/* bit0~1: 三档旋转 => three_kfs_position */
		switch (action & 0x03U)
		{
			case 0: three_kfs_position = three_kfs_p1; break; /* 00 */
			case 1: three_kfs_position = three_kfs_p2; break; /* 01 */
			case 2: three_kfs_position = three_kfs_p3; break; /* 10 */
			default: three_kfs_position = three_kfs_p1; break; /* 11预留 */
		}

		/* bit2: 前臂两档 => kfs_spin_position */
		kfs_spin_position = ((action & (1U << 2)) != 0U) ? kfs_spin_p2 : kfs_spin_p1; /* 1/0 */

		/* bit3~5: 主轴抬升四状态映射
		 * 001 -> 状态1
		 * 010 -> 状态2
		 * 011 -> 状态3
		 * 100 -> 状态4
		 * 其他值预留：保持当前状态不变
		 */
		{
			uint8_t lift_code = (uint8_t)((action >> 3) & 0x07U);
			switch (lift_code)
			{
				case 0x00U: main_lift_position = main_lift_p0; break; /* 000: 不动 */
				case 0x01U: main_lift_position = main_lift_p1; break; /* 001 */
				case 0x02U: main_lift_position = main_lift_p2; break; /* 010 */
				case 0x03U: main_lift_position = main_lift_p3; break; /* 011 */
				case 0x04U: main_lift_position = main_lift_p4; break; /* 100 */
				default:
					/* reserved */
					break;
			}
		}

		/* 伸缩两电机控制：
		 * 把两个字节拼成16位后取4位（两个电机各占2位命令）
		 * - above电机命令位：bit7~6
		 * - below电机命令位：bit9~8（来自第二字节低2位）
		 *
		 * above命令编码：
		 * 00 不动
		 * 01 伸出 -> -100
		 * 10 收回 -> +100
		 * 11 预留（按不动处理）
		 *
		 * below命令编码（按当前机械方向）：
		 * 00 不动
		 * 01 收回 -> -100
		 * 10 伸出 -> +100
		 * 11 预留（按不动处理）
		 */
		{
			uint8_t above_cmd = (uint8_t)((kfs_action_word >> 6) & 0x03U);
			uint8_t below_cmd = (uint8_t)((kfs_action_word >> 8) & 0x03U);

			if (above_cmd == 0x01U)
			{
				master_kfs_above_spd_cmd = -2500;
			}
			else if (above_cmd == 0x02U)
			{
				master_kfs_above_spd_cmd = 2500;
			}
			else
			{
				master_kfs_above_spd_cmd = 0;
			}

			if (below_cmd == 0x01U)
			{
				master_kfs_below_spd_cmd = -2500; /* 01: 收回 */
			}
			else if (below_cmd == 0x02U)
			{
				master_kfs_below_spd_cmd = 2500;  /* 10: 伸出 */
			}
			else
			{
				master_kfs_below_spd_cmd = 0;
			}
		}
	}

	/* ==================== 三档旋转 ==================== */
	// 通道一控制三档旋转KFS
	static uint16_t ch1_prev = 0;
	static int8_t three_kfs_pingpong_dir = 1; /* 1: p1->p3, -1: p3->p1 */
	
	if (control_mode == remote_control)
	{
		if (RCctrl.CH1 == CH1_HIGH && ch1_prev != CH1_HIGH)
		{
			if (three_kfs_position == three_kfs_p1) three_kfs_pingpong_dir = 1;
			else if (three_kfs_position == three_kfs_p3) three_kfs_pingpong_dir = -1;

			if (three_kfs_pingpong_dir > 0)
			{
				if (three_kfs_position == three_kfs_p1) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p3;
				else three_kfs_position = three_kfs_p2;
			}
			else
			{
				if (three_kfs_position == three_kfs_p3) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p1;
				else three_kfs_position = three_kfs_p2;
			}
		}
		if (RCctrl.CH1 == CH1_LOW && ch1_prev != CH1_LOW)
		{
			if (three_kfs_position == three_kfs_p1) three_kfs_pingpong_dir = 1;
			else if (three_kfs_position == three_kfs_p3) three_kfs_pingpong_dir = -1;

			if (three_kfs_pingpong_dir > 0)
			{
				if (three_kfs_position == three_kfs_p1) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p3;
				else three_kfs_position = three_kfs_p2;
			}
			else
			{
				if (three_kfs_position == three_kfs_p3) three_kfs_position = three_kfs_p2;
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
				if (RCctrl.CH3 >= (CH3_HIGH - 120)) ch3_zone = 2U;
				else if (RCctrl.CH3 <= (CH3_LOW + 120)) ch3_zone = 0U;

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
			const float v_up = -2.5f;                                           /* 上升固定速度 */
			const float v_down = 2.5f;    
			//p0:000 p1:001 p2:010 p3:011 p4:100
			const uint32_t t_up_ms[4]   = {400U, 0U, 2600U, 1500U};
			const uint32_t t_down_ms[4] = {400U, 0U, 2600U, 1500U};

			if (control_mode == master_control || control_mode == remote_control)
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
							if (lvl >= 0 && lvl <= 3) duration += t_up_ms[lvl];
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
							if (lvl >= 1 && lvl <= 4) duration += t_down_ms[lvl - 1];
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

		if (control_mode == remote_control)
		{
			if (RCctrl.CH4 == CH4_HIGH && ch4_prev != CH4_HIGH)
			{
				kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position + 1) % 2);
			}
			if (RCctrl.CH4 == CH4_LOW && ch4_prev != CH4_LOW)
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
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.3f, 0.4f, 0.0f);
		break;
	}


	
	
	/* ==================== 伸缩机构 ==================== */
	// 通道二控制伸缩
	

		if (control_mode == master_control)
		{
			kfs_above.PID_Calculate(&kfs_above, master_kfs_above_spd_cmd);
			kfs_below.PID_Calculate(&kfs_below, master_kfs_below_spd_cmd);
		}
		// CH5切换控制电机
		else if (control_mode == remote_control)
		{
			/* 从其他模式切回遥控时，同步上一拍输入，避免CH5边沿误触发 */
			if (last_control_mode != remote_control)
			{
				ch5_prev = RCctrl.CH5;
			}

			if (RCctrl.CH5 == CH5_LOW && ch5_prev != CH5_LOW)
			{
				kfs_motor_select = !kfs_motor_select;
			}
			ch5_prev = RCctrl.CH5;

			if(kfs_motor_select==0)
			{
				kfs_above.PID_Calculate(&kfs_above,(992-RCctrl.CH2)*8);
				kfs_below.PID_Calculate(&kfs_below,0);
			}
			else
			{
				kfs_above.PID_Calculate(&kfs_above,0);
				kfs_below.PID_Calculate(&kfs_below,(RCctrl.CH2-992)*8);
			}
		}
		else
		{
			/* 其他模式下伸缩电机输出清零 */
			kfs_above.PID_Calculate(&kfs_above, 0);
			kfs_below.PID_Calculate(&kfs_below, 0);
		}

		last_control_mode = control_mode;



 	DJIset_motor_data(&hfdcan3, 0X200, kfs_above.pid_spd.Output,kfs_below.pid_spd.Output,0.0f,0.0f);

}
