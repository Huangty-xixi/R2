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
volatile int32_t main_lift_dbg_target = 0;
volatile int32_t main_lift_dbg_current = 0;
volatile uint32_t main_lift_dbg_duration = 0U;
volatile int8_t main_lift_dbg_dir = 0;
volatile uint8_t main_lift_dbg_moving = 0U;

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
	main_lift.set_mit_data(&main_lift, MAIN_LIFT_OFFSET1, 0.0f, 0.2, 0.15f, -5.0f);
 	kfs_spin.set_mit_data(&kfs_spin, kfs_spin_Initpos + KFS_SPIN_OFFSET1, 0.0f, 6.5f, 2.0f, 0.0f);

	three_kfs_position = three_kfs_p1;
	main_lift_position = main_lift_p0; /* 开机默认不动作，等待上位机001再动 */
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
		 * 命令编码：
		 * 00 不动
		 * 01 伸出 -> -100
		 * 10 收回 -> +100
		 * 11 预留（按不动处理）
		 */
		{
			uint8_t above_cmd = (uint8_t)((kfs_action_word >> 6) & 0x03U);
			uint8_t below_cmd = (uint8_t)((kfs_action_word >> 8) & 0x03U);

			if (above_cmd == 0x01U)
			{
				master_kfs_above_spd_cmd = -100;
			}
			else if (above_cmd == 0x02U)
			{
				master_kfs_above_spd_cmd = 100;
			}
			else
			{
				master_kfs_above_spd_cmd = 0;
			}

			if (below_cmd == 0x01U)
			{
				master_kfs_below_spd_cmd = -100;
			}
			else if (below_cmd == 0x02U)
			{
				master_kfs_below_spd_cmd = 100;
			}
			else
			{
				master_kfs_below_spd_cmd = 0;
			}
		}
	}

	// 通道一控制三档旋转kfs
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
	
		/* 遥控：CH3边沿换挡（与CH4切挡风格一致） */
		if (control_mode == remote_control)
		{
			static uint16_t ch3_prev = 0;
			static uint8_t ch3_cmd_lock = 0U; /* 1=主轴动作执行中，忽略新换挡命令 */

			if (control_mode == remote_control)
			{
				if (RCctrl.CH3 == CH3_HIGH && ch3_prev != CH3_HIGH && ch3_cmd_lock == 0U)
				{
					main_lift_position = (Main_lift_position)(((int)main_lift_position + 1) % 5);
				}
				if (RCctrl.CH3 == CH3_LOW && ch3_prev != CH3_LOW && ch3_cmd_lock == 0U)
				{
					main_lift_position = (Main_lift_position)(((int)main_lift_position - 1+5) % 5);
				}
				ch3_prev = RCctrl.CH3;
			}
			/* main_lift.set_mit_data(&main_lift, 0.0f, (992.0f - (float)RCctrl.CH3) / 150.0f, 0.0f, 0.3f, -1.0f); */
			/* 兜底边沿：避免摇杆值没精确到192/1792时触发不到换挡 */
			{
				static uint8_t ch3_zone_prev = 1U; /* 0=LOW,1=MID,2=HIGH */
				uint8_t ch3_zone = 1U;
				if (RCctrl.CH3 >= (CH3_HIGH - 120)) ch3_zone = 2U;
				else if (RCctrl.CH3 <= (CH3_LOW + 120)) ch3_zone = 0U;

				if (ch3_zone == 2U && ch3_zone_prev != 2U && RCctrl.CH3 != CH3_HIGH && ch3_cmd_lock == 0U)
				{
					main_lift_position = (Main_lift_position)(((int)main_lift_position + 1) % 5);
				}
				if (ch3_zone == 0U && ch3_zone_prev != 0U && RCctrl.CH3 != CH3_LOW && ch3_cmd_lock == 0U)
				{
					main_lift_position = (Main_lift_position)(((int)main_lift_position - 1 + 5) % 5);
				}
				ch3_zone_prev = ch3_zone;
			}
			ch3_cmd_lock = main_lift_dbg_moving;

		}
		/* 主轴最简控制：档位变化 -> 固定速度 + 分段计时 -> 到时停止 */
		{
			static Main_lift_position main_lift_cmd_prev = main_lift_p0;
			static Main_lift_position main_lift_pos_est = main_lift_p0;
			static Main_lift_position main_lift_target_active = main_lift_p0;
			static Main_lift_position main_lift_target_pending = main_lift_p0;
			static uint8_t main_lift_pending_valid = 0U;
			static uint8_t lift_moving = 0U;
			static int8_t lift_dir = 0; /* +1上升，-1下降 */
			static uint32_t lift_move_end_tick = 0U;
			const float v_up = -2.5f;
			const float v_down = 2.5f;
			const uint32_t t_up_ms[4]   = {420U, 400U, 380U, 360U};
			const uint32_t t_down_ms[4] = {360U, 380U, 400U, 420U};

			if (control_mode == master_control || control_mode == remote_control)
			{
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

				if (main_lift_target_active != main_lift_cmd_prev)
				{
					uint32_t duration = 0U;
					main_lift_dbg_target = (int32_t)main_lift_target_active;
					main_lift_dbg_current = (int32_t)main_lift_pos_est;

					if (main_lift_target_active == main_lift_p0)
					{
						lift_moving = 0U;
						lift_dir = 0;
					}
					else if ((int32_t)main_lift_target_active > (int32_t)main_lift_pos_est)
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
							lift_move_end_tick = osKernelSysTick() + duration;
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
							lift_move_end_tick = osKernelSysTick() + duration;
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
					main_lift_dbg_duration = duration;
					main_lift_dbg_dir = lift_dir;
					main_lift_dbg_moving = lift_moving;
				}

				if (lift_moving != 0U)
				{
					/* 运行中方向兜底：防止lift_dir偶发为0导致不进速度分支 */
					if (lift_dir == 0)
					{
						if ((int32_t)main_lift_cmd_prev > (int32_t)main_lift_pos_est) lift_dir = +1;
						else if ((int32_t)main_lift_cmd_prev < (int32_t)main_lift_pos_est) lift_dir = -1;
					}
					if ((int32_t)(lift_move_end_tick - osKernelSysTick()) <= 0)
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
				main_lift_dbg_dir = lift_dir;
				main_lift_dbg_moving = lift_moving;
			}
			else
			{
				main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			}
		}

	/* ==================== 主轴旋转 ==================== */


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
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.1f, 0.4f, 0.0f);
		break;
	}


	
	
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
			/* 閸忔湹绮Ο鈥崇础娑撳绔婚梿璁圭礉闁灝鍘ゅ▽璺ㄦ暏娑撳﹣绔撮幏宥嗙暙閻ｆ瑨绶崙? */
			kfs_above.PID_Calculate(&kfs_above, 0);
			kfs_below.PID_Calculate(&kfs_below, 0);
		}

		last_control_mode = control_mode;



 	DJIset_motor_data(&hfdcan3, 0X200, kfs_above.pid_spd.Output,kfs_below.pid_spd.Output,0.0f,0.0f);

}
