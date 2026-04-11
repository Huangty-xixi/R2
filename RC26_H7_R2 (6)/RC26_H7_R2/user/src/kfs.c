#include "kfs.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include <math.h>
#include "cmsis_os.h"

Kfs_Module Kfs;

DJI_MotorModule kfs_above;  
DJI_MotorModule kfs_below;  

DM_MotorModule main_lift;
DM_MotorModule kfs_spin;
DM_MotorModule three_kfs;

Three_kfs_position three_kfs_position;
Kfs_spin_position kfs_spin_position;
Main_lift_position main_lift_position;

//上电初始位置
float main_lift_Initpos = 0.0f;
float kfs_spin_Initpos = 0.0f;
float three_kfs_Initpos = 1.6f;

float kfs_above_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};
float kfs_below_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

//初始化：读取上电初始位置
void kfs_three_kfs_spin_main_lift_pos_init(void)
{
	three_kfs.set_mit_data(&three_kfs, three_kfs_Initpos, 0.0f, 0.5f, 0.2f, 0.2f);
	main_lift.set_mit_data(&main_lift, main_lift_Initpos, 0.0f, 0.5f, 0.2f, 0.2f);
	kfs_spin.set_mit_data(&kfs_spin, kfs_spin_Initpos, 0.0f, 0.5f, 0.2f, 0.2f);

	three_kfs_position = three_kfs_p1;
	main_lift_position = main_lift_p1;
	kfs_spin_position  = kfs_spin_p1;
}

// void Initpos_Get(void)
// {
// 	kfs_three_kfs_spin_main_lift_pos_init();
// }

/**
  * @brief kfs函数
  */
void manual_kfs_function(void)
{
	/* ========== [临时调参·已停用] 三个kfs：原 CH5 切多套 PID（整段保留备查） ========== */
	// /* 临时调参：CH5拨到高位一次，切换到下一套PID参数（三个kfs旋转，已停用） */
	// static uint8_t three_kfs_pid_idx = 0;
	// static uint8_t ch5_high_latched = 0;
	// static uint32_t ch5_last_switch_ms = 0;
	// const uint32_t ch5_debounce_ms = 180;
	// static const float three_kfs_pid_sets[][3] = {
	// 	{0.5f, 0.2f, 0.2f},
	// 	{0.8f, 0.25f, 0.2f},
	// 	{1.1f, 0.30f, 0.2f},
	// 	{1.4f, 0.35f, 0.2f},
	// };
	// const uint8_t three_kfs_pid_set_num = (uint8_t)(sizeof(three_kfs_pid_sets) / sizeof(three_kfs_pid_sets[0]));
	// uint32_t now_ms = HAL_GetTick();

	// if (RCctrl.CH5 == CH5_HIGH)
	// {
	// 	if (!ch5_high_latched && (now_ms - ch5_last_switch_ms >= ch5_debounce_ms))
	// 	{
	// 		three_kfs_pid_idx = (uint8_t)((three_kfs_pid_idx + 1) % three_kfs_pid_set_num);
	// 		ch5_last_switch_ms = now_ms;
	// 	}
	// 	ch5_high_latched = 1;
	// }
	// else
	// {
	// 	ch5_high_latched = 0;
	// }
	/* ======================== [临时调参·已停用] 以上结束 ======================== */

//通道一控制三个kfs旋转
	static uint16_t ch1_prev = 0;
	
	if (RCctrl.CH1 == CH1_HIGH && ch1_prev != CH1_HIGH)
	{
		three_kfs_position = (Three_kfs_position)(((int)three_kfs_position + 1) % 3);
	}
	if (RCctrl.CH1 == CH1_LOW && ch1_prev != CH1_LOW)
	{
		three_kfs_position = (Three_kfs_position)(((int)three_kfs_position - 1+3) % 3);
	}
	ch1_prev = RCctrl.CH1;


float tar_3k;
	switch(three_kfs_position)
	{
		case three_kfs_p1:
			tar_3k = THREE_KFS_OFFSET1;
		break;
		case three_kfs_p2:
			tar_3k = THREE_KFS_OFFSET2;
		break;
		case three_kfs_p3: 
			tar_3k = THREE_KFS_OFFSET3;
		break;
		default: tar_3k = three_kfs_Initpos;
	}

	/* [原逻辑] 三个kfs：固定 MIT 增益（停用临时调参后恢复为此写法） */
	three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.5f, 0.2f, 0.2f);
	
//通道三控制主升机构升降
	static uint16_t ch3_prev = 0;
	
	if (RCctrl.CH3 == CH3_HIGH && ch3_prev != CH3_HIGH)
	{
		main_lift_position = (Main_lift_position)(((int)main_lift_position + 1) % 3);	
	}	
	if (RCctrl.CH3 == CH3_LOW && ch3_prev != CH3_LOW)
	{
		main_lift_position = (Main_lift_position)(((int)main_lift_position - 1+3) % 3);	
	}
	ch3_prev = RCctrl.CH3;


float tar_lift;
	switch(main_lift_position)
	{
		case main_lift_p1:
			tar_lift = MAIN_LIFT_OFFSET1;
			main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.2, 0.15f, -5.0f);
		break;
		case main_lift_p2:
			tar_lift = MAIN_LIFT_OFFSET2;
			main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.2, 0.15f, -6.0f);
		break;
		case main_lift_p3:
			tar_lift = MAIN_LIFT_OFFSET3;
			main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.2, 0.15f, -7.0f);
		break;
		case main_lift_p4:
			tar_lift = MAIN_LIFT_OFFSET4;
			main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.2, 0.15f, -8.0f);
		break;		
		default: tar_lift = main_lift_Initpos;
	}
//	main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.2, 0.15f, -5.0f);
//	main_lift.set_mit_data(&main_lift, 0, 1.0, 0, 0.2, (988-RCctrl.CH3)/100);
				
//通道四控制前臂kfs旋转
	/* ---------- [临时调参·新增] kfs_spin：CH5 置高边沿 + 防抖，循环切换多套 {KP,KD,T} ---------- */
	static uint8_t kfs_spin_pid_idx = 0;
	static uint8_t kfs_spin_ch5_high_latched = 0;
	static uint32_t kfs_spin_ch5_last_switch_ms = 0;
	const uint32_t kfs_spin_ch5_debounce_ms = 180;
	static const float kfs_spin_pid_sets[][3] = {
		{6.5f, 2.0f, 0.0f},
		{0.2f, 1.0f, 0.0f},
		{7.0f, 2.2f, 0.0f},
		{0.3f, 1.1f, 0.0f},
	};
	const uint8_t kfs_spin_pid_set_num = (uint8_t)(sizeof(kfs_spin_pid_sets) / sizeof(kfs_spin_pid_sets[0]));
	uint32_t kfs_spin_now_ms = HAL_GetTick();

	if (RCctrl.CH5 == CH5_HIGH)
	{
		if (!kfs_spin_ch5_high_latched && (kfs_spin_now_ms - kfs_spin_ch5_last_switch_ms >= kfs_spin_ch5_debounce_ms))
		{
			kfs_spin_pid_idx = (uint8_t)((kfs_spin_pid_idx + 1) % kfs_spin_pid_set_num);
			kfs_spin_ch5_last_switch_ms = kfs_spin_now_ms;
		}
		kfs_spin_ch5_high_latched = 1;
	}
	else
	{
		kfs_spin_ch5_high_latched = 0;
	}
	/* ---------- [临时调参·新增] CH5 切换逻辑结束 ---------- */

	/* [原逻辑] CH4 边沿切换 kfs_spin 档位 */
	static uint16_t ch4_prev = 0;

		if (RCctrl.CH4 == CH4_HIGH && ch4_prev != CH4_HIGH)
		{
			kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position + 1) % 2);
		}
		if (RCctrl.CH4 == CH4_LOW && ch4_prev != CH4_LOW)
		{
			kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position - 1+2) % 2);
		}
		ch4_prev = RCctrl.CH4;

	/* [原逻辑] 仅根据档位算目标角；各档 MIT 参数曾在 case 内分别下发（见下行注释） */
float tar_spin;
	switch(kfs_spin_position)
	{
		case kfs_spin_p1:
			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET1;
			/* [原逻辑·已注释] 档内固定增益下发 */
//			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 6.5f, 2.0f, 0.0f);
		break;
		case kfs_spin_p2:
			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET2;
			/* [原逻辑·已注释] 档内固定增益下发 */
//			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.2f, 1.0f, 0.0f);
		break;
//		case kfs_spin_p3:
//			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET3;
//		break;
		default: tar_spin = kfs_spin_Initpos;
	}

	/* [临时调参·新增] 统一 MIT 下发，增益来自上方 kfs_spin_pid_sets + CH5 切换的 idx */
	kfs_spin.set_mit_data(
		&kfs_spin,
		tar_spin,
		0.0f,
		kfs_spin_pid_sets[kfs_spin_pid_idx][0],
		kfs_spin_pid_sets[kfs_spin_pid_idx][1],
		kfs_spin_pid_sets[kfs_spin_pid_idx][2]);
//	kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.2f, 0.3f, -2.0f);  /* [原逻辑·历史注释] 旧试验参数 */
	
	
//通道二控制伸缩
	
	// CH5切换控制电机
// 	if (RCctrl.CH5 == CH5_LOW && ch5_prev != CH5_LOW)
// 	{
// 		kfs_motor_select = !kfs_motor_select;
// 	}
// 	ch5_prev = RCctrl.CH5;
	
// 		if(kfs_motor_select==0)
// 		{
// 			kfs_above.PID_Calculate(&kfs_above,(992-RCctrl.CH2)*8);
// 			kfs_below.PID_Calculate(&kfs_below,0);
// 		}
// 		else
// 		{
// 			kfs_above.PID_Calculate(&kfs_above,0);
// 			kfs_below.PID_Calculate(&kfs_below,(RCctrl.CH2-992)*8);
// 		}
// 	DJIset_motor_data(&hfdcan3, 0X200, kfs_above.pid_spd.Output,kfs_below.pid_spd.Output,0.0f,0.0f);

}
