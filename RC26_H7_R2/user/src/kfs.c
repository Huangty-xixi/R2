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
			tar_lift = main_lift_Initpos + MAIN_LIFT_OFFSET1;
		break;
		case main_lift_p2:
			tar_lift = main_lift_Initpos + MAIN_LIFT_OFFSET2;
		break;
		case main_lift_p3:
			tar_lift = main_lift_Initpos + MAIN_LIFT_OFFSET3;
		break;
		default: tar_lift = main_lift_Initpos;
	}
	main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0, 0.15f, 0.0f);
				
//通道四控制kfs旋转		   
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


float tar_spin;
	switch(kfs_spin_position)
	{
		case kfs_spin_p1:
			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET1;
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 6.5f, 2.0f, 0.0f);

		break;
		case kfs_spin_p2:
			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET2;
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.2f, 1.0f, 0.0f);

		break;
//		case kfs_spin_p3:
//			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET3;
		break;
		default: tar_spin = kfs_spin_Initpos;
	}
//	kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.2f, 0.3f, -2.0f);
	
	
//通道二控制伸缩
	
	// CH5切换控制电机
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
	DJIset_motor_data(&hfdcan3, 0X200, kfs_above.pid_spd.Output,kfs_below.pid_spd.Output,0.0f,0.0f);

}
