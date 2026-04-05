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
Kfs_spin_pos_t kfs_spin_slot;

float kfs_spin_Initpos=0.0f;


float kfs_above_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};
float kfs_below_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

void kfs_three_kfs_spin_pos_init(void)
{
	three_kfs.set_mit_data(&three_kfs, 1.0f, 2.0f, 0, 0.15f, 3.0f);
	three_kfs_position = position2;
	kfs_spin.set_mit_data(&kfs_spin, 1.0f, 2.0f, 0, 0.15f, 3.0f);
	kfs_spin_slot = kfs_spin_p2;
}

/**
  * @brief kfs????
  */
void manual_kfs_function(void)
{


					static uint16_t ch1_prev = 0;
	
					if (RCctrl.CH1 == CH1_HIGH && ch1_prev != CH1_HIGH)
					{
						switch (three_kfs_position)
						{
						case position1:
							three_kfs.set_mit_data(&three_kfs, 1.0f, 2.0f, 0, 0.15f, 3.0f);
							break;
						case position2:
							three_kfs.set_mit_data(&three_kfs, 2.0f, 2.0f, 0, 0.15f, 3.0f);
							break;
						case position3:
							three_kfs.set_mit_data(&three_kfs, 3.0f, 2.0f, 0, 0.15f, 3.0f);
							break;
						}
						three_kfs_position = (Three_kfs_position)(((int)three_kfs_position + 1) % 3);
					}
					ch1_prev = RCctrl.CH1;
				
				
				

					static uint16_t ch4_prev = 0;
					
					if (RCctrl.CH4 == CH4_HIGH && ch4_prev != CH4_HIGH)
					{
						switch (kfs_spin_slot)
						{
						case kfs_spin_p1:
							kfs_spin.set_mit_data(&kfs_spin, 1.0f, 2.0f, 0, 0.15f, 3.0f);
							break;
						case kfs_spin_p2:
							kfs_spin.set_mit_data(&kfs_spin, 2.0f, 2.0f, 0, 0.15f, 3.0f);
							break;
						case kfs_spin_p3:
							kfs_spin.set_mit_data(&kfs_spin, 3.0f, 2.0f, 0, 0.15f, 3.0f);
							break;
						}
						kfs_spin_slot = (Kfs_spin_pos_t)(((int)kfs_spin_slot + 1) % 3);
					}
					ch4_prev = RCctrl.CH4;
				

				//发送数据驱动电机
				main_lift.set_mit_data(&main_lift, KFS_AXIS_LIFT,  2.0f, 0, 0.15f,  3.0f);
				kfs_above.PID_Calculate(&kfs_above,KFS_FLEXIBLE);
				kfs_below.PID_Calculate(&kfs_below,KFS_FLEXIBLE-100);
				DJIset_motor_data(&hfdcan3, 0X200, kfs_above.pid_spd.Output,kfs_below.pid_spd.Output,0.0f,0.0f);
		
//

//void Initpos_Get()
//{
//    kfs_spin_Initpos =kfs_spin.position ;


//    set_param(1, Initpos, 5, 0, 0.3, 0);

//    // 3. 
//    vTaskDelay(200);
//}
//	if(RCctrl.CH4==992)
//	{
//	
//	kfs_spin_Initpos==
//	}
//	DMset_mit_data (&kfs_spin,kfs_spin_Initpos,0.5,2.0,2.5,0);
}
