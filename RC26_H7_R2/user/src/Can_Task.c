#include "Can_Task.h"
#include "Motion_Task.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"
#include "tim.h"


static float flexible_motor_PID_input;



void Can_Task(void const * argument)
{
    TickType_t Systick = 0;
    uint32_t can1_free_level = 0;
    uint32_t can2_free_level = 0;
    
    
    for(;;)
    {
        Systick = osKernelSysTick();

        if(Chassis.super_struct.base.error_code == 0x00)
        {
				
            chassis_motor1.PID_Calculate(&chassis_motor1, 50*Chassis.param.V_out[0]);
            chassis_motor2.PID_Calculate(&chassis_motor2, 50*Chassis.param.V_out[1]);
            chassis_motor3.PID_Calculate(&chassis_motor3, 50*Chassis.param.V_out[2]);
            chassis_motor4.PID_Calculate(&chassis_motor4, 50*Chassis.param.V_out[3]);
						 
            guide_motor1.PID_Calculate(&guide_motor1, 200*Chassis.param.V_out[0]);
            guide_motor2.PID_Calculate(&guide_motor2, 200*Chassis.param.V_out[1]);
						
            flexible_motor1.PID_Calculate(&flexible_motor1,flexible_motor_PID_input);
						flexible_motor2.PID_Calculate(&flexible_motor2,flexible_motor_PID_input);
						
             
		}
	
		if (RCctrl.rc_lost){
			if(Systick % 2 == 1){	
                 Chassis.Chassis_Stop(&Chassis);
			}
			if(Systick % 2 == 0){	
			
			}
		}
		else{
            switch(control_mode)
            {
                case master_control:
                    break;
								case part_remote_control:
										break;
                case remote_control:
									switch (remote_mode)
									{
										case chassis_mode:
											manual_chassis_function();
										break;
										
										case weapon_mode:
											manual_weapon_function();
										break;
										
										case lift_mode:
											manual_lift_function();
										break;
										
										case kfs_mode:
											manual_kfs_function();
										break;
										
										case remote_none:
										break;
									}
                break;
            }

		}
        can1_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        can2_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
		
		osDelay(1);
    }

}

