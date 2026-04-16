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
#include "remote_control.h"




void Can_Task(void const * argument)
{
    TickType_t Systick = 0;
    uint32_t can1_free_level = 0;
    uint32_t can2_free_level = 0;
    uint32_t can3_free_level = 0;
   
    for(;;)
    {
        Systick = osKernelSysTick();

//          if(Chassis.super_struct.base.error_code == 0x00)
//          {
				
						
             
//					}

//          if(Lift.super_struct.base.error_code == 0x00)
//          {
//              
//          }
//          if(Weapon.super_struct.base.error_code == 0x00)
//          {
//						
//          }
//	
		
		if (RCctrl.rc_lost){
			if(Systick % 2 == 1){	
                 Chassis.Chassis_Stop(&Chassis);
				DJIset_motor_data(&hfdcan1, 0X200,0,0,0,0);
				DJIset_motor_data(&hfdcan2, 0X200,0,0,0,0);
			}
			if(Systick % 2 == 0){	
			
			}
		}
		else{
            switch(control_mode)
            {
                case master_control:
                    /* 并行调度：各模块按enable位独立运行，可同时生效 */
                    if ((master_enable_bits & MASTER_EN_CHASSIS) != 0U)
                    {
                        manual_chassis_function();
                    }
                    if ((master_enable_bits & MASTER_EN_WEAPON) != 0U)
                    {
                        manual_weapon_function();
                    }
                    if ((master_enable_bits & MASTER_EN_LIFT) != 0U)
                    {
                        manual_lift_function();
                    }
                    if ((master_enable_bits & MASTER_EN_KFS) != 0U)
                    {
                        manual_kfs_function();
                    }
                    break;
                case emergency_stop_mode:
                    /* 急停模式：主动清零所有输出，避免残留命令继续驱动 */
                    Chassis.Chassis_Stop(&Chassis);
                    DJIset_motor_data(&hfdcan1, 0X200, 0, 0, 0, 0);
                    DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
                    DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

                    /* DM电机（MIT）清零：kp/kd/torque全0 */
                    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                    /* 急停时将weapon相关执行器拉回初始化电平 */
                    servo_state = 1U;
                    clamp_state = 0U;
                    sucker1_state = 0U;
                    sucker2_state = 0U;
                    sucker3_state = 0U;
                    sucker4_state = 0U;
                    pump1_state = 0U;
                    pump2_state = 0U;

                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);            /* 舵机初始化位 */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);           /* 夹爪初始化电平 */
                    pump1_two_suckers_linkage_nominal_open(0U, 0U);                /* 吸盘1/2与泵1初始化 */
                    pump2_two_suckers_linkage_nominal_open(0U, 0U);                /* 吸盘3/4与泵2初始化 */
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
											Chassis.Chassis_Stop(&Chassis);
										   // 直接发0，刹死！
											DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
											manual_lift_function();
										break;
										case kfs_mode:
											Chassis.Chassis_Stop(&Chassis);
										   // 直接发0，刹死！
											DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
											manual_kfs_function();
										break;
										case remote_none:
										break;
									}
                break;
            }

//			if(Systick % 10 == 0){	
//                
//                
//			}
//			if(Systick % 10 == 5){	
//                
//                
//			}

		}
        can1_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        can2_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
		    can3_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3);

		osDelay(3);
    }

}


