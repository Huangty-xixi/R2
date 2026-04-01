#include "Can_Task.h"
#include "Motion_Task.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"

//HAL_StatusTypeDef flag;
//HAL_StatusTypeDef d=HAL_ERROR;

static float flexible_motor_PID_input;

void Can_Task(void const * argument)
{
    TickType_t Systick = 0;
    uint32_t can1_free_level = 0;
    uint32_t can2_free_level = 0;
    
    
//    while(kfs_lift_motor.send_cmd(&kfs_lift_motor,Motor_Enable) != HAL_OK);
//    kfs_flex_motor.send_cmd(&kfs_flex_motor,Motor_Enable);
//    R2_lift_motor_left.send_cmd(&R2_lift_motor_left,Motor_Enable);
//    R2_lift_motor_right.send_cmd(&R2_lift_motor_right,Motor_Enable);
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
			}
			if(Systick % 2 == 0){	
			
			}
		}
		else{
            switch(control_mode)
            {
                case master_control:
                    break;
                case remote_control:
                  switch(remote_mode)
                 {
                   case chassis_move:
										 {											
											// 底盘电机正常输出（始终运行）
											DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
											DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);
//											R2_lift();
                       break;
                     }

                    case weapon_switch:
											switch(weapon_mode)
											{
													case weapon_none:
															break;
													case steering_mode:
															Chassis.Chassis_Stop(&Chassis);//对接，锁定底盘
																	  steering_use();//相应函数调用
//                                    weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);//										
//																    weapon_collect_motor.set_mit_data(&weapon_collect_motor,0.f,weapon_collect_input,0.f,0.2,0.f);
															break;
													case pump_mode:
																  	pump_use ();//相关函数调用
//                                  //  kfs_lift_motor.set_mit_data(&kfs_lift_motor,0.f,KFS_LIFT,0.f,0.5,0.f);//单控速
//																    kfs_lift_motor.set_mit_data(&kfs_lift_motor,kfs_lift_position,kfs_lift_v,kfs_lift_kp,kfs_lift_kd,kfs_lift_t);												 
//                                    kfs_flex_motor.set_mit_data(&kfs_flex_motor,kfs_flew_position,kfs_flew_v,kfs_flew_kp,kfs_flew_kd,kfs_flew_t);
////																kfs_flex_motor.set_mit_data(&kfs_flex_motor,0.f,-4,0.f,0.2,0.f);
													
															break;
											}
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
		
		osDelay(1);
    }

}

//数据
//												      switch(r2_lift_mode)
//																		{
//																			case fall:
////                                     R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,0, 8, -0.2, 0.2, 0);
////                                     R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, -9, -0.2, 0.2, 0);
////																		 R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,Initpos[0]-1, 4, 0.8, 0.05, 0.1);
////																     R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,Initpos[1]-1.0, 5, 0.8, 0.05, -2.5);
////																		   R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,Initpos[0]+7, 4, 0.8, 0.05, 0.1);
////																       R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,Initpos[1]-8.0, 5, 0.8, 0.05, -2.5);																			
//                                      break;
//                                      case raise:
////																		 R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,Initpos[0]-10, 5, 0.8, 0.05, -2.5);
////																     R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,Initpos[1]+11, 4, 0.8, 0.05, 0.1);
////																			 R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,Initpos[0]-2, 4, 0.8, 0.05, 0.1);
////																       R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,Initpos[1]+2.0, 5, 0.8, 0.05, -2.5);																			
// 
//																			
//                                      break;
//																		}
//	

