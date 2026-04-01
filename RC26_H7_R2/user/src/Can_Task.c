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
//HAL_StatusTypeDef flag;
//HAL_StatusTypeDef d=HAL_ERROR;

static uint8_t lift_has_stopped = 0;   // 1=已触限位停机
static uint8_t lift_running = 0;
static int    lift_stop_mode  = 0;     // 记录是上升停还是下降停，用于给刹车力矩
//    	int test = 0;
			
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

//			 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,test);
			
          if(Chassis.super_struct.base.error_code == 0x00)
          {
				
              chassis_motor1.PID_Calculate(&chassis_motor1, 50*Chassis.param.V_out[0]);
              chassis_motor2.PID_Calculate(&chassis_motor2, 50*Chassis.param.V_out[1]);
              chassis_motor3.PID_Calculate(&chassis_motor3, 50*Chassis.param.V_out[2]);
              chassis_motor4.PID_Calculate(&chassis_motor4, 50*Chassis.param.V_out[3]);
						 
							guide_motor1.PID_Calculate(&guide_motor1, 20*Chassis.param.V_out[0]);
              guide_motor2.PID_Calculate(&guide_motor2, 20*Chassis.param.V_out[1]);
              
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
											DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output, 0, 0);										
										               
//                            // ==================== 升降电机防掉负载修复 ====================
//                            static int last_mode = -1;

//                            // 模式切换 → 复位所有状态
//                            if(r2_lift_mode != last_mode)
//                            {
//                                last_mode = r2_lift_mode;
//                                lift_has_stopped = 0;
//                                lift_running = 0;
//                            }

//                            // 已经触底/触顶停止 → 输出刹车力矩，不掉落
//                            if(lift_has_stopped)
//                            {
//                                if(lift_stop_mode == fall)
//                                {
//                                    // 上升到顶：给微小向下力矩顶住不下滑
//                                    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f,  -0.15f);
//                                    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f, 0.3f);
//                                }
//                                else if(lift_stop_mode == raise)
//                                {
//                                     // 下降到底：给一个微小向上力矩顶住不回落
//                                    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f, 1.6f);
//                                    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f,  -2.5f);
//                                }
//                                break;
//                            }

//                            // 正常运行
//                            if(r2_lift_mode == fall)
//                            {

//                                R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -2.0f, 0, 0.05f, -2.0f);
//                                R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0,  2.0f, 0, 0.05f,  2.0f);

//                                if(fabsf(R2_lift_motor_left.speed_w) > 1.5f || fabsf(R2_lift_motor_right.speed_w) > 1.5f)                                
//                                {
//                                    lift_running = 1;
//                                }

//                                // 触底停止
//                                if(lift_running && 
//                                   fabsf(R2_lift_motor_left.speed_w) < 0.5f && fabsf(R2_lift_motor_right.speed_w) < 0.5f)                               
//                                {
//                                    lift_has_stopped = 1;
//                                    lift_stop_mode = fall;  // 记录停止模式
//                                }
//                            }
//                            else if(r2_lift_mode == raise)
//                            {
//                                R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0,  2.5f, 0, 0.05f,  2.0f);
//                                R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, -2.5f, 0, 0.05f, -2.3f);


//                                if(fabsf(R2_lift_motor_left.speed_w) > 1.5f || fabsf(R2_lift_motor_right.speed_w) > 1.5f)                                 
//                                {
//                                    lift_running = 1;
//                                }

//                                // 触顶停止
//                                if(lift_running && 
//                                   fabsf(R2_lift_motor_left.speed_w) < 0.5f &&fabsf(R2_lift_motor_right.speed_w) < 0.5f)                                  
//                                {
//                                    lift_has_stopped = 1;
//                                    lift_stop_mode = raise; // 记录停止模式
//                                }
//                            }

//                            break;
                        }

                    case weapon_switch:
											switch(weapon_mode)
											{
													case weapon_none:
															break;
													case steering_mode:
															Chassis.Chassis_Stop(&Chassis);//对接，锁定底盘

															break;
													case pump_mode :
 																  	pump_motion ();//相关函数调用

													
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


