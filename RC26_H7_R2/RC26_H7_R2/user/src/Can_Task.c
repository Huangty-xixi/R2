#include "Can_Task.h"
#include "Motion_Task.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"
#include "Process_Flow.h"
#include "app_flow_dispatch.h"
#include "app_clamp_head_ctrl.h"
#include "tim.h"
#include "remote_control.h"
#include "usart.h"
void Can_Task(void const * argument)
{
    uint32_t can1_free_level = 0;
    uint32_t can2_free_level = 0;
    uint32_t can3_free_level = 0;
    uint8_t app_flow_inited = 0U;
    uint8_t app_clamp_head_inited = 0U;
   
    for(;;)
    {
        if (app_flow_inited == 0U)
        {
            AppFlowDispatch_Init();
            app_flow_inited = 1U;
        }
        if (app_clamp_head_inited == 0U)
        {
            AppClampHeadCtrl_Init();
            app_clamp_head_inited = 1U;
        }

        RemoteControl_LinkWatchdog_SimpleTest(&RCctrl);
#if REMOTE_LOST_PROTECT_ENABLE
        RemoteControl_LinkWatchdog_Update(&RCctrl);

        if (RCctrl.rc_lost != false)
        {
            /* 遥控链路丢失：全电机关断输出 */
            Chassis.Chassis_Stop(&Chassis);
            DJIset_motor_data(&hfdcan1, 0X200, 0, 0, 0, 0);
            DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
            DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

            R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

            osDelay(1);
            continue;
        }
#endif

        Motor_OverTemp_SimpleTest();

        if (Motor_OverTempProtect_Update() != 0U)
        {
            /* 过温保护：全电机输出清零并跳过本周期业务控制 */
            Chassis.Chassis_Stop(&Chassis);
            DJIset_motor_data(&hfdcan1, 0X200, 0, 0, 0, 0);
            DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
            DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

            R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

            osDelay(1);
            continue;
        }

            switch(control_mode)
            {
                case semi_auto_control:
                    AppFlowDispatch_Run();
                    Process_Flow_DebugSnapshot();
                    /* 半自动下保持底盘手动：CH1~CH4 与遥控模式一�? */
                    manual_chassis_function();
                    AppClampHeadCtrl_Run();
                    manual_weapon_function();
                    manual_lift_function();
                    manual_kfs_function();
                    break;
                case emergency_stop_mode:
                    /* 急停模式：主动清零所有输出，避免残留命令继续驱动 */
                    Chassis.Chassis_Stop(&Chassis);
                    DJIset_motor_data(&hfdcan1, 0X200, 0, 0, 0, 0);
                    DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
                    DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

                    /* DM电机（MIT）清零：kp/kd/torque�?0 */
                    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.9f, 0.3f, 0.4f, 0.0f);
                    three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                    /* 急停时将weapon相关执行器拉回初始化电平 */
                    servo_state = 1U;
                    clamp_state = 0U;
                    sucker1_state = 0U;
                    sucker2_state = 0U;
                    sucker3_state = 0U;
                    sucker4_state = 0U;

                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);            /* 舵机初始化位 */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);           /* 夹爪初始化电�? */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);           /* 吸盘1初始化电�? */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);           /* 吸盘2初始化电�? */
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);           /* 吸盘3初始化电�? */
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);           /* 吸盘4初始化电�? */
                    break;
                case remote_control:
                                   Process_Flow_DebugSnapshot();
                                   AppClampHeadCtrl_Run();
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
										   // 直接�?0，刹死！
											DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
											manual_lift_function();
										break;
										case kfs_mode:
											Chassis.Chassis_Stop(&Chassis);
										   // 直接�?0，刹死！
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

		// }
        can1_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        can2_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
		    can3_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3);

		osDelay(3);
    }

}


