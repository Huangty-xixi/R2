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
#include "app_yaw_heading_ctrl.h"
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
    uint8_t app_yaw_heading_inited = 0U;
   
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
        if (app_yaw_heading_inited == 0U)
        {
            AppYawHeadingCtrl_Init();
            app_yaw_heading_inited = 1U;
        }

        RemoteControl_LinkWatchdog_SimpleTest(&RCctrl);
#if REMOTE_LOST_PROTECT_ENABLE
        RemoteControl_LinkWatchdog_Update(&RCctrl);

        if (RCctrl.rc_lost != false)
        {
            /* Ò£¿ØÁ´Â·¶ªÊ§£ºÈ«³µµç»úÁ¢¼´¹Ø±ÕÊä³ö */
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
            /* ¹ýÎÂ±£»¤£ºÈ«³µµç»úÊä³öÇåÁã²¢ÍË³ö±¾µ÷¶ÈÖÜÆÚÒµÎñ¿ØÖÆ */
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
<<<<<<< HEAD
                case full_auto_control:
                    /* È«×Ô¶¯£ºget/put/ÉÏÆÂ ÔÚ´ËÅÜ£»¶þÇø½ö Motion_Task ÔÚ CH6 ×î´óÊ± app_zone2_poll£¨ÓëÖÐÎ»ÔÝÍ£Ò»ÖÂ£© */
                    switch (full_auto_mode)
                    {
                        case full_auto_get_kfs_mode:
                            Process_GetKFS();
                            break;
                        case full_auto_put_kfs_mode:
                            Process_PutKFS();
                            break;
                        case full_auto_upslope_mode:
                            Process_UpSlope();
                            break;
                        case full_auto_upstairs_mode:
                        case full_auto_downstairs_mode:
                        case full_auto_face_field_dir_mode:
                        case full_auto_zone2_mode:
                        case full_auto_none:
                        default:
                            break;
                    }
                    Process_Flow_DebugSnapshot();
                    /* È«×Ô¶¯µµÏÂ±£³Öµ×ÅÌÊÖ¶¯£ºCH1~CH4 ÓëÒ£¿ØÄ£Ê½Ò»ÖÂ */
=======
                case semi_auto_control:
                    AppFlowDispatch_Run();
                    Process_Flow_DebugSnapshot();
                    /* °ë×Ô¶¯Ä£Ê½ÏÂ±£³Öµ×ÅÌÊÖ¶¯£ºCH1~CH4 ÓëÊÖ¿ØÄ£Ê½Ò»ÖÂ */
>>>>>>> origin/1åŒº
                    manual_chassis_function();
                    AppClampHeadCtrl_Run();
                    manual_weapon_function();
                    manual_lift_function();
                    manual_kfs_function();
                    break;
                case emergency_stop_mode:
                    /* ¼±Í£Ä£Ê½£ºÖ÷¿Ø¹Ø±ÕËùÓÐÊä³ö£¬·ÀÖ¹²ÐÓàÖ¸Áî¼ÌÐøÇý¶¯ */
                    Chassis.Chassis_Stop(&Chassis);
                    DJIset_motor_data(&hfdcan1, 0X200, 0, 0, 0, 0);
                    DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
                    DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

                    /* DM µç»ú£¨MIT£©ÇåÁã£ºkp/kd/Å¤¾ØÈ«²¿ÖÃ 0 */
                    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.9f, 0.3f, 0.4f, 0.0f);
                    three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                    /* ¼±Í£Ê±½« weapon Ïà¹ØÖ´ÐÐ»ú¹¹ÍË»Øµ½³õÊ¼»¯×ËÌ¬ */
                    servo_state = 1U;
                    clamp_state = 0U;
                    sucker1_state = 0U;
                    sucker2_state = 0U;
                    sucker3_state = 0U;
                    sucker4_state = 0U;

                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);            /* ¶æ»ú³õÊ¼»¯Î»ÖÃ */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);           /* ¼Ð×¦³õÊ¼»¯µçÆ½ */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);           /* ÎüÅÌ 1 ³õÊ¼»¯µçÆ½ */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);           /* ÎüÅÌ 2 ³õÊ¼»¯µçÆ½ */
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);           /* ÎüÅÌ 3 ³õÊ¼»¯µçÆ½ */
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);           /* ÎüÅÌ 4 ³õÊ¼»¯µçÆ½ */
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
										   // Ö±½Ó¸ø 0£¬µ×ÅÌÍ£Ö¹
											DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
											manual_lift_function();
										break;
										case kfs_mode:
											Chassis.Chassis_Stop(&Chassis);
										   // Ö±½Ó¸ø 0£¬µ×ÅÌÍ£Ö¹
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


