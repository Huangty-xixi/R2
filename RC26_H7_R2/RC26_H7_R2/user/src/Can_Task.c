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
#include "app_zone1.h"
#include "clamp_head_ctrl.h"
#include "yaw_heading_ctrl.h"
#include "tim.h"
#include "remote_control.h"
#include "usart.h"

volatile uint32_t g_can1_tx_fifo_min_free = 4U;
volatile uint32_t g_can2_tx_fifo_min_free = 4U;
volatile uint32_t g_can3_tx_fifo_min_free = 4U;

void Can_Task(void const * argument)
{
    uint32_t can1_free_level = 0;
    uint32_t can2_free_level = 0;
    uint32_t can3_free_level = 0;
    uint8_t app_zone1_inited = 0U;
    uint8_t clamp_head_inited = 0U;
    uint8_t yaw_heading_inited = 0U;
   
    for(;;)
    {
        if (app_zone1_inited == 0U)
        {
            AppZone1_Init();
            app_zone1_inited = 1U;
        }
        if (clamp_head_inited == 0U)
        {
            ClampHeadCtrl_Init();
            clamp_head_inited = 1U;
        }
        if (yaw_heading_inited == 0U)
        {
            YawHeadingCtrl_Init();
            yaw_heading_inited = 1U;
        }

        RemoteControl_LinkWatchdog_SimpleTest(&RCctrl);
#if REMOTE_LOST_PROTECT_ENABLE
        RemoteControl_LinkWatchdog_Update(&RCctrl);

        if (RCctrl.rc_lost != false)
        {
            /* 遥控链路丢失：全车电机立即关闭输出 */
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
            /* 过温保护：全车电机输出清零并退出本调度周期业务控制 */
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
                case full_auto_control:
                    /* flow_mode: CH5 低=取KFS/高=下台阶；一区/二区在 Motion_Task（CH7/CH6 高档） */
                    switch (flow_mode)
                    {
                        case flow_get_kfs_mode:
                            Process_GetKFS(APP_ZONE2_GET_KFS_HIGH_TO_LOW);
                            break;
                        case flow_put_kfs_mode:
                            Process_PutKFS();
                            break;
                        case flow_upstairs_mode:
                            Process_UpStairs();
                            break;
                        case flow_upslope_mode:
                            Process_UpSlope();
                            break;
                        case flow_downstairs_mode:
                            Process_DownStairs();
                            break;
                        case flow_none:
                        default:
                            break;
                    }
                    Process_Flow_DebugSnapshot();
                    /* 全自动档下保持底盘手动：CH1~CH4 与遥控模式一致 */
                    manual_chassis_function();
                    manual_weapon_function();
                    manual_lift_function();
                    manual_kfs_function();
                    break;
                case emergency_stop_mode:
                    /* 急停：清流程覆盖，底盘三轴指令 0 经 PID 制动；其余轴仍直接清零 */
                    Process_Flow_ClearChassisOverride();
                    Chassis_EmergencyBrakeRun(&Chassis);
                    DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

                    /* DM 电机（MIT）清零：kp/kd/扭矩全部置 0 */
                    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.9f, 0.3f, 0.4f, 0.0f);
                    three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                    /* 急停时将 weapon 相关执行机构退回到初始化姿态 */
                    main_lift_position = main_lift_p0;
                    servo_state = 1U;
                    clamp_state = 0U;
                    sucker1_state = 0U;
                    sucker2_state = 0U;
                    sucker3_state = 0U;
                    sucker4_state = 0U;

                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);            /* 舵机初始化位置 */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);           /* 夹爪初始化电平 */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);           /* 吸盘 1 初始化电平 */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);           /* 吸盘 2 初始化电平 */
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);           /* 吸盘 3 初始化电平 */
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);           /* 吸盘 4 初始化电平 */
                    break;
                case remote_control:
                                   Process_Flow_DebugSnapshot();
                                   
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
										   // 直接给 0，底盘停止
											DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
											manual_lift_function();
										break;
										case kfs_mode:
											Chassis.Chassis_Stop(&Chassis);
										   // 直接给 0，底盘停止
											DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
											manual_kfs_function();
										break;
										case remote_none:
										break;
									}

					/* 遥控非抬升模式：继续跑抬升 DM 输出与到位检测，但不读 CH3/CH4（状态机仅在 lift_mode 内切换） */
					if (remote_mode != lift_mode)
					{
						lift_motor_run_output();
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

        if (can1_free_level < g_can1_tx_fifo_min_free) g_can1_tx_fifo_min_free = can1_free_level;
        if (can2_free_level < g_can2_tx_fifo_min_free) g_can2_tx_fifo_min_free = can2_free_level;
        if (can3_free_level < g_can3_tx_fifo_min_free) g_can3_tx_fifo_min_free = can3_free_level;
		/* TX FIFO stuck: free_level==0 for 100 cycles (~300ms) means a frame is stuck, reset CAN */
		{
			static uint32_t s_can1_stuck_cnt = 0;
			static uint32_t s_can2_stuck_cnt = 0;
			static uint32_t s_can3_stuck_cnt = 0;

			if (can1_free_level == 0) {
				if (++s_can1_stuck_cnt > 100) {
					HAL_FDCAN_Stop(&hfdcan1);
					HAL_FDCAN_Start(&hfdcan1);
					s_can1_stuck_cnt = 0;
				}
			} else s_can1_stuck_cnt = 0;

			if (can2_free_level == 0) {
				if (++s_can2_stuck_cnt > 100) {
					HAL_FDCAN_Stop(&hfdcan2);
					HAL_FDCAN_Start(&hfdcan2);
					s_can2_stuck_cnt = 0;
				}
			} else s_can2_stuck_cnt = 0;

			if (can3_free_level == 0) {
				if (++s_can3_stuck_cnt > 100) {
					HAL_FDCAN_Stop(&hfdcan3);
					HAL_FDCAN_Start(&hfdcan3);
					s_can3_stuck_cnt = 0;
				}
			} else s_can3_stuck_cnt = 0;
		}

		/* DM 电机健康检查：状态为 OFF 则重新使能 */
		{
			static uint32_t s_dm_check_tick = 0;
			if (++s_dm_check_tick >= 33)
			{
				s_dm_check_tick = 0;
				if (R2_lift_motor_left.state == OFF)
					R2_lift_motor_left.send_cmd(&R2_lift_motor_left, Motor_Enable);
				if (R2_lift_motor_right.state == OFF)
					R2_lift_motor_right.send_cmd(&R2_lift_motor_right, Motor_Enable);
				if (main_lift.state == OFF)
					main_lift.send_cmd(&main_lift, Motor_Enable);
				if (kfs_spin.state == OFF)
					kfs_spin.send_cmd(&kfs_spin, Motor_Enable);
				if (three_kfs.state == OFF)
					three_kfs.send_cmd(&three_kfs, Motor_Enable);
			}
		}

		osDelay(3);
    }

}


