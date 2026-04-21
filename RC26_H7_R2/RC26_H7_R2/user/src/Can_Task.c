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
#include "bsp_uart.h"
#include "usart.h"

volatile float g_imu_acc_x_g = 0.0f;
volatile float g_imu_acc_y_g = 0.0f;
volatile float g_imu_acc_z_g = 0.0f;
volatile float g_imu_gyr_x_dps = 0.0f;
volatile float g_imu_gyr_y_dps = 0.0f;
volatile float g_imu_gyr_z_dps = 0.0f;
volatile float g_imu_mag_x_ut = 0.0f;
volatile float g_imu_mag_y_ut = 0.0f;
volatile float g_imu_mag_z_ut = 0.0f;
volatile float g_imu_roll_deg = 0.0f;
volatile float g_imu_pitch_deg = 0.0f;
volatile float g_imu_yaw_deg = 0.0f;




void Can_Task(void const * argument)
{
    // TickType_t Systick = 0;
    uint32_t can1_free_level = 0;
    uint32_t can2_free_level = 0;
    uint32_t can3_free_level = 0;
    uint32_t imu_last_tick = 0U;
    static const uint8_t req[8] = {0x50U, 0x03U, 0x00U, 0x34U, 0x00U, 0x18U, 0x09U, 0x8FU};
   
    for(;;)
    {
        // Systick = osKernelGetTickCount();

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
		
		// if (RCctrl.rc_lost){
		// 	if(Systick % 2 == 1){	
        //          Chassis.Chassis_Stop(&Chassis);
		// 		DJIset_motor_data(&hfdcan1, 0X200,0,0,0,0);
		// 		DJIset_motor_data(&hfdcan2, 0X200,0,0,0,0);
		// 	}
		// 	if(Systick % 2 == 0){	
			
		// 	}
		// }
		// else{
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
                    kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.9f, 0.3f, 0.4f, 0.0f);
                    three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                    /* 急停时将weapon相关执行器拉回初始化电平 */
                    servo_state = 1U;
                    clamp_state = 0U;
                    sucker1_state = 0U;
                    sucker2_state = 0U;
                    sucker3_state = 0U;
                    sucker4_state = 0U;
//                    pump1_state = 0U;
//                    pump2_state = 0U;

                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);            /* 舵机初始化位 */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);           /* 夹爪初始化电平 */
//                    pump1_two_suckers_linkage_nominal_open(0U, 0U);                /* 吸盘1/2与泵1初始化 */
//                    pump2_two_suckers_linkage_nominal_open(0U, 0U);                /* 吸盘3/4与泵2初始化 */
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

        if ((HAL_GetTick() - imu_last_tick) >= 200U)
        {
            imu_last_tick = HAL_GetTick();
            /* 软件控向：先发后收 */
            BSP_USART2_DE(1U);
            (void)HAL_UART_Transmit(&huart2, (uint8_t *)req, 8U, 30U);
            while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {;}
            BSP_USART2_DE(0U);

            BSP_USART2_StartRxIT();
        }

        if ((g_imu_rx_ready != 0U) && (g_imu_rx_size == 53U) && (g_imu_rx_buf[0] == 0x50U) && (g_imu_rx_buf[1] == 0x03U) && (g_imu_rx_buf[2] == 0x30U))
        {
            int16_t accx, accy, accz;
            int16_t gyrx, gyry, gyrz;
            int16_t magx, magy, magz;
            int32_t roll, pitch, yaw;

            accx = (int16_t)(((uint16_t)g_imu_rx_buf[3] << 8) | g_imu_rx_buf[4]);
            accy = (int16_t)(((uint16_t)g_imu_rx_buf[5] << 8) | g_imu_rx_buf[6]);
            accz = (int16_t)(((uint16_t)g_imu_rx_buf[7] << 8) | g_imu_rx_buf[8]);

            gyrx = (int16_t)(((uint16_t)g_imu_rx_buf[9] << 8) | g_imu_rx_buf[10]);
            gyry = (int16_t)(((uint16_t)g_imu_rx_buf[11] << 8) | g_imu_rx_buf[12]);
            gyrz = (int16_t)(((uint16_t)g_imu_rx_buf[13] << 8) | g_imu_rx_buf[14]);

            magx = (int16_t)(((uint16_t)g_imu_rx_buf[15] << 8) | g_imu_rx_buf[16]);
            magy = (int16_t)(((uint16_t)g_imu_rx_buf[17] << 8) | g_imu_rx_buf[18]);
            magz = (int16_t)(((uint16_t)g_imu_rx_buf[19] << 8) | g_imu_rx_buf[20]);

            roll  = (int32_t)(((uint32_t)g_imu_rx_buf[21] << 24) | ((uint32_t)g_imu_rx_buf[22] << 16) | ((uint32_t)g_imu_rx_buf[23] << 8) | (uint32_t)g_imu_rx_buf[24]);
            pitch = (int32_t)(((uint32_t)g_imu_rx_buf[25] << 24) | ((uint32_t)g_imu_rx_buf[26] << 16) | ((uint32_t)g_imu_rx_buf[27] << 8) | (uint32_t)g_imu_rx_buf[28]);
            yaw   = (int32_t)(((uint32_t)g_imu_rx_buf[29] << 24) | ((uint32_t)g_imu_rx_buf[30] << 16) | ((uint32_t)g_imu_rx_buf[31] << 8) | (uint32_t)g_imu_rx_buf[32]);

            g_imu_acc_x_g = (float)accx * 0.00048828f;
            g_imu_acc_y_g = (float)accy * 0.00048828f;
            g_imu_acc_z_g = (float)accz * 0.00048828f;

            g_imu_gyr_x_dps = (float)gyrx * 0.061035f;
            g_imu_gyr_y_dps = (float)gyry * 0.061035f;
            g_imu_gyr_z_dps = (float)gyrz * 0.061035f;

            g_imu_mag_x_ut = (float)magx * 0.030517f;
            g_imu_mag_y_ut = (float)magy * 0.030517f;
            g_imu_mag_z_ut = (float)magz * 0.030517f;

            g_imu_roll_deg = (float)roll * 0.001f;
            g_imu_pitch_deg = (float)pitch * 0.001f;
            g_imu_yaw_deg = (float)yaw * 0.001f;

            g_imu_rx_ready = 0U;
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


