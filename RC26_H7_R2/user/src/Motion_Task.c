#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"
#include "usbd_cdc_if.h"

//Weapon_mode weapon_mode;
Control_mode control_mode;
Remote_mode remote_mode;
Master_mode master_mode;
uint8_t master_chassis_action_bits_0;
uint8_t master_chassis_action_bits_1;
uint8_t master_weapon_action_bits;
uint8_t master_lift_action_bits;
uint8_t master_kfs_action_bits_0;
uint8_t master_kfs_action_bits_1;
R2_lift_mode r2_lift_mode;

static uint8_t rc_bit_minmax_decode(uint16_t ch_val)
{
    if (ch_val <= 500u) return 0u;
    if (ch_val >= 1500u) return 1u;
    return 2u;}


void Motion_Task(void const * argument)
{

  for(;;)
  {
		uint8_t ch6_bit = rc_bit_minmax_decode(RCctrl.CH6); 
		uint8_t ch7_bit = rc_bit_minmax_decode(RCctrl.CH7);
		uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);
	
		
		
		
		
		
		if(RCctrl.CH8 < 500)
		{
			uint8_t i = 0U;
			/* 此档位定义为急停模式 */
			control_mode  = part_remote_control;
			master_mode = master_none;

			/* 清空各模块动作字节 */
			master_chassis_action_bits_0 = 0U;
			master_chassis_action_bits_1 = 0U;
			master_weapon_action_bits = 0U;
			master_lift_action_bits = 0U;
			master_kfs_action_bits_0 = 0U;
			master_kfs_action_bits_1 = 0U;

			/* 清空USB数据区（20字节） */
			for (i = 0U; i < 20U; i++)
			{
				usb_last_packet_data[i] = 0U;
			}
			usb_last_packet_valid = 0U;
		}
		
		else if(RCctrl.CH8 > 500 && RCctrl.CH8 < 1500)
		{
			control_mode  = master_control;
		}
		
		else
		{
			control_mode  = remote_control;

		}
		
		
		
		
		switch(control_mode)
			{
						
				case remote_control:
//00;底盘 01;武器 10;抬升 11;kfs
          if ((ch6_bit <= 1u) && (ch7_bit <= 1u))
          {
              switch (mode_code)
              {
                  case 0u: // 00
                      remote_mode = chassis_mode;
                      break;
                  case 1u: // 01
											remote_mode = weapon_mode;
                      break;
                  case 2u: // 10
											remote_mode = lift_mode;
                      break;
                  case 3u: // 11
											remote_mode = kfs_mode;
                      break;
                  default:
                      break;
              }
          }
					break;
					
				case part_remote_control:
					break;
				case master_control:
          /* 主控模式：使用USB数据包data[0]的位控制任务状态机
           * bit0=底盘 bit1=武器 bit2=抬升 bit3=kfs
           * 多个位同时为1时按底盘>武器>抬升>kfs优先级取一个
           */
          if (usb_last_packet_valid != 0U)
          {
            uint8_t master_bits = usb_last_packet_data[0];
            /* 主控帧仅使用三个字节：
             * data[0]：模式选择（bit0底盘 bit1武器 bit2抬升 bit3kfs）
             * data[1]：模式内部动作字节0（各模式含义不同）
             * data[2]：模式内部动作字节1（仅部分模式需要）
             *
             * 根据 master_mode 决定 data[1]/data[2] 分别填到哪个动作变量里。
             */

            /* 先清空，避免上一帧残留到当前模式 */
            master_chassis_action_bits_0 = 0U;
            master_chassis_action_bits_1 = 0U;
            master_weapon_action_bits = 0U;
            master_lift_action_bits = 0U;
            master_kfs_action_bits_0 = 0U;
            master_kfs_action_bits_1 = 0U;

            if ((master_bits & 0x01U) != 0U)
            {
              master_mode = master_chassis_mode;
              master_chassis_action_bits_0 = usb_last_packet_data[1];
              master_chassis_action_bits_1 = usb_last_packet_data[2];
            }
            else if ((master_bits & 0x02U) != 0U)
            {
              master_mode = master_weapon_mode;
              master_weapon_action_bits = usb_last_packet_data[1];
            }
            else if ((master_bits & 0x04U) != 0U)
            {
              master_mode = master_lift_mode;
              master_lift_action_bits = usb_last_packet_data[1];
            }
            else if ((master_bits & 0x08U) != 0U)
            {
              master_mode = master_kfs_mode;
              master_kfs_action_bits_0 = usb_last_packet_data[1];
              master_kfs_action_bits_1 = usb_last_packet_data[2];
            }
            else
            {
              master_mode = master_none;
            }
          }
					break;
			}
		
		}
      
      
    osDelay(1);

  }

