#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"
#include "usbd_cdc_if.h"

//Weapon_mode weapon_mode;
Control_mode control_mode;
Remote_mode remote_mode;
Semi_auto_mode semi_auto_mode;
// Master_mode master_mode;
// uint8_t master_enable_bits;
// uint8_t master_chassis_action_bits_0;
// uint8_t master_chassis_action_bits_1;
// uint8_t master_weapon_action_bits;
// uint8_t master_lift_action_bits;
// uint8_t master_kfs_action_bits_0;
// uint8_t master_kfs_action_bits_1;
R2_lift_mode r2_lift_mode;
static uint8_t semi_auto_ready_pending = 0U;
static uint8_t semi_auto_ready_msg[] = "ready\r\n";
static uint8_t semi_auto_trigger_armed = 1U;//半自动模式下，触发器是否允许触发

static uint8_t rc_bit_minmax_decode(uint16_t ch_val)
{
    if (ch_val <= 500u) return 0u;
    if (ch_val >= 1500u) return 1u;
    return 2u;}


void Motion_Task(void const * argument)
{
  Control_mode last_control_mode = emergency_stop_mode;

  for(;;)
  {
		
		// 读取 PE0 引脚状态
        GPIO_PinState pe0_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
//        
//        // 根据引脚状态执行相应操作
//        if(pe0_state == GPIO_PIN_SET)
//        {
//            // PE0 为高电平，执行相应操作
//        }
//        else
//        {
//            // PE0 为低电平，执行相应操作
//        }
//        
				
		uint8_t ch6_bit = rc_bit_minmax_decode(RCctrl.CH6); 
		uint8_t ch7_bit = rc_bit_minmax_decode(RCctrl.CH7);
		uint8_t ch5_bit = rc_bit_minmax_decode(RCctrl.CH5);
		uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);
	
		
		
		
		
		
		if(RCctrl.CH8 < 500)
		{
      control_mode  = emergency_stop_mode;
		}
		
		else if(RCctrl.CH8 > 500 && RCctrl.CH8 < 1500)
		{
			control_mode  = semi_auto_control;
		}
		
		else
		{
			control_mode  = remote_control;

		}

    /* 切换到主控模式时，给上位机回传一次 ready */
    if ((control_mode == semi_auto_control) && (last_control_mode != semi_auto_control))
    {
      semi_auto_ready_pending = 1U;
    }
    if (semi_auto_ready_pending != 0U)
    {
      if (CDC_Transmit_HS(semi_auto_ready_msg, (uint16_t)(sizeof(semi_auto_ready_msg) - 1U)) == USBD_OK)
      {
        semi_auto_ready_pending = 0U;
      }
    }
		
		
		
		
		switch(control_mode)
			{
						
				case remote_control:
//00;底盘 01;武器 10;抬升 11;kfs
          semi_auto_mode = semi_auto_none;
          semi_auto_trigger_armed = 1U;
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
					
				case emergency_stop_mode:
          {
            uint8_t i = 0U;
            semi_auto_mode = semi_auto_none;
            semi_auto_trigger_armed = 1U;
            /* 此档位定义为急停模式 */
            // master_mode = master_none;
            // master_enable_bits = 0U;

            /* 清空各模块动作字节 */
            // master_chassis_action_bits_0 = 0U;
            // master_chassis_action_bits_1 = 0U;
            // master_weapon_action_bits = 0U;
            // master_lift_action_bits = 0U;
            // master_kfs_action_bits_0 = 0U;
            // master_kfs_action_bits_1 = 0U;

            /* 清空USB数据区（20字节） */
            for (i = 0U; i < 20U; i++)
            {
              usb_last_packet_data[i] = 0U;
            }
            usb_last_packet_valid = 0U;
					break;
          }
				case semi_auto_control:
        {
          uint8_t cmd_count = 0U;
          uint8_t ch5_upstairs_req = (uint8_t)(ch5_bit == 0u);
          uint8_t ch5_downstairs_req = (uint8_t)(ch5_bit == 1u);
          uint8_t ch6_get_kfs_req = (uint8_t)(ch6_bit == 0u);
          uint8_t ch7_put_kfs_req = (uint8_t)(ch7_bit == 0u);

          /* 半自动模式下，CH1~CH4 仍按底盘手动控制 */
          remote_mode = chassis_mode;

          /* 防误触：
           * 1) CH5 回中位 + CH6/CH7 处于释放位（最大值）后，才重新允许触发；
           * 2) 一次只允许一个流程触发，多拨杆同时触发则忽略。
           */
          if ((ch5_bit == 2u) && (ch6_bit == 1u) && (ch7_bit == 1u))
          {
            semi_auto_mode = semi_auto_none;
            semi_auto_trigger_armed = 1U;
          }
          else if (semi_auto_trigger_armed != 0U)
          {
            cmd_count = (uint8_t)(ch5_upstairs_req + ch5_downstairs_req + ch6_get_kfs_req + ch7_put_kfs_req);
            if (cmd_count == 1U)
            {
              if (ch5_downstairs_req != 0U)
              {
                semi_auto_mode = semi_auto_downstairs_mode;
              }
              else if (ch5_upstairs_req != 0U)
              {
                semi_auto_mode = semi_auto_upstairs_mode;
              }
              else if (ch6_get_kfs_req != 0U)
              {
                semi_auto_mode = semi_auto_get_kfs_mode;
              }
              else
              {
                semi_auto_mode = semi_auto_put_kfs_mode;
              }
              semi_auto_trigger_armed = 0U;
            }
            else
            {
              semi_auto_mode = semi_auto_none;
            }
          }
          else
          {
            semi_auto_mode = semi_auto_none;
          }

          /* 主控模式（并行）：
           * data[0]：bit0~bit3 作为各子系统使能位，可同时置位
           * data[1]：底盘动作字节0
           * data[2]：底盘动作字节1
           * data[3]：武器动作字节
           * data[4]：抬升动作字节
           * data[5]：KFS动作字节0
           * data[6]：KFS动作字节1
           */
          // if (usb_last_packet_valid != 0U)
          // {
            // master_enable_bits = (uint8_t)(usb_last_packet_data[0] & 0x0FU);
            // master_chassis_action_bits_0 = usb_last_packet_data[1];
            // master_chassis_action_bits_1 = usb_last_packet_data[2];
            // master_weapon_action_bits = usb_last_packet_data[3];
            // master_lift_action_bits = usb_last_packet_data[4];
            // master_kfs_action_bits_0 = usb_last_packet_data[5];
            // master_kfs_action_bits_1 = usb_last_packet_data[6];

            /* 兼容变量：仅用于少量旧逻辑观测，不参与并行调度 */
            // if ((master_enable_bits & MASTER_EN_CHASSIS) != 0U)
            // {
            //   master_mode = master_chassis_mode;
            // }
            // else if ((master_enable_bits & MASTER_EN_WEAPON) != 0U)
            // {
            //   master_mode = master_weapon_mode;
            // }
            // else if ((master_enable_bits & MASTER_EN_LIFT) != 0U)
            // {
            //   master_mode = master_lift_mode;
            // }
            // else if ((master_enable_bits & MASTER_EN_KFS) != 0U)
            // {
            //   master_mode = master_kfs_mode;
            // }
            // else
            // {
            //   master_mode = master_none;
            // }
          }
					break;
			}

    last_control_mode = control_mode;
		
		
      
      
    osDelay(1);
		}

  }

