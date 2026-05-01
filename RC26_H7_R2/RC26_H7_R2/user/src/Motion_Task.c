#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"
#include "usbd_cdc_if.h"
#include "Process_Flow.h"

//Weapon_mode weapon_mode;
Control_mode control_mode;
Remote_mode remote_mode;
Semi_auto_mode semi_auto_mode;
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

    
		switch(control_mode)
			{
						
				case remote_control:
//00;底盘 01;武器 10;抬升 11;kfs
          Process_Flow_ResetAll();
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
            Process_Flow_ResetAll();
            semi_auto_mode = semi_auto_none;
            semi_auto_trigger_armed = 1U;
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
            /* 仅用于重新上膛，不打断已触发流程 */
            semi_auto_trigger_armed = 1U;
          }
          else if ((semi_auto_mode == semi_auto_none) && (semi_auto_trigger_armed != 0U))
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
              /* 空闲态下无有效单命令：保持 none，不覆盖进行中的流程 */
              semi_auto_mode = semi_auto_none;
            }
          }
          else
          {
            /* 流程进行中：保持当前流程模式，交由 Process_Flow 状态机收尾 */
          }

          }
					break;
			}

    last_control_mode = control_mode;
		
    osDelay(1);
		}

  }

