#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"
#include "Process_Flow.h"
#include "app_zone2.h"

//Weapon_mode weapon_mode;
Control_mode control_mode;
Remote_mode remote_mode;
Full_auto_mode full_auto_mode;

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
		uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);
		
		if(RCctrl.CH8 < 500)
		{
      control_mode  = emergency_stop_mode;
		}
		
		else if(RCctrl.CH8 > 500 && RCctrl.CH8 < 1500)
		{
			control_mode  = full_auto_control;
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
          full_auto_mode = full_auto_none;
          app_zone2_mission_clear();
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
            Process_Flow_ResetAll();
            full_auto_mode = full_auto_none;
            app_zone2_mission_clear();
					break;
          }

          
				case full_auto_control:
        {
          uint8_t ch5_bit = rc_bit_minmax_decode(RCctrl.CH5);
          uint8_t r_put = (uint8_t)(ch5_bit == 0u);  /* CH5 最小 → 放 KFS */
          uint8_t r_up  = (uint8_t)(ch5_bit == 1u);  /* CH5 最大 → 上坡 */
          uint8_t r_get = (uint8_t)(ch7_bit == 1u); /* CH7 最大 → 取 KFS */
          uint8_t r_z2  = (uint8_t)(ch6_bit == 1u); /* CH6 最大 → 二区 */
          uint8_t cmd_count;

          remote_mode = chassis_mode;

          /* CH6 最小：中断二区、复位任务；退出二区独占 */
          if (ch6_bit == 0u)
          {
            app_zone2_mission_clear();
            if (full_auto_mode == full_auto_zone2_mode)
              full_auto_mode = full_auto_none;
          }
          else if (full_auto_mode == full_auto_zone2_mode)
          {
            /* 仅 CH6 最大时轮询二区；中位暂停、不 abort */
            if (ch6_bit == 1u)
            {
              app_zone2_poll();
              if (app_zone2_is_done() != 0U)
                full_auto_mode = full_auto_none;
            }
          }
          else if (full_auto_mode == full_auto_none)
          {
            /* 四路请求互斥：恰好一个才进入对应流程 */
            cmd_count = (uint8_t)(r_z2 + r_put + r_up + r_get);
            if (cmd_count == 1u)
            {
              if (r_put != 0u)
                full_auto_mode = full_auto_put_kfs_mode;
              else if (r_up != 0u)
                full_auto_mode = full_auto_upslope_mode;
              else if (r_get != 0u)
                full_auto_mode = full_auto_get_kfs_mode;
              else
              {
                full_auto_mode = full_auto_zone2_mode;
                app_zone2_poll();
              }
            }
          }
          break;
        }
			}

    last_control_mode = control_mode;
		
    osDelay(1);
		}

  }

