#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"

//Weapon_mode weapon_mode;
Control_mode control_mode;
Remote_mode remote_mode;

// 手动模式标志位（由 CH6/CH7 00/01/10/11 编码得到）
uint8_t manual_mode_flag = 0;      // CH8 最大：1=手动
uint8_t manual_chassis_flag = 0;  // 00
uint8_t manual_weapon_flag = 0;   // 01
uint8_t manual_lift_flag = 0;     // 10（占位）
uint8_t manual_kfs_flag = 0;      // 11（占位）

// 将“最小/最大拨杆”解码为 0/1；中间值（约 992）判为非法
static uint8_t rc_bit_minmax_decode(uint16_t ch_val)
{
    if (ch_val <= 500u) return 0u;
    if (ch_val >= 1500u) return 1u;
    return 2u; // 中间值：无效/不认为是最小或最大
}


void Motion_Task(void const * argument)
{

  for(;;)
  {
					uint8_t ch6_bit = rc_bit_minmax_decode(RCctrl.CH6); // 0=最小, 1=最大
          uint8_t ch7_bit = rc_bit_minmax_decode(RCctrl.CH7); // 0=最小, 1=最大
					uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);
		
		
		switch(control_mode)
			{
						
				case remote_control:
				

          // 通过 CH6/CH7 的 00/01/10/11 编码选择子模式
          // 00：底盘；01：武器；10：抬升；11：KFS

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
					break;
			}
		
		}
      
      
    osDelay(1);
  }

