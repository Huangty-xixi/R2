#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"

//Weapon_mode weapon_mode;
Control_mode control_mode;
Remote_mode remote_mode;
R2_lift_mode r2_lift_mode;

// ������С/��󲦸ˡ�����Ϊ 0/1���м�ֵ��Լ 992����Ϊ�Ƿ�
static uint8_t rc_bit_minmax_decode(uint16_t ch_val)
{
    if (ch_val <= 500u) return 0u;
    if (ch_val >= 1500u) return 1u;
    return 2u; // �м�ֵ����Ч/����Ϊ����С�����
}


void Motion_Task(void const * argument)
{

  for(;;)
  {
		uint8_t ch6_bit = rc_bit_minmax_decode(RCctrl.CH6); // 0=��С, 1=���
		uint8_t ch7_bit = rc_bit_minmax_decode(RCctrl.CH7); // 0=��С, 1=���
		uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);
	
		
		
		
		
		
		if(RCctrl.CH8 < 500)
		{
			control_mode  = master_control;
		}
		
		else if(RCctrl.CH8 > 500 && RCctrl.CH8 < 1500)
		{
			control_mode  = part_remote_control;
		}
		
		else
		{
			control_mode  = remote_control;
		}
		
		
		
		
		switch(control_mode)
			{
						
				case remote_control:
				

          // ͨ�� CH6/CH7 �� 00/01/10/11 ����ѡ����ģʽ
          // 00�����̣�01��������10��̧����11��KFS

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

