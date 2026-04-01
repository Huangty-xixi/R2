#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"

Weapon_mode weapon_mode;
Control_mode control_mode;
Remote_mode remote_mode;


void Motion_Task(void const * argument)
{

  for(;;)
  {
      if(RCctrl.CH6 < 1000)
      {
          control_mode = master_control;
          remote_mode = remote_none;
      }
      else
      {
          Chassis.Chassis_Calc(&Chassis);
          control_mode = remote_control;
          if(RCctrl.CH7 < 1000)
          {
              remote_mode = chassis_move;
              weapon_mode = weapon_none ;
						
          }
          else 
          {
              remote_mode = weapon_switch;
              if(RCctrl.CH8 < 500)
              {
                weapon_mode=steering_mode ;              
							}

              else
              {
								weapon_mode=pump_mode ;
              }
          }

      }
      
      
      
      
      
      
    osDelay(1);
  }

}