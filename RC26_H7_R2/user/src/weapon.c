#include "weapon.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"

Weapon_Module Weapon;



uint8_t CH5_trigger_flag1 = 0; //舵机
uint8_t CH5_trigger_flag0 = 0; //气缸

int test =0;


void streeing_motion ()//舵机
{

		if(CH5_trigger_flag1 %2==0)
     {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,test);
     }
	  if(CH5_trigger_flag1 %2==1)
     {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,test);
     }
	}
	
void pump_motion()//气泵
{
		if(CH5_trigger_flag0 %2==0)
     {
       HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
     }
	  if(CH5_trigger_flag0 %2==1)
     {
       HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
     }
}
