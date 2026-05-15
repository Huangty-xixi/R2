#include "bsp_uart.h"
#include "ti_msp_dl_config.h"
#include "ti/driverlib/m0p/dl_core.h"



volatile unsigned int delay_times = 0;

void delay_ms(unsigned int ms);


int main(void)
{
		
    SYSCFG_DL_init(); 

	  NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    //使能串口中断 Enable serial port interrupt
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
	
	 printf("Distance: %d mm, Confidence: %d\r\n", 0, 1);
	  
    while (1)
    { 
			
			
        if (data_ready) {

						uint16_t dist = distance_value;
						uint8_t conf = confidence_value;
						   // 发送结果
                printf("Distance: %d mm, Confidence: %d\r\n", dist, conf);
            
            data_ready = 0;  // 清除标志
				}
		}
}
		


//搭配滴答定时器实现的精确ms延时 Accurate ms delay with tick timer
void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while( delay_times != 0 );
}



//滴答定时器的中断服务函数 Tick ??timer interrupt service function
void SysTick_Handler(void)
{
    if( delay_times != 0 )
    {
        delay_times--;
    }
}
 







