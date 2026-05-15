#include "stm32f10x.h"
#include "delay.h"
#include "bsp_usart.h"
#include <stdio.h>

extern uint8_t data_buff[5]; //备份buf Backup buf

int main()
{
	
	SystemInit();
	
	delay_init();
	
	USART1_init(115200);//接PC的串口 Connect to PC's serial port
	USART2_init(115200);  //接测距模块PA2(TX) PA3(RX) Connect the distance measurement modules PA2(TX) and PA3(RX)
  delay_ms(100);

while(1) {
		//接收到数据就做打印处理 Upon receiving the data, print it out
    if(data_ready) {
        uint16_t dist = distance_value;
        uint8_t conf = confidence_value;
        data_ready = 0;
        printf("Distance:%dmm, Confidence:%d\r\n", dist, conf);
    }
}
}

