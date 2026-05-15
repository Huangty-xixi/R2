#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// 函数声明
void USART1_init(u32 baudrate);
void USART2_init(u32 baudrate);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr);
void Processing_Data(uint8_t RXdata);

// 外部变量声明
extern volatile uint8_t data_ready;
extern volatile uint16_t distance_value;
extern volatile uint8_t confidence_value;

#endif
