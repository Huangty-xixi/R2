/**
 * @file r1_uart_rx_dispatch.c
 * @brief R1 相关 UART 收字节统一分发（USART1/3/10）
 */

#include "r1_uart_rx_dispatch.h"

#include "r1_link.h"
#include "r1_usart1_link.h"
#include "r1_usart3_link.h"

#include "usart.h"

static uint8_t s_rx_byte1;
static uint8_t s_rx_byte3;
static uint8_t s_rx_byte10;

static void r1_uart_start_rx_it(UART_HandleTypeDef *huart, uint8_t *rx_byte)
{
    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    (void)HAL_UART_Receive_IT(huart, rx_byte, 1U);
}

void R1UartRxDispatch_Start(void)
{
    r1_uart_start_rx_it(&huart1, &s_rx_byte1);
    r1_uart_start_rx_it(&huart3, &s_rx_byte3);
    r1_uart_start_rx_it(&huart10, &s_rx_byte10);
}

void R1UartRxDispatch_ErrorRecover(void)
{
    R1Link_ErrorRecover();
    R1Usart1Link_ErrorRecover();
    R1Usart3Link_ErrorRecover();
    R1UartRxDispatch_Start();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    if (huart == &huart10)
    {
        R1Link_OnRxByte(s_rx_byte10);   
        (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte10, 1U);  
    }
    else if (huart == &huart1)
    {
        R1Usart1Link_OnRxByte(s_rx_byte1);
        (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte1, 1U);
    }
    else if (huart == &huart3)
    {
        R1Usart3Link_OnRxByte(s_rx_byte3);
        (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte3, 1U);
    }
}
