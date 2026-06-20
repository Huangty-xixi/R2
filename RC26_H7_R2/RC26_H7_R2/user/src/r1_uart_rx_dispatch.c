/**
 * @file r1_uart_rx_dispatch.c
 * @brief R1 相关 UART 收字节统一分发（USART1/10）
 *
 * === 业务调用链 ===
 * App_Init → R1UartRxDispatch_Start()      // 启动两路UART IT接收
 * bsp_uart错误 → R1UartRxDispatch_ErrorRecover()  // 重置全部RX parser
 * 
 * USART1 IRQ: HAL_UART_RxCpltCallback
 *   → R1LinkZ3CmdLink_OnRxByte(s_rx_byte1)            // EE..FF 指令帧
 * 
 * USART10 IRQ: HAL_UART_RxCpltCallback
 *   → R1Link_OnRxByte(s_rx_byte10)       // 4种帧统一入口
 */
/*---------------------------------------------------------------------
 * 功能：统一管理 USART1/10 的单字节中断接收
 *
 *    USART1  → R1LinkZ3CmdLink_OnRxByte（EE..FF 三区指令 1~5）
 *    USART10 → R1Link_OnRxByte（任务/信令/放三层/STOP）
 *---------------------------------------------------------------------*/
#include "r1_uart_rx_dispatch.h"

#include "r1_link.h"
#include "r1_link_z3_cmd_link.h"

#include "usart.h"

static uint8_t s_rx_byte1;
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
    r1_uart_start_rx_it(&huart10, &s_rx_byte10);
}

void R1UartRxDispatch_ErrorRecover(void)
{
    R1Link_ErrorRecover();
    R1LinkZ3CmdLink_ErrorRecover();
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
        R1LinkZ3CmdLink_OnRxByte(s_rx_byte1);
        (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte1, 1U);
    }
}
