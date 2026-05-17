#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include <stdint.h>
#include "usart.h"
#include "dma.h"
#include "remote_control.h"

/** USART2(HI14 IMU) 接收与调试状态；volatile 只加在实例 g_imu_uart_ctx 上，避免成员重复修饰 */
typedef struct
{
    uint8_t  rx_ready;
    uint16_t rx_size;
    uint32_t start_rx_ret;
    uint32_t start_rx_cnt;
    uint32_t start_rx_busy_cnt;
    uint32_t rx_event_cnt;
    uint32_t uart2_gstate_dbg;
    uint32_t uart2_rxstate_dbg;
    uint32_t uart2_isr_dbg;
    uint32_t uart2_err_dbg;
    uint8_t rx_buf[53];
} bsp_imu_uart_ctx_t;

void BSP_USART_Init(void);
void BSP_SBUS_RecoverPoll(void);
void BSP_USART2_StartRxIT(void);
void BSP_USART2_DE(uint8_t en);

extern volatile bsp_imu_uart_ctx_t g_imu_uart_ctx;

#endif
