#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"
#include "dma.h"
#include "remote_control.h"


void BSP_USART_Init(void);
void BSP_USART2_StartRxIT(void);
void BSP_USART2_DE(uint8_t en);
extern volatile uint8_t g_imu_rx_ready;
extern volatile uint16_t g_imu_rx_size;
extern volatile uint32_t g_imu_start_rx_ret;
extern volatile uint32_t g_imu_start_rx_cnt;
extern volatile uint32_t g_imu_start_rx_busy_cnt;
extern volatile uint32_t g_imu_rx_event_cnt;
extern volatile uint32_t g_imu_uart2_gstate_dbg;
extern volatile uint32_t g_imu_uart2_rxstate_dbg;
extern volatile uint32_t g_imu_uart2_isr_dbg;
extern volatile uint32_t g_imu_uart2_err_dbg;
extern uint8_t g_imu_rx_buf[53];

#endif
