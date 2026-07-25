#include "bsp_uart.h"
#include "usart.h"
#include "gpio.h"
#include "sensor.h"
#include "r1_uart_rx_dispatch.h"

volatile bsp_imu_uart_ctx_t g_imu_uart_ctx = {0};
static volatile uint8_t s_uart9_rx_restart_req = 0U;

void BSP_USART2_DE(uint8_t en)
{
    /* 软件控制 RS485 方向：DM-MC02 板 PD4 = USART2_DE */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, (en != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  启动 UART 接收 DMA 双缓冲（配合 IDLE 中断）
 * @details 配置为 IDLE 空闲中断接收；DMA 传完(TC)或检测到 IDLE 时进回调，适合不定长数据
 * @param  huart            UART 句柄
 * @param  SrcAddress       DMA 源地址，一般为 &huart->Instance->RDR
 * @param  DstAddress       缓冲 0 目标地址
 * @param  SecondMemAddress 缓冲 1 目标地址
 * @param  DataLength       单缓冲长度（字节）
 */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType = HAL_UART_RXEVENT_TC;
    huart->RxXferSize = DataLength;

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    HAL_DMAEx_MultiBufferStart(&hdma_uart9_rx, (uint32_t)SrcAddress, (uint32_t)DstAddress, (uint32_t)SecondMemAddress, DataLength);
}

/* 解析 SBUS 帧并更新遥控数据 */
static void BSP_SBUS_OnUartRx(uint16_t size, uint8_t *buf)
{
    const uint8_t *frame = buf;
    uint16_t offset = 0U;

    if (buf == NULL)
    {
        return;
    }

    if (size < RC_FRAME_LENGTH)
    {
        g_rc_link_dbg.frame_reject++;
        return;
    }

    if (size > RC_FRAME_LENGTH)
    {
        offset = (uint16_t)(size - RC_FRAME_LENGTH);
        frame = buf + offset;
    }

    if (frame[0] != 0x0FU || frame[24] != 0x00U)
    {
        g_rc_link_dbg.frame_reject++;
        return;
    }

    SBUS_TO_RC(frame, &RCctrl);
    g_rc_link_dbg.frame_ok++;
}

/** SBUS 接收异常后，在主循环中重启 UART9 DMA 接收 */
void BSP_SBUS_RecoverPoll(void)
{
    if (s_uart9_rx_restart_req == 0U)
    {
        return;
    }
    s_uart9_rx_restart_req = 0U;
    __HAL_UART_CLEAR_FLAG(&huart9, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    (void)HAL_UART_AbortReceive(&huart9);
    BSP_USART_Init();
}

/** 初始化遥控链路，启动 UART9 SBUS DMA 双缓冲接收 */
void BSP_USART_Init(void){
	RemoteControl_Link_Init();
	USART_RxDMA_MultiBufferStart(&huart9,
                                 (uint32_t *)&(huart9.Instance->RDR),
                                 (uint32_t *)SBUS_MultiRx_Buf[0],
                                 (uint32_t *)SBUS_MultiRx_Buf[1],
                                 SBUS_RX_BUF_NUM);
}

/** 启动 USART2（IMU/RS485）ReceiveToIdle 中断接收 */
void BSP_USART2_StartRxIT(void)
{
    g_imu_uart_ctx.rx_ready = 0U;
    g_imu_uart_ctx.rx_size = 0U;
    g_imu_uart_ctx.start_rx_cnt++;
    g_imu_uart_ctx.uart2_gstate_dbg = (uint32_t)huart2.gState;
    g_imu_uart_ctx.uart2_rxstate_dbg = (uint32_t)huart2.RxState;
    g_imu_uart_ctx.uart2_isr_dbg = huart2.Instance->ISR;
    g_imu_uart_ctx.uart2_err_dbg = (uint32_t)huart2.ErrorCode;
    (void)HAL_UART_AbortReceive(&huart2);
    g_imu_uart_ctx.start_rx_ret =
        (uint32_t)HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)g_imu_uart_ctx.rx_buf, sizeof(g_imu_uart_ctx.rx_buf));
    if (g_imu_uart_ctx.start_rx_ret == 2U)
    {
        g_imu_uart_ctx.start_rx_busy_cnt++;
    }
}

/* UART 接收事件：UART9 处理 SBUS 双缓冲切换，UART2 通知 IMU 数据就绪 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart == &huart9){
        if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
        {
            /* 当前使用 Memory 0 */
            __HAL_DMA_DISABLE(huart->hdmarx);
            ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
            __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);

            BSP_SBUS_OnUartRx(Size, SBUS_MultiRx_Buf[0]);
        }
        else
        {
            /* 当前使用 Memory 1 */
            __HAL_DMA_DISABLE(huart->hdmarx);
            ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);

            BSP_SBUS_OnUartRx(Size, SBUS_MultiRx_Buf[1]);
        }

        huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
        huart->RxEventType = HAL_UART_RXEVENT_TC;
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
    else if (huart == &huart2)
    {
        g_imu_uart_ctx.rx_event_cnt++;
        g_imu_uart_ctx.rx_size = Size;
        g_imu_uart_ctx.rx_ready = 1U;
        g_imu_uart_ctx.uart2_gstate_dbg = (uint32_t)huart2.gState;
        g_imu_uart_ctx.uart2_rxstate_dbg = (uint32_t)huart2.RxState;
        g_imu_uart_ctx.uart2_isr_dbg = huart2.Instance->ISR;
        g_imu_uart_ctx.uart2_err_dbg = (uint32_t)huart2.ErrorCode;
    }
}

/* UART 错误：UART7 激光测距恢复，UART9 置位 SBUS 重启请求 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart9)
    {
        s_uart9_rx_restart_req = 1U;
    }
    else if (huart == &huart1 || huart == &huart3 || huart == &huart10)
    {
        R1UartRxDispatch_ErrorRecover();
    }
}
