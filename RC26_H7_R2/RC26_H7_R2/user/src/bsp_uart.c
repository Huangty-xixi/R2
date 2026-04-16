#include "bsp_uart.h"
#include "usart.h"
/**
 * @brief       启动UART接收的DMA双缓冲区模式（支持IDLE中断触发）
 * @details     配置UART为IDLE空闲中断接收模式，启用DMA双缓冲区接收数据，
 *              当DMA传输完成(TC)或检测到IDLE空闲帧时触发中断，适用于不定长数据接收
 * @param       huart           UART句柄指针
 * @param       SrcAddress      DMA源地址（UART数据寄存器地址，一般为&huart->Instance->DR）
 * @param       DstAddress      DMA第一缓冲区目标地址（接收数据存储的首地址）
 * @param       SecondMemAddress DMA第二缓冲区目标地址（双缓冲备用存储地址）
 * @param       DataLength      单个DMA缓冲区的数据长度（字节数）
 * @date        2025/12/25
 * @author      zhouxy
 */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxEventType = HAL_UART_RXEVENT_TC;

 huart->RxXferSize    = DataLength;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 

 HAL_DMAEx_MultiBufferStart(&hdma_uart5_rx,(uint32_t)SrcAddress,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength);

}

/**
* @brief 串口初始化
* @date&author  2025/12/25  zhouxy
*/
void BSP_USART_Init(void){

	USART_RxDMA_MultiBufferStart(&huart5,(uint32_t *)&(huart5.Instance->RDR),(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],36);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart == &huart5){
         if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
      {
                //Disable DMA 
                __HAL_DMA_DISABLE(huart->hdmarx);

                ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
          /* reset the receive count */
          __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

          if(Size == 0x000B && SBUS_MultiRx_Buf[0][0] == 0x0F  && SBUS_MultiRx_Buf[0][24] == 0x00)
          {
            SBUS_TO_RC(SBUS_MultiRx_Buf[0],&RCctrl);
          }
      }
      /* Current memory buffer used is Memory 1 */
      else
      {
                //Disable DMA 
            __HAL_DMA_DISABLE(huart->hdmarx);
            
           ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
            
          /* reset the receive count */
          __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

          if(Size == 0x000B && SBUS_MultiRx_Buf[1][0] == 0x0F  && SBUS_MultiRx_Buf[1][24] == 0x00)
          {
            SBUS_TO_RC(SBUS_MultiRx_Buf[1],&RCctrl);
          }
      }
        

      huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
        
         huart->RxEventType = HAL_UART_RXEVENT_TC;

      /* Enalbe IDLE interrupt */
      __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        
      /* Enable the DMA transfer for the receiver request */
      SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        
      /* Enable DMA */
      __HAL_DMA_ENABLE(huart->hdmarx);
  }

}
