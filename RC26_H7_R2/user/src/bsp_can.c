#include "bsp_can.h"

#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"

uint8_t             rx_data[8];


FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN2_RxFrame;


FDCAN_TxFrame_TypeDef SteerTxFrame = {
  .hcan = &hfdcan2,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};
FDCAN_TxFrame_TypeDef WheelTxFrame = {
  .hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};



/**
* @brief can³õÊ¼»¯
* @date&author  2025/12/24  zhouxy
*/
void BSP_CAN_Init(void)
{

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_FilterConfig.FilterIndex = 0;
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; 
	

  
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  
	
	 FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	
	FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; 
	
  
	if(HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

FDCAN_FilterTypeDef FDCAN3_FilterConfig;
	
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
	
  
	if(HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
                     
}




void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t             rx_data[8];
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
    
    if(hfdcan->Instance == FDCAN1)
	{
        switch (rx_header.Identifier){
            case CHASSIS_MOTOR1_FEEDBACK_ID:{
				DJIget_motor_measure(&chassis_motor1,rx_data);
				break;
			}
			case CHASSIS_MOTOR2_FEEDBACK_ID:{
				DJIget_motor_measure(&chassis_motor2,rx_data);
				break;
			}
      case CHASSIS_MOTOR3_FEEDBACK_ID:{
				DJIget_motor_measure(&chassis_motor3,rx_data);
				break;
			}
			case CHASSIS_MOTOR4_FEEDBACK_ID:{
				DJIget_motor_measure(&chassis_motor4,rx_data);
				break;
			}
			
		}
        switch(rx_data[0] & 0x0F)
        {
            case R2_LIFT_MOTOR_LEFT_FEEDBACK_ID:
                DMget_motor_measure(&R2_lift_motor_left,rx_data);
                break;
            case R2_LIFT_MOTOR_RIGHT_FEEDBACK_ID:
                DMget_motor_measure(&R2_lift_motor_right,rx_data);
                break;
        }
    }
    else if(hfdcan->Instance == FDCAN2)
    {
        switch (rx_header.Identifier){
			case GUIDE_MOTOR1_FEEDBACK_ID:{
				DJIget_motor_measure(&guide_motor1,rx_data);
				break;
			}
			case GUIDE_MOTOR2_FEEDBACK_ID:{
				DJIget_motor_measure(&guide_motor2,rx_data);
				break;
			}
			

        }
        switch(rx_data[0] & 0x0F)
        {
            case FLEXIBLE_MOTOR1_FEEDBACK_ID:
                DMget_motor_measure(&flexible_motor1,rx_data);
                break;
            case FLEXIBLE_MOTOR2_FEEDBACK_ID:
                DMget_motor_measure(&flexible_motor2,rx_data);
                break;
        }
        
    }
	 else if(hfdcan->Instance == FDCAN3)
    {
        switch (rx_header.Identifier){

        }
        switch(rx_data[0] & 0x0F)
        {

        }
        
    }
}
	
