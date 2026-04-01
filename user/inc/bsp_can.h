#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "fdcan.h"

void BSP_CAN_Init(void);

typedef struct {
		FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
}FDCAN_TxFrame_TypeDef;

typedef struct {
		FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
} FDCAN_RxFrame_TypeDef;

//extern  FDCAN_TxFrame_TypeDef   SteerTxFrame;
//extern  FDCAN_TxFrame_TypeDef   WheelTxFrame;

#endif

