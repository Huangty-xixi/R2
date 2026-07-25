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

/* CAN error state flags, accumulated by ErrorStatusCallback */
extern volatile uint32_t g_can1_err_flags;
extern volatile uint32_t g_can2_err_flags;
extern volatile uint32_t g_can3_err_flags;

#endif

