/**
 * @file r1_usart3_link.h
 * @brief R2 经 USART3 收 R1 单字节信令
 */
#ifndef R1_USART3_LINK_H
#define R1_USART3_LINK_H

#include <stdint.h>
#include "r1_usart3_proto.h"

typedef struct
{
    uint8_t frame_rx[R1_USART3_FRAME_BYTES];
    uint8_t decode_rc;
    uint8_t frame_tick;
    uint8_t data;
} r1_usart3_link_dbg_t;

extern volatile r1_usart3_link_dbg_t g_r1_usart3_link_dbg;

void R1Usart3Link_Init(void);
void R1Usart3Link_ErrorRecover(void);

void R1Usart3Link_OnRxByte(uint8_t b);

uint8_t R1Usart3Link_HasNewData(void);

uint8_t R1Usart3Link_TakeData(uint8_t *out);

uint8_t R1Usart3Link_PeekData(uint8_t *out);

uint32_t R1Usart3Link_FrameOkCount(void);
uint32_t R1Usart3Link_FrameErrCount(void);

#endif /* R1_USART3_LINK_H */
