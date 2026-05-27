/**
 * @file r1_usart1_link.h
 * @brief R2 经 USART1 收 R1 指令帧（cmd_id：1=放三层，2/3 预留）
 */
#ifndef R1_USART1_LINK_H
#define R1_USART1_LINK_H

#include <stdint.h>
#include "r1_usart1_proto.h"

typedef struct
{
    uint8_t frame_rx[R1_USART1_FRAME_BYTES];
    uint8_t decode_rc;
    uint8_t frame_tick;
    r1_usart1_cmd_t cmd;
} r1_usart1_link_dbg_t;

extern volatile r1_usart1_link_dbg_t g_r1_usart1_link_dbg;

void R1Usart1Link_Init(void);
void R1Usart1Link_ErrorRecover(void);

/** 每收 1 字节（HAL 回调内调用） */
void R1Usart1Link_OnRxByte(uint8_t b);

uint8_t R1Usart1Link_HasNewCmd(void);

/** 取走最新命令；无新帧或 out==NULL 返回 0 */
uint8_t R1Usart1Link_TakeCmd(r1_usart1_cmd_t *out);

/** 窥视最新命令，不清标志 */
uint8_t R1Usart1Link_PeekCmd(r1_usart1_cmd_t *out);

uint32_t R1Usart1Link_FrameOkCount(void);
uint32_t R1Usart1Link_FrameErrCount(void);

#endif /* R1_USART1_LINK_H */
