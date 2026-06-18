/**
 * @file r1_link_z3_cmd_link.h
 * @brief R2 经 USART1 收 R1 三区 EE..FF 指令帧（5字节，wire 1~7）
 */
#ifndef R1_LINK_Z3_CMD_LINK_H
#define R1_LINK_Z3_CMD_LINK_H

#include <stdint.h>

#include "r1_link_z3_cmd.h"

typedef struct
{
    uint8_t frame_rx[R1_LINK_Z3_CMD_FRAME_BYTES];
    uint8_t decode_rc;
    uint8_t frame_tick;
    uint8_t cmd_id;
    uint8_t put_sub;
} r1_link_z3_cmd_link_dbg_t;

extern volatile r1_link_z3_cmd_link_dbg_t g_r1_link_z3_cmd_link_dbg;

void R1LinkZ3CmdLink_Init(void);
void R1LinkZ3CmdLink_ErrorRecover(void);

/** HAL 收字节回调内调用（USART1） */
void R1LinkZ3CmdLink_OnRxByte(uint8_t b);

uint8_t R1LinkZ3CmdLink_HasNewData(void);
uint8_t R1LinkZ3CmdLink_TakeData(uint8_t *out);
uint8_t R1LinkZ3CmdLink_PeekData(uint8_t *out);

uint32_t R1LinkZ3CmdLink_FrameOkCount(void);
uint32_t R1LinkZ3CmdLink_FrameErrCount(void);

#endif /* R1_LINK_Z3_CMD_LINK_H */
