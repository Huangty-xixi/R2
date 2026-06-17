/**
 * @file r1_link_z3_cmd.h
 * @brief R1/R2 三区线协议帧：EE + cmd_id + chk + FF（chk = SYNC1 ^ cmd_id）
 *
 * === 业务调用链 ===
 * 上行(R1→R2)：
 *   USART1: HAL_UART_RxCpltCallback → R1LinkZ3CmdLink_OnRxByte
 *     → r1_link_z3_cmd_rx_feed_byte → r1_link_z3_cmd_link_on_frame
 *     → r1_zone3_parse_from_link_z3_cmd → r1_zone3_link_z3_cmd_wire_to_z3
 *     → r1_zone3_parse_post → AppZone3_PostR1Cmd
 * 
 *   USART10: HAL_UART_RxCpltCallback → R1Link_OnRxByte
 *     → r1_link_z3_cmd_rx_feed_byte → r1_link_on_z3_cmd_frame
 *     → STOP: r1_zone3_parse_from_usart10_stop → AppZone3_PostR1Cmd
 *     → GET_KFS_POS1/2: r1_zone3_parse_from_link_z3_cmd → AppZone3_PostR1Cmd
 * 
 * 下行(R2→R1)：r1_link_z3_cmd_frame_pack → HAL_UART_Transmit
 * 
 * cmd_id: 1=左 2=中 3=右 4=STOP 5=上R1 6=取kfs位1 7=取kfs位2
 */
#ifndef R1_LINK_Z3_CMD_H
#define R1_LINK_Z3_CMD_H

#include <stdint.h>

#define R1_LINK_Z3_CMD_SYNC1       0xEEU
#define R1_LINK_Z3_CMD_SYNC2       0xFFU
#define R1_LINK_Z3_CMD_FRAME_BYTES 4U

#define R1_LINK_Z3_CMD_WIRE_CMD_ID_MAX  7U

typedef struct
{
    uint8_t buf[R1_LINK_Z3_CMD_FRAME_BYTES];
    uint8_t idx;
} r1_link_z3_cmd_rx_ctx_t;

void r1_link_z3_cmd_rx_reset(r1_link_z3_cmd_rx_ctx_t *ctx);

/** 每收 1 字节调用；返回 1 表示 frame4 已收齐 */
uint8_t r1_link_z3_cmd_rx_feed_byte(r1_link_z3_cmd_rx_ctx_t *ctx, uint8_t b,
                                    uint8_t frame4[R1_LINK_Z3_CMD_FRAME_BYTES]);

void r1_link_z3_cmd_frame_pack(uint8_t data, uint8_t frame4[R1_LINK_Z3_CMD_FRAME_BYTES]);

/** 0=OK 1=sync 2=chk 3=cmd_id 非法 4=null */
uint8_t r1_link_z3_cmd_frame_decode(const uint8_t frame4[R1_LINK_Z3_CMD_FRAME_BYTES],
                                    uint8_t *out_data);

#endif /* R1_LINK_Z3_CMD_H */
