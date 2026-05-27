/**
 * @file r1_usart1_proto.h
 * @brief R1->R2 USART1：55 + cmd_id(1B) + chk(55^cmd_id) + AA
 *
 * cmd_id：1=放三层，2/3 预留（帧可解，业务未实现）
 */
#ifndef R1_USART1_PROTO_H
#define R1_USART1_PROTO_H

#include <stdint.h>

#define R1_USART1_SYNC1       0x55U
#define R1_USART1_SYNC2       0xAAU
#define R1_USART1_FRAME_BYTES 4U

#define R1_USART1_WIRE_CMD_ID_PUT_L3  1U
#define R1_USART1_WIRE_CMD_ID_RSVD2   2U
#define R1_USART1_WIRE_CMD_ID_RSVD3   3U
#define R1_USART1_WIRE_CMD_ID_MAX     3U

typedef struct
{
    uint8_t cmd_id;
} r1_usart1_cmd_t;

typedef struct
{
    uint8_t buf[R1_USART1_FRAME_BYTES];
    uint8_t idx;
} r1_usart1_rx_ctx_t;

void r1_usart1_rx_reset(r1_usart1_rx_ctx_t *ctx);

uint8_t r1_usart1_rx_feed_byte(r1_usart1_rx_ctx_t *ctx, uint8_t b,
                                uint8_t frame4[R1_USART1_FRAME_BYTES]);

void r1_usart1_frame_pack_cmd_id(uint8_t cmd_id, uint8_t frame4[R1_USART1_FRAME_BYTES]);

void r1_usart1_frame_pack(const r1_usart1_cmd_t *cmd, uint8_t frame4[R1_USART1_FRAME_BYTES]);

/** 0=OK 1=sync 2=chk 3=cmd_id 非法 4=null */
uint8_t r1_usart1_frame_decode(const uint8_t frame4[R1_USART1_FRAME_BYTES],
                               r1_usart1_cmd_t *out);

#endif /* R1_USART1_PROTO_H */
