/**
 * @file r1_usart3_proto.h
 * @brief R1->R2 USART3：EE + cmd_id(1B) + chk(EE^cmd_id) + FF
 *
 * cmd_id 与 app_zone3_cmd_id_t 一致：1~5
 */
#ifndef R1_USART3_PROTO_H
#define R1_USART3_PROTO_H

#include <stdint.h>

#define R1_USART3_SYNC1       0xEEU
#define R1_USART3_SYNC2       0xFFU
#define R1_USART3_FRAME_BYTES 4U

#define R1_USART3_WIRE_CMD_ID_MAX  5U

typedef struct
{
    uint8_t buf[R1_USART3_FRAME_BYTES];
    uint8_t idx;
} r1_usart3_rx_ctx_t;

void r1_usart3_rx_reset(r1_usart3_rx_ctx_t *ctx);

uint8_t r1_usart3_rx_feed_byte(r1_usart3_rx_ctx_t *ctx, uint8_t b,
                                uint8_t frame4[R1_USART3_FRAME_BYTES]);

void r1_usart3_frame_pack(uint8_t data, uint8_t frame4[R1_USART3_FRAME_BYTES]);

/** 0=OK 1=sync 2=chk 3=cmd_id 非法 4=null */
uint8_t r1_usart3_frame_decode(const uint8_t frame4[R1_USART3_FRAME_BYTES], uint8_t *out_data);

#endif /* R1_USART3_PROTO_H */
