/**
 * @file r1_link_z3_stop.h
 * @brief USART10 三区 STOP 简单帧：BA + 0xFF + chk + AB（chk = BA ^ data）
 */
#ifndef R1_LINK_Z3_STOP_H
#define R1_LINK_Z3_STOP_H

#include <stdint.h>

#define R1_LINK_Z3_STOP_SYNC1       0xBAU
#define R1_LINK_Z3_STOP_SYNC2       0xABU
#define R1_LINK_Z3_STOP_DATA        0xFFU
#define R1_LINK_Z3_STOP_FRAME_BYTES 4U

typedef struct
{
    uint8_t buf[R1_LINK_Z3_STOP_FRAME_BYTES];
    uint8_t idx;
} r1_link_z3_stop_rx_ctx_t;

void r1_link_z3_stop_rx_reset(r1_link_z3_stop_rx_ctx_t *ctx);

/** 每收 1 字节；返回 1 表示 frame4 已收齐 */
uint8_t r1_link_z3_stop_rx_feed_byte(r1_link_z3_stop_rx_ctx_t *ctx, uint8_t b,
                                       uint8_t frame4[R1_LINK_Z3_STOP_FRAME_BYTES]);

void r1_link_z3_stop_frame_pack(uint8_t frame4[R1_LINK_Z3_STOP_FRAME_BYTES]);

/** 0=OK，非 0 失败 */
uint8_t r1_link_z3_stop_frame_decode(const uint8_t frame4[R1_LINK_Z3_STOP_FRAME_BYTES]);

#endif /* R1_LINK_Z3_STOP_H */
