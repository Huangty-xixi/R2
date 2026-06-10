/**
 * @file r1_link_z3_cmd.h
 * @brief R1/R2 三区线协议帧：EE + cmd_id + chk + FF（chk = SYNC1 ^ cmd_id）
 *
 * cmd_id 与 app_zone3_cmd_id_t 一致：1~5
 */
#ifndef R1_LINK_Z3_CMD_H
#define R1_LINK_Z3_CMD_H

#include <stdint.h>

#define R1_LINK_Z3_CMD_SYNC1       0xEEU
#define R1_LINK_Z3_CMD_SYNC2       0xFFU
#define R1_LINK_Z3_CMD_FRAME_BYTES 4U

#define R1_LINK_Z3_CMD_WIRE_CMD_ID_MAX  5U

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
