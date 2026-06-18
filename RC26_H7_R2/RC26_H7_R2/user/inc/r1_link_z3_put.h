/**
 * @file r1_link_z3_put.h
 * @brief R1/R2 4 字节三区线协议帧：55 + cmd_id + chk + AA（chk = SYNC1 ^ cmd_id）
 *
 * === 业务调用链 ===
 * USART10: HAL_UART_RxCpltCallback → R1Link_OnRxByte
 *   → r1_link_z3_put_rx_feed_byte()                    // 4字节帧拼装
 *   → r1_link_on_z3_put_frame(frame4)
 *   → r1_link_z3_put_frame_decode()                     // 校验55/AA/chk/cmd_id
 *   → r1_zone3_parse_from_link_z3_put(cmd_id, raw)
 *   → wire 1/2/3 → AppZone3_PostR1Cmd
 *   → AppZone3_PostR1Cmd(&z3)
 *
 * cmd_id: 1=放三层 2=二层左纠偏 3=二层右纠偏
 */
#ifndef R1_LINK_Z3_PUT_H
#define R1_LINK_Z3_PUT_H

#include <stdint.h>

#define R1_LINK_Z3_PUT_SYNC1       0x55U
#define R1_LINK_Z3_PUT_SYNC2       0xAAU
#define R1_LINK_Z3_PUT_FRAME_BYTES 4U

#define R1_LINK_Z3_PUT_WIRE_CMD_ID_PUT_L3         1U
#define R1_LINK_Z3_PUT_WIRE_CMD_ID_TRIM_P2_LEFT  2U
#define R1_LINK_Z3_PUT_WIRE_CMD_ID_TRIM_P2_RIGHT 3U
#define R1_LINK_Z3_PUT_WIRE_CMD_ID_MAX           3U

typedef struct
{
    uint8_t cmd_id;
} r1_link_z3_put_cmd_t;

typedef struct
{
    uint8_t buf[R1_LINK_Z3_PUT_FRAME_BYTES];
    uint8_t idx;
} r1_link_z3_put_rx_ctx_t;

void r1_link_z3_put_rx_reset(r1_link_z3_put_rx_ctx_t *ctx);

/** 接收 1 字节，解析三区线协议帧 */
uint8_t r1_link_z3_put_rx_feed_byte(r1_link_z3_put_rx_ctx_t *ctx, uint8_t b,
                                    uint8_t frame4[R1_LINK_Z3_PUT_FRAME_BYTES]);

void r1_link_z3_put_frame_pack_cmd_id(uint8_t cmd_id, uint8_t frame4[R1_LINK_Z3_PUT_FRAME_BYTES]);

void r1_link_z3_put_frame_pack(const r1_link_z3_put_cmd_t *cmd,
                               uint8_t frame4[R1_LINK_Z3_PUT_FRAME_BYTES]);

/** 0=OK 1=sync 2=chk 3=cmd_id 非法 4=null */
uint8_t r1_link_z3_put_frame_decode(const uint8_t frame4[R1_LINK_Z3_PUT_FRAME_BYTES],
                                    r1_link_z3_put_cmd_t *out);

#endif /* R1_LINK_Z3_PUT_H */
