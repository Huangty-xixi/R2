/**
 * @file r1_link_sig.h
 * @brief R1/R2 红外简单信令帧（4 字节 CC..DD），与 7 字节任务帧 AA..BB 分离。
 */
#ifndef R1_LINK_SIG_H
#define R1_LINK_SIG_H

#include <stdint.h>

#define R1_LINK_SIG_SYNC1        0xCCU
#define R1_LINK_SIG_SYNC2        0xDDU
#define R1_LINK_SIG_CMD_RELEASE  0x01U
#define R1_LINK_SIG_FRAME_BYTES  4U

typedef enum
{
    r1_link_sig_none = 0,
    r1_link_sig_release = R1_LINK_SIG_CMD_RELEASE,
} r1_link_sig_cmd_t;   //信令命令    

typedef struct
{
    uint8_t buf[R1_LINK_SIG_FRAME_BYTES];
    uint8_t idx;
} r1_link_sig_rx_ctx_t;

void r1_link_sig_rx_reset(r1_link_sig_rx_ctx_t *ctx);

/** 每收 1 字节调用；返回 1 表示 frame4 已收齐 */
uint8_t r1_link_sig_rx_feed_byte(r1_link_sig_rx_ctx_t *ctx, uint8_t b, uint8_t frame4[R1_LINK_SIG_FRAME_BYTES]);

void r1_link_sig_frame_pack(r1_link_sig_cmd_t cmd, uint8_t frame4[R1_LINK_SIG_FRAME_BYTES]);

/** 解帧到 out；返回 0 成功，非 0 失败 */
uint8_t r1_link_sig_frame_decode(const uint8_t frame4[R1_LINK_SIG_FRAME_BYTES], r1_link_sig_cmd_t *out);

#endif /* R1_LINK_SIG_H */
