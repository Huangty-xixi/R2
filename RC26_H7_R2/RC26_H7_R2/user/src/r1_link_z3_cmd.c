/**
 * @file r1_link_z3_cmd.c
 * @brief R1/R2 4 字节三区指令帧：EE + cmd_id + chk + FF（chk = SYNC1 ^ cmd_id）
 *
 * === 业务调用链 ===
 * r1_link_z3_cmd_rx_feed_byte() — 逐字节喂入，4字节帧完成返回1
 * r1_link_z3_cmd_frame_decode()  — 校验EE/FF头尾 + checksum + cmd_id范围(1~7)
 * r1_link_z3_cmd_frame_pack()    — 组帧：EE + cmd_id + chk(EE^cmd_id) + FF
 * 
 * 上层入口: r1_link_z3_cmd_link.c (USART1) / r1_link.c (USART10)
 */

#include "r1_link_z3_cmd.h"

#include <string.h>

static uint8_t r1_link_z3_cmd_calc_chk(uint8_t data)
{
    return (uint8_t)(R1_LINK_Z3_CMD_SYNC1 ^ data);
}

void r1_link_z3_cmd_rx_reset(r1_link_z3_cmd_rx_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }
    ctx->idx = 0U;
    (void)memset(ctx->buf, 0, sizeof(ctx->buf));
}

void r1_link_z3_cmd_frame_pack(uint8_t data, uint8_t frame4[R1_LINK_Z3_CMD_FRAME_BYTES])
{
    if (frame4 == NULL || data == 0U || data > R1_LINK_Z3_CMD_WIRE_CMD_ID_MAX)
    {
        return;
    }

    frame4[0] = R1_LINK_Z3_CMD_SYNC1;
    frame4[1] = data;
    frame4[2] = r1_link_z3_cmd_calc_chk(data);
    frame4[3] = R1_LINK_Z3_CMD_SYNC2;
}

uint8_t r1_link_z3_cmd_frame_decode(const uint8_t frame4[R1_LINK_Z3_CMD_FRAME_BYTES],
                                    uint8_t *out_data)
{
    if (frame4 == NULL || out_data == NULL)
    {
        return 4U;
    }

    if (frame4[0] != R1_LINK_Z3_CMD_SYNC1 || frame4[3] != R1_LINK_Z3_CMD_SYNC2)
    {
        return 1U;
    }

    if (frame4[2] != r1_link_z3_cmd_calc_chk(frame4[1]))
    {
        return 2U;
    }

    if (frame4[1] == 0U || frame4[1] > R1_LINK_Z3_CMD_WIRE_CMD_ID_MAX)
    {
        return 3U;
    }

    *out_data = frame4[1];
    return 0U;
}

uint8_t r1_link_z3_cmd_rx_feed_byte(r1_link_z3_cmd_rx_ctx_t *ctx, uint8_t b,
                                    uint8_t frame4[R1_LINK_Z3_CMD_FRAME_BYTES])
{
    if (ctx == NULL || frame4 == NULL)
    {
        return 0U;
    }

    if (ctx->idx == 0U)
    {
        if (b == R1_LINK_Z3_CMD_SYNC1)
        {
            ctx->buf[0] = b;
            ctx->idx = 1U;
        }
        return 0U;
    }

    if (ctx->idx < R1_LINK_Z3_CMD_FRAME_BYTES)
    {
        ctx->buf[ctx->idx] = b;
        ctx->idx++;

        if (ctx->idx == R1_LINK_Z3_CMD_FRAME_BYTES)
        {
            ctx->idx = 0U;
            if (ctx->buf[0] == R1_LINK_Z3_CMD_SYNC1 &&
                ctx->buf[R1_LINK_Z3_CMD_FRAME_BYTES - 1U] == R1_LINK_Z3_CMD_SYNC2)
            {
                (void)memcpy(frame4, ctx->buf, (size_t)R1_LINK_Z3_CMD_FRAME_BYTES);
                return 1U;
            }
            if (b == R1_LINK_Z3_CMD_SYNC1)
            {
                ctx->buf[0] = b;
                ctx->idx = 1U;
            }
        }
        return 0U;
    }

    return 0U;
}
