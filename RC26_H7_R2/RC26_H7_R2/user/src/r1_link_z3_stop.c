/**
 * @file r1_link_z3_stop.c
 * @brief USART10 三区 STOP：BA FF 45 AB
 */
// R1 STOP（USART10）链路：
// 1. 发送 BA FF 45 AB
// 2. r1_link_z3_stop_rx_feed_byte() 收齐 4 字节
// 3. r1_link.c R1Link_OnRxByte() -> r1_link_on_z3_stop_frame()
// 4. r1_zone3_parse_from_usart10_stop() -> AppZone3_PostR1Cmd(STOP_ACTION)
// 5. Motion_Task -> AppZone3_Run() 执行停止

#include "r1_link_z3_stop.h"

#include <string.h>

static uint8_t r1_link_z3_stop_calc_chk(uint8_t data)
{
    return (uint8_t)(R1_LINK_Z3_STOP_SYNC1 ^ data);
}

void r1_link_z3_stop_rx_reset(r1_link_z3_stop_rx_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }
    ctx->idx = 0U;
    (void)memset(ctx->buf, 0, sizeof(ctx->buf));
}

void r1_link_z3_stop_frame_pack(uint8_t frame4[R1_LINK_Z3_STOP_FRAME_BYTES])
{
    if (frame4 == NULL)
    {
        return;
    }

    frame4[0] = R1_LINK_Z3_STOP_SYNC1;
    frame4[1] = R1_LINK_Z3_STOP_DATA;
    frame4[2] = r1_link_z3_stop_calc_chk(R1_LINK_Z3_STOP_DATA);
    frame4[3] = R1_LINK_Z3_STOP_SYNC2;
}

uint8_t r1_link_z3_stop_frame_decode(const uint8_t frame4[R1_LINK_Z3_STOP_FRAME_BYTES])
{
    if (frame4 == NULL)
    {
        return 4U;
    }

    if (frame4[0] != R1_LINK_Z3_STOP_SYNC1 || frame4[3] != R1_LINK_Z3_STOP_SYNC2)
    {
        return 1U;
    }

    if (frame4[1] != R1_LINK_Z3_STOP_DATA)
    {
        return 2U;
    }

    if (frame4[2] != r1_link_z3_stop_calc_chk(R1_LINK_Z3_STOP_DATA))
    {
        return 3U;
    }

    return 0U;
}

uint8_t r1_link_z3_stop_rx_feed_byte(r1_link_z3_stop_rx_ctx_t *ctx, uint8_t b,
                                     uint8_t frame4[R1_LINK_Z3_STOP_FRAME_BYTES])
{
    if (ctx == NULL || frame4 == NULL)
    {
        return 0U;
    }

    if (ctx->idx == 0U)
    {
        if (b == R1_LINK_Z3_STOP_SYNC1)
        {
            ctx->buf[0] = b;
            ctx->idx = 1U;
        }
        return 0U;
    }

    if (ctx->idx < R1_LINK_Z3_STOP_FRAME_BYTES)
    {
        ctx->buf[ctx->idx] = b;
        ctx->idx++;

        if (ctx->idx == R1_LINK_Z3_STOP_FRAME_BYTES)
        {
            ctx->idx = 0U;
            if (ctx->buf[0] == R1_LINK_Z3_STOP_SYNC1 &&
                ctx->buf[R1_LINK_Z3_STOP_FRAME_BYTES - 1U] == R1_LINK_Z3_STOP_SYNC2)
            {
                (void)memcpy(frame4, ctx->buf, (size_t)R1_LINK_Z3_STOP_FRAME_BYTES);
                return 1U;
            }
            if (b == R1_LINK_Z3_STOP_SYNC1)
            {
                ctx->buf[0] = b;
                ctx->idx = 1U;
            }
        }
        return 0U;
    }

    return 0U;
}
