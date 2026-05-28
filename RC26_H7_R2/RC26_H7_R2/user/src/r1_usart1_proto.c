/**
 * @file r1_usart1_proto.c
 *@brief ÔÝ¶¨55 01 54 AA
 */

#include "r1_usart1_proto.h"

#include <string.h>

static uint8_t r1_usart1_calc_chk(uint8_t cmd_id)
{
    return (uint8_t)(R1_USART1_SYNC1 ^ cmd_id);
}

static uint8_t r1_usart1_cmd_id_valid(uint8_t cmd_id)
{
    return (uint8_t)(cmd_id >= 1U && cmd_id <= R1_USART1_WIRE_CMD_ID_MAX);
}

void r1_usart1_rx_reset(r1_usart1_rx_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }
    ctx->idx = 0U;
    (void)memset(ctx->buf, 0, sizeof(ctx->buf));
}

void r1_usart1_frame_pack_cmd_id(uint8_t cmd_id, uint8_t frame4[R1_USART1_FRAME_BYTES])
{
    if (frame4 == NULL || r1_usart1_cmd_id_valid(cmd_id) == 0U)
    {
        return;
    }

    frame4[0] = R1_USART1_SYNC1;
    frame4[1] = cmd_id;
    frame4[2] = r1_usart1_calc_chk(cmd_id);
    frame4[3] = R1_USART1_SYNC2;
}

void r1_usart1_frame_pack(const r1_usart1_cmd_t *cmd, uint8_t frame4[R1_USART1_FRAME_BYTES])
{
    if (cmd == NULL)
    {
        return;
    }

    r1_usart1_frame_pack_cmd_id(cmd->cmd_id, frame4);
}

uint8_t r1_usart1_frame_decode(const uint8_t frame4[R1_USART1_FRAME_BYTES], r1_usart1_cmd_t *out)
{
    if (frame4 == NULL || out == NULL)
    {
        return 4U;
    }

    if (frame4[0] != R1_USART1_SYNC1 || frame4[3] != R1_USART1_SYNC2)
    {
        return 1U;
    }

    if (frame4[2] != r1_usart1_calc_chk(frame4[1]))
    {
        return 2U;
    }

    if (r1_usart1_cmd_id_valid(frame4[1]) == 0U)
    {
        return 3U;
    }

    out->cmd_id = frame4[1];
    return 0U;
}

uint8_t r1_usart1_rx_feed_byte(r1_usart1_rx_ctx_t *ctx, uint8_t b, uint8_t frame4[R1_USART1_FRAME_BYTES])
{
    if (ctx == NULL || frame4 == NULL)
    {
        return 0U;
    }

    if (ctx->idx == 0U)
    {
        if (b == R1_USART1_SYNC1)
        {
            ctx->buf[0] = b;
            ctx->idx = 1U;
        }
        return 0U;
    }

    if (ctx->idx < R1_USART1_FRAME_BYTES)
    {
        ctx->buf[ctx->idx] = b;
        ctx->idx++;

        if (ctx->idx == R1_USART1_FRAME_BYTES)
        {
            ctx->idx = 0U;
            if (ctx->buf[0] == R1_USART1_SYNC1 && ctx->buf[R1_USART1_FRAME_BYTES - 1U] == R1_USART1_SYNC2)
            {
                (void)memcpy(frame4, ctx->buf, (size_t)R1_USART1_FRAME_BYTES);
                return 1U;
            }
            if (b == R1_USART1_SYNC1)
            {
                ctx->buf[0] = b;
                ctx->idx = 1U;
            }
        }
        return 0U;
    }

    return 0U;
}
