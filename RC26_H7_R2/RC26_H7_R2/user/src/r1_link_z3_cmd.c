/**
 * @file r1_link_z3_cmd.c
 * @brief R1/R2 5 字节三区指令帧：EE + cmd_id + put_sub + chk + FF
 *
 * chk = SYNC1 ^ cmd_id ^ put_sub
 * put_sub: cmd 1~3 须 00/01/02；其它命令须 00
 */

#include "r1_link_z3_cmd.h"

#include <string.h>

static uint8_t r1_link_z3_cmd_calc_chk(uint8_t cmd_id, uint8_t put_sub)
{
    return (uint8_t)(R1_LINK_Z3_CMD_SYNC1 ^ cmd_id ^ put_sub);
}

static uint8_t r1_link_z3_cmd_put_sub_valid(uint8_t cmd_id, uint8_t put_sub)
{
    if (cmd_id >= 1U && cmd_id <= 3U)
    {
        return (uint8_t)(put_sub <= R1_LINK_Z3_CMD_PUT_SUB_RIGHT);
    }

    return (uint8_t)(put_sub == R1_LINK_Z3_CMD_PUT_SUB_NONE);
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

void r1_link_z3_cmd_frame_pack(uint8_t cmd_id, uint8_t put_sub,
                               uint8_t frame[R1_LINK_Z3_CMD_FRAME_BYTES])
{
    if (frame == NULL || cmd_id == 0U || cmd_id > R1_LINK_Z3_CMD_WIRE_CMD_ID_MAX)
    {
        return;
    }
    if (r1_link_z3_cmd_put_sub_valid(cmd_id, put_sub) == 0U)
    {
        return;
    }

    frame[0] = R1_LINK_Z3_CMD_SYNC1;
    frame[1] = cmd_id;
    frame[2] = put_sub;
    frame[3] = r1_link_z3_cmd_calc_chk(cmd_id, put_sub);
    frame[4] = R1_LINK_Z3_CMD_SYNC2;
}

uint8_t r1_link_z3_cmd_frame_decode(const uint8_t frame[R1_LINK_Z3_CMD_FRAME_BYTES],
                                    r1_link_z3_cmd_frame_t *out)
{
    if (frame == NULL || out == NULL)
    {
        return 4U;
    }

    if (frame[0] != R1_LINK_Z3_CMD_SYNC1 || frame[4] != R1_LINK_Z3_CMD_SYNC2)
    {
        return 1U;
    }

    if (frame[1] == 0U || frame[1] > R1_LINK_Z3_CMD_WIRE_CMD_ID_MAX)
    {
        return 3U;
    }

    if (frame[3] != r1_link_z3_cmd_calc_chk(frame[1], frame[2]))
    {
        return 2U;
    }

    if (r1_link_z3_cmd_put_sub_valid(frame[1], frame[2]) == 0U)
    {
        return 5U;
    }

    out->cmd_id = frame[1];
    out->put_sub = frame[2];
    return 0U;
}

uint8_t r1_link_z3_cmd_rx_feed_byte(r1_link_z3_cmd_rx_ctx_t *ctx, uint8_t b,
                                    uint8_t frame[R1_LINK_Z3_CMD_FRAME_BYTES])
{
    if (ctx == NULL || frame == NULL)
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
                (void)memcpy(frame, ctx->buf, (size_t)R1_LINK_Z3_CMD_FRAME_BYTES);
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
