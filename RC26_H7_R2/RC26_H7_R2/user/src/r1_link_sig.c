/**
 * @file r1_link_sig.c
 * @brief R1/R2 4 ×Ö½ÚĞÅÁîÖ¡£ºCC + cmd + chk + DD£¨chk = SYNC1 ^ cmd£©
 */

#include "r1_link_sig.h"

#include <string.h>

static uint8_t r1_link_sig_calc_chk(uint8_t cmd)
{
    return (uint8_t)(R1_LINK_SIG_SYNC1 ^ cmd);
}

void r1_link_sig_rx_reset(r1_link_sig_rx_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }
    ctx->idx = 0U;
    (void)memset(ctx->buf, 0, sizeof(ctx->buf));
}

void r1_link_sig_frame_pack(r1_link_sig_cmd_t cmd, uint8_t frame4[R1_LINK_SIG_FRAME_BYTES])
{
    if (frame4 == NULL || cmd != r1_link_sig_release)
    {
        return;
    }

    frame4[0] = R1_LINK_SIG_SYNC1;
    frame4[1] = R1_LINK_SIG_CMD_RELEASE;
    frame4[2] = r1_link_sig_calc_chk(R1_LINK_SIG_CMD_RELEASE);
    frame4[3] = R1_LINK_SIG_SYNC2;
}

uint8_t r1_link_sig_frame_decode(const uint8_t frame4[R1_LINK_SIG_FRAME_BYTES], r1_link_sig_cmd_t *out)
{
    if (frame4 == NULL || out == NULL)
    {
        return 4U;
    }

    if (frame4[0] != R1_LINK_SIG_SYNC1 || frame4[3] != R1_LINK_SIG_SYNC2)
    {
        return 1U;
    }

    if (frame4[1] != R1_LINK_SIG_CMD_RELEASE)
    {
        return 2U;
    }

    if (frame4[2] != r1_link_sig_calc_chk(R1_LINK_SIG_CMD_RELEASE))
    {
        return 3U;
    }

    *out = r1_link_sig_release;
    return 0U;
}

uint8_t r1_link_sig_rx_feed_byte(r1_link_sig_rx_ctx_t *ctx, uint8_t b, uint8_t frame4[R1_LINK_SIG_FRAME_BYTES])
{
    if (ctx == NULL || frame4 == NULL)
    {
        return 0U;
    }

    if (ctx->idx == 0U)
    {
        if (b == R1_LINK_SIG_SYNC1)
        {
            ctx->buf[0] = b;
            ctx->idx = 1U;
        }
        return 0U;
    }

    if (ctx->idx < R1_LINK_SIG_FRAME_BYTES)
    {
        ctx->buf[ctx->idx] = b;
        ctx->idx++;

        if (ctx->idx == R1_LINK_SIG_FRAME_BYTES)
        {
            ctx->idx = 0U;
            if (ctx->buf[0] == R1_LINK_SIG_SYNC1 && ctx->buf[R1_LINK_SIG_FRAME_BYTES - 1U] == R1_LINK_SIG_SYNC2)
            {
                (void)memcpy(frame4, ctx->buf, (size_t)R1_LINK_SIG_FRAME_BYTES);
                return 1U;
            }
            if (b == R1_LINK_SIG_SYNC1)
            {
                ctx->buf[0] = b;
                ctx->idx = 1U;
            }
        }
        return 0U;
    }

    return 0U;
}
