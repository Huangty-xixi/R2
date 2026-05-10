/**
 * @file R1_R2_connect.c
 */

#include "R1_R2_connect.h"

#include <string.h>

static r1_r2_connect_hooks_t s_hooks;
static uint8_t s_hook_decoded_set;

static uint8_t peek_bit(const uint8_t *buf, uint16_t bit_idx)
{
    return (uint8_t)((buf[bit_idx >> 3] >> (7U - (bit_idx & 7U))) & 1U);
}

static uint8_t peek_nibble(const uint8_t *buf, uint16_t bit_pos)
{
    uint8_t v = 0U;
    for (uint8_t k = 0U; k < 4U; k++)
        v = (uint8_t)((v << 1) | peek_bit(buf, (uint16_t)(bit_pos + k)));
    return v;
}

static uint16_t peek_12(const uint8_t *buf, uint16_t bit_pos)
{
    uint16_t acc = 0U;
    for (uint8_t k = 0U; k < 12U; k++)
        acc = (uint16_t)((acc << 1) | (uint16_t)peek_bit(buf, (uint16_t)(bit_pos + k)));
    return acc;
}

void r1_r2_connect_set_hooks(const r1_r2_connect_hooks_t *hooks)
{
    if (hooks == NULL)
    {
        s_hook_decoded_set = 0U;
        return;
    }
    s_hooks = *hooks;
    s_hook_decoded_set = 1U;
}

void r1_r2_connect_decode_bits(const uint8_t *buf, uint16_t payload_bit_len, r1_r2_mission_t *out)
{
    memset(out, 0, sizeof(*out));

    uint16_t bit_pos = 0U;
    uint8_t pi = 0U;

    for (;;)
    {
        if ((uint32_t)bit_pos + 12U > payload_bit_len)
            break;

        if (peek_12(buf, bit_pos) == 0U)
        {
            bit_pos = (uint16_t)(bit_pos + 12U);
            break;
        }

        if ((uint32_t)bit_pos + 4U > payload_bit_len)
            break;
        if (pi < R1_R2_CONNECT_MAX_PATH - 1U)
            out->path[pi++] = peek_nibble(buf, bit_pos);
        bit_pos = (uint16_t)(bit_pos + 4U);
    }

    out->path[pi] = 0U;

    uint8_t ki = 0U;
    while ((uint32_t)bit_pos + 4U <= payload_bit_len)
    {
        uint8_t nib = peek_nibble(buf, bit_pos);
        bit_pos = (uint16_t)(bit_pos + 4U);
        if (nib == 0U)
            continue;
        if (ki < R1_R2_CONNECT_MAX_KFS - 1U)
            out->kfs[ki++] = nib;
    }
    out->kfs[ki] = 0U;
}

void r1_r2_connect_decode_and_dispatch(const uint8_t *buf, uint16_t payload_bit_len)
{
    r1_r2_mission_t m;
    r1_r2_connect_decode_bits(buf, payload_bit_len, &m);
    if (s_hook_decoded_set && s_hooks.on_decoded != NULL)
        s_hooks.on_decoded(&m, s_hooks.user);
}
