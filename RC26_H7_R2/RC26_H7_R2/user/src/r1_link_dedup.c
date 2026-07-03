#include "r1_link_dedup.h"

#include <stddef.h>

typedef struct
{
    uint8_t valid;
    uint32_t fingerprint;
    uint32_t last_ms;
} r1_link_dedup_slot_t;

static r1_link_dedup_slot_t s_dedup[r1_link_dedup_ch_count];
volatile uint32_t g_r1_link_dedup_drop_count[r1_link_dedup_ch_count];

void R1LinkDedup_Reset(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)r1_link_dedup_ch_count; i++)
    {
        s_dedup[i].valid = 0U;
        s_dedup[i].fingerprint = 0U;
        s_dedup[i].last_ms = 0U;
        g_r1_link_dedup_drop_count[i] = 0U;
    }
}

uint8_t R1LinkDedup_IsDuplicate(r1_link_dedup_channel_t ch, uint32_t fingerprint, uint32_t now_ms)
{
    r1_link_dedup_slot_t *slot;
    uint32_t elapsed;

    if (ch >= r1_link_dedup_ch_count)
    {
        return 0U;
    }

    slot = &s_dedup[ch];
    if (slot->valid != 0U)
    {
        elapsed = now_ms - slot->last_ms;
        if ((slot->fingerprint == fingerprint) && (elapsed < R1_LINK_DEDUP_WINDOW_MS))
        {
            g_r1_link_dedup_drop_count[ch]++;
            return 1U;
        }
    }

    slot->fingerprint = fingerprint;
    slot->last_ms = now_ms;
    slot->valid = 1U;
    return 0U;
}

uint32_t R1LinkDedup_FpZ3Cmd(uint8_t zone3_cmd_id, uint8_t put_sub)
{
    return ((uint32_t)zone3_cmd_id << 8) | (uint32_t)put_sub;
}

uint32_t R1LinkDedup_FpZ2Mission(const uint8_t *frame7, uint8_t frame_len)
{
    uint32_t fp = 0U;
    uint8_t i;

    if ((frame7 == NULL) || (frame_len < 7U))
    {
        return 0U;
    }

    for (i = 1U; i <= 5U; i++)
    {
        fp = (fp * 131U) + (uint32_t)frame7[i];
    }
    return fp;
}
uint32_t R1LinkDedup_FpZ1Sig(uint8_t sig_cmd)
{
    return 0xCC000000U | (uint32_t)sig_cmd;
}
