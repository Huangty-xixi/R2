/**
 * @file r1_usart3_link.c
 * @brief USART3 R1->R2 ºÚµ•–≈¡Ó¡¥¬∑
 */

#include "r1_usart3_link.h"

#include "r1_zone3_parse.h"

#include <string.h>

volatile r1_usart3_link_dbg_t g_r1_usart3_link_dbg;

static r1_usart3_rx_ctx_t s_rx_ctx;

static uint8_t s_last_data;
static volatile uint8_t s_has_new;

static volatile uint32_t s_frame_ok;
static volatile uint32_t s_frame_err;

static void r1_usart3_link_on_frame(const uint8_t frame4[R1_USART3_FRAME_BYTES])
{
    uint8_t data;
    uint8_t rc;

    rc = r1_usart3_frame_decode(frame4, &data);

    (void)memcpy((void *)g_r1_usart3_link_dbg.frame_rx, frame4, (size_t)R1_USART3_FRAME_BYTES);
    g_r1_usart3_link_dbg.decode_rc = rc;
    g_r1_usart3_link_dbg.frame_tick++;

    if (rc == 0U)
    {
        s_last_data = data;
        g_r1_usart3_link_dbg.data = data;
        s_has_new = 1U;
        s_frame_ok++;
        r1_zone3_parse_from_usart3(data);
    }
    else
    {
        g_r1_usart3_link_dbg.data = 0U;
        s_frame_err++;
    }
}

void R1Usart3Link_Init(void)
{
    r1_usart3_rx_reset(&s_rx_ctx);
    (void)memset((void *)&g_r1_usart3_link_dbg, 0, sizeof(g_r1_usart3_link_dbg));
    s_has_new = 0U;
    s_frame_ok = 0U;
    s_frame_err = 0U;
}

void R1Usart3Link_ErrorRecover(void)
{
    r1_usart3_rx_reset(&s_rx_ctx);
}

void R1Usart3Link_OnRxByte(uint8_t b)
{
    uint8_t frame4[R1_USART3_FRAME_BYTES];

    if (r1_usart3_rx_feed_byte(&s_rx_ctx, b, frame4) != 0U)
    {
        r1_usart3_link_on_frame(frame4);
    }
}

uint8_t R1Usart3Link_HasNewData(void)
{
    return s_has_new;
}

uint8_t R1Usart3Link_TakeData(uint8_t *out)
{
    if (out == NULL || s_has_new == 0U)
    {
        return 0U;
    }

    __disable_irq();
    *out = s_last_data;
    s_has_new = 0U;
    __enable_irq();
    return 1U;
}

uint8_t R1Usart3Link_PeekData(uint8_t *out)
{
    if (out == NULL || s_has_new == 0U)
    {
        return 0U;
    }

    __disable_irq();
    *out = s_last_data;
    __enable_irq();
    return 1U;
}

uint32_t R1Usart3Link_FrameOkCount(void)
{
    return s_frame_ok;
}

uint32_t R1Usart3Link_FrameErrCount(void)
{
    return s_frame_err;
}
