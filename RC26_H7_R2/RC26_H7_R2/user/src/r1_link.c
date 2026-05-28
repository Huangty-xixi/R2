/**
 * @file r1_link.c
 * @brief USART10 收 R1：7 字节任务帧 + 4 字节信令帧，分别解码缓存。
 */

#include "r1_link.h"

#include <string.h>

#include "usart.h"

/** Keil Watch：最近一帧解码前线数据与解码后任务快照 */
volatile r1_link_debug_t g_r1_link_dbg;

static r1_r2_connect_rx_ctx_t s_rx_ctx;
static r1_link_sig_rx_ctx_t s_sig_rx_ctx;
static uint8_t s_rx_byte;

static app_zone2_mission_t s_last_mission;
static volatile uint8_t s_has_new;

static r1_link_sig_cmd_t s_last_sig;
static volatile uint8_t s_has_new_sig;

static uint8_t s_last_frame7[R1_R2_CONNECT_FRAME_BYTES];
static volatile uint8_t s_has_last_frame;

static volatile uint32_t s_frame_ok;
static volatile uint32_t s_frame_err;
static volatile uint32_t s_sig_ok;
static volatile uint32_t s_sig_err;

#define R1_LINK_TX_TIMEOUT_MS 20U

static void r1_link_wire_to_zone2(const r1_r2_mission_t *wire, app_zone2_mission_t *z2)
{
    uint8_t i;

    if (wire == NULL || z2 == NULL)
    {
        return;
    }

    (void)memset(z2, 0, sizeof(*z2));
    z2->path_n = wire->path_n;
    z2->kfs_n = wire->kfs_n;

    for (i = 0U; i < wire->path_n && i < APP_ZONE2_MAX_PATH; i++)
    {
        z2->path[i] = wire->path[i];
    }
    for (i = 0U; i < wire->kfs_n && i < APP_ZONE2_MAX_KFS; i++)
    {
        z2->kfs[i] = wire->kfs[i];
    }
}

static void r1_link_debug_capture_frame(const uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES],
                                        uint8_t decode_rc,
                                        const r1_r2_mission_t *wire,
                                        const app_zone2_mission_t *z2)
{
    (void)memcpy((void *)g_r1_link_dbg.frame_rx, frame7, (size_t)R1_LINK_FRAME_BYTES);
    g_r1_link_dbg.decode_rc = decode_rc;
    g_r1_link_dbg.frame_tick++;

    if (wire != NULL)
    {
        (void)memcpy((void *)&g_r1_link_dbg.wire, wire, sizeof(g_r1_link_dbg.wire));
    }
    else
    {
        (void)memset((void *)&g_r1_link_dbg.wire, 0, sizeof(g_r1_link_dbg.wire));
    }

    if (decode_rc == 0U && z2 != NULL)
    {
        (void)memcpy((void *)&g_r1_link_dbg.zone2, z2, sizeof(g_r1_link_dbg.zone2));
    }
    else
    {
        (void)memset((void *)&g_r1_link_dbg.zone2, 0, sizeof(g_r1_link_dbg.zone2));
    }
}

static void r1_link_debug_capture_sig(const uint8_t frame4[R1_LINK_SIG_FRAME_BYTES], uint8_t decode_rc)
{
    (void)memcpy((void *)g_r1_link_dbg.frame_sig_rx, frame4, (size_t)R1_LINK_SIG_FRAME_BYTES);
    g_r1_link_dbg.sig_decode_rc = decode_rc;
    g_r1_link_dbg.sig_tick++;
}

static void r1_link_on_mission_frame(const uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES])
{
    r1_r2_mission_t wire;
    app_zone2_mission_t z2;
    uint8_t rc;

    (void)memcpy(s_last_frame7, frame7, (size_t)R1_R2_CONNECT_FRAME_BYTES);
    s_has_last_frame = 1U;

    (void)memset(&wire, 0, sizeof(wire));
    (void)memset(&z2, 0, sizeof(z2));
    rc = r1_r2_connect_mission_decode(frame7, &wire);

    if (rc == 0U)
    {
        r1_link_wire_to_zone2(&wire, &z2);
        r1_link_wire_to_zone2(&wire, &s_last_mission);
        s_has_new = 1U;
        s_frame_ok++;
    }
    else
    {
        s_frame_err++;
    }

    r1_link_debug_capture_frame(frame7, rc, &wire, (rc == 0U) ? &z2 : NULL);
}

static void r1_link_on_sig_frame(const uint8_t frame4[R1_LINK_SIG_FRAME_BYTES])
{
    r1_link_sig_cmd_t cmd;
    uint8_t rc;

    rc = r1_link_sig_frame_decode(frame4, &cmd);
    r1_link_debug_capture_sig(frame4, rc);

    if (rc == 0U)
    {
        s_last_sig = cmd;
        s_has_new_sig = 1U;
        s_sig_ok++;
    }
    else
    {
        s_sig_err++;
    }
}

static void r1_link_start_rx_it(void)
{
    (void)HAL_UART_AbortReceive(&huart10);
    __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte, 1U);
}

static void r1_link_on_rx_byte(uint8_t b)
{
    uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES];
    uint8_t frame4[R1_LINK_SIG_FRAME_BYTES];

    if (r1_r2_connect_rx_feed_byte(&s_rx_ctx, b, frame7) != 0U)
    {
        r1_link_on_mission_frame(frame7);
        return;
    }

    if (r1_link_sig_rx_feed_byte(&s_sig_rx_ctx, b, frame4) != 0U)
    {
        r1_link_on_sig_frame(frame4);
    }
}

void R1Link_Init(void)
{
    r1_r2_connect_rx_reset(&s_rx_ctx);
    r1_link_sig_rx_reset(&s_sig_rx_ctx);
    (void)memset(&s_last_mission, 0, sizeof(s_last_mission));
    (void)memset(s_last_frame7, 0, sizeof(s_last_frame7));
    (void)memset((void *)&g_r1_link_dbg, 0, sizeof(g_r1_link_dbg));
    s_has_new = 0U;
    s_has_new_sig = 0U;
    s_last_sig = r1_link_sig_none;
    s_has_last_frame = 0U;
    s_frame_ok = 0U;
    s_frame_err = 0U;
    s_sig_ok = 0U;
    s_sig_err = 0U;
    r1_link_start_rx_it();
}

void R1Link_ErrorRecover(void)
{
    r1_r2_connect_rx_reset(&s_rx_ctx);
    r1_link_sig_rx_reset(&s_sig_rx_ctx);
    r1_link_start_rx_it();
}

uint8_t R1Link_HasNewMission(void)
{
    return s_has_new;
}

uint8_t R1Link_TakeMission(app_zone2_mission_t *out)
{
    if (out == NULL || s_has_new == 0U)
    {
        return 0U;
    }

    __disable_irq();
    (void)memcpy(out, &s_last_mission, sizeof(*out));
    s_has_new = 0U;
    __enable_irq();
    return 1U;
}

uint8_t R1Link_PeekMission(app_zone2_mission_t *out)
{
    if (out == NULL || s_has_new == 0U)
    {
        return 0U;
    }

    __disable_irq();
    (void)memcpy(out, &s_last_mission, sizeof(*out));
    __enable_irq();
    return 1U;
}

uint8_t R1Link_TakeAndApply(void)
{
    app_zone2_mission_t m;

    if (R1Link_TakeMission(&m) == 0U)
    {
        return 0U;
    }

    app_zone2_mission_apply(&m);
    return 1U;
}

uint8_t R1Link_HasNewSig(void)
{
    return s_has_new_sig;
}

uint8_t R1Link_TakeSig(r1_link_sig_cmd_t *out)
{
    if (out == NULL || s_has_new_sig == 0U)
    {
        return 0U;
    }

    __disable_irq();
    *out = s_last_sig;
    s_has_new_sig = 0U;
    __enable_irq();
    return 1U;
}

uint8_t R1Link_SendSig(r1_link_sig_cmd_t cmd)
{
    uint8_t frame4[R1_LINK_SIG_FRAME_BYTES];

    if (cmd != r1_link_sig_release)
    {
        return 0U;
    }

    r1_link_sig_frame_pack(cmd, frame4);
    if (HAL_UART_Transmit(&huart10, frame4, R1_LINK_SIG_FRAME_BYTES, R1_LINK_TX_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

uint32_t R1Link_FrameOkCount(void)
{
    return s_frame_ok;
}

uint32_t R1Link_FrameErrCount(void)
{
    return s_frame_err;
}

uint32_t R1Link_SigOkCount(void)
{
    return s_sig_ok;
}

uint32_t R1Link_SigErrCount(void)
{
    return s_sig_err;
}

uint8_t R1Link_HasLastRxFrame(void)
{
    return s_has_last_frame;
}

uint8_t R1Link_CopyLastRxFrame(uint8_t frame7[R1_LINK_FRAME_BYTES])
{
    if (frame7 == NULL || s_has_last_frame == 0U)
    {
        return 0U;
    }

    __disable_irq();
    (void)memcpy(frame7, s_last_frame7, (size_t)R1_LINK_FRAME_BYTES);
    __enable_irq();
    return 1U;
}

uint8_t R1Link_SendLastRxFrameToR1(void)
{
    uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES];

    if (R1Link_CopyLastRxFrame(frame7) == 0U)
    {
        return 0U;
    }

    if (HAL_UART_Transmit(&huart10, frame7, R1_R2_CONNECT_FRAME_BYTES, R1_LINK_TX_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        r1_link_on_rx_byte(s_rx_byte);
        (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte, 1U);
    }
}
