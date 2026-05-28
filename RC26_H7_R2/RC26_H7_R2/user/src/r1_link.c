/**
 * @file r1_link.c
 * @brief USART10 收 R1：7 字节任务帧 + 4 字节信令 + 4 字节三区 STOP（EE 04 EA FF）。
 */

#include "r1_link.h"

#include <string.h>

#include "app_zone3.h"
#include "r1_usart3_proto.h"
#include "r1_zone3_parse.h"
#include "usart.h"

/** Keil Watch：最近一帧解码前线数据与解码后任务快照 */
volatile r1_link_debug_t g_r1_link_dbg;

static r1_r2_connect_rx_ctx_t s_rx_ctx;
static r1_link_sig_rx_ctx_t s_sig_rx_ctx;
static r1_usart3_rx_ctx_t s_z3_stop_rx_ctx;

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
static volatile uint32_t s_z3_stop_ok;
static volatile uint32_t s_z3_stop_err;

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

static void r1_link_on_mission_frame(const uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES])    /* 处理任务帧 */
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
        s_has_new = 1U;    /* 设置有新任务 */
        s_frame_ok++;
    }
    else
    {
        s_frame_err++;
    }

    r1_link_debug_capture_frame(frame7, rc, &wire, (rc == 0U) ? &z2 : NULL);
}

static void r1_link_on_sig_frame(const uint8_t frame4[R1_LINK_SIG_FRAME_BYTES])    /* 处理信令帧 */
{
    r1_link_sig_cmd_t cmd;
    uint8_t rc;

    rc = r1_link_sig_frame_decode(frame4, &cmd);
    r1_link_debug_capture_sig(frame4, rc);

    if (rc == 0U)
    {
        s_last_sig = cmd;
        s_has_new_sig = 1U;    /* 设置有新信令 */
        s_sig_ok++;
    }
    else
    {
        s_sig_err++;
    }
}

static void r1_link_on_z3_stop_frame(const uint8_t frame4[R1_USART3_FRAME_BYTES])    /* USART10 三区 STOP：EE cmd chk FF，仅 cmd_id=4 */
{
    uint8_t cmd_id;
    uint8_t rc;

    rc = r1_usart3_frame_decode(frame4, &cmd_id);
    if (rc != 0U || cmd_id != (uint8_t)APP_Z3_CMD_STOP_ACTION)
    {
        s_z3_stop_err++;
        return;
    }

    s_z3_stop_ok++;
    r1_zone3_parse_from_usart10_stop();
}

void R1Link_OnRxByte(uint8_t b) /* 每收 1 字节；收齐任务/信令/STOP 帧后分别处理 */
{
    uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES];
    uint8_t frame4[R1_LINK_SIG_FRAME_BYTES];
    uint8_t frame4_stop[R1_USART3_FRAME_BYTES];

    if (r1_r2_connect_rx_feed_byte(&s_rx_ctx, b, frame7) != 0U)    /* 收齐 7 字节任务帧 */
    {
        r1_link_on_mission_frame(frame7);
        return;
    }

    if (r1_link_sig_rx_feed_byte(&s_sig_rx_ctx, b, frame4) != 0U)    /* 收齐 4 字节信令帧 */
    {
        r1_link_on_sig_frame(frame4);
        return;
    }

    if (r1_usart3_rx_feed_byte(&s_z3_stop_rx_ctx, b, frame4_stop) != 0U)    /* 收齐 4 字节三区 STOP 帧 */
    {
        r1_link_on_z3_stop_frame(frame4_stop);
    }
}

void R1Link_Init(void)
{
    r1_r2_connect_rx_reset(&s_rx_ctx);
    r1_link_sig_rx_reset(&s_sig_rx_ctx);
    r1_usart3_rx_reset(&s_z3_stop_rx_ctx);
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
    s_z3_stop_ok = 0U;
    s_z3_stop_err = 0U;
}

void R1Link_ErrorRecover(void)
{
    r1_r2_connect_rx_reset(&s_rx_ctx);
    r1_link_sig_rx_reset(&s_sig_rx_ctx);
    r1_usart3_rx_reset(&s_z3_stop_rx_ctx);
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

uint8_t R1Link_HasNewSig(void)   /* 是否有新信令 */
{
    return s_has_new_sig;
}

uint8_t R1Link_TakeSig(r1_link_sig_cmd_t *out)    /* 取走最新信令；无新信令或 out==NULL 返回 0 */
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

uint8_t R1Link_SendSig(r1_link_sig_cmd_t cmd)    /* 发送信令；成功返回 1 */
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

uint32_t R1Link_FrameOkCount(void)    /* 任务帧接收成功次数 */
{
    return s_frame_ok;
}

uint32_t R1Link_FrameErrCount(void)    /* 任务帧接收失败次数 */
{
    return s_frame_err;
}

uint32_t R1Link_SigOkCount(void)    /* 信令帧接收成功次数 */
{
    return s_sig_ok;
}

uint32_t R1Link_SigErrCount(void)    /* 信令帧接收失败次数 */
{
    return s_sig_err;
}

uint8_t R1Link_HasLastRxFrame(void)    /* 是否有最新接收的任务帧 */
{
    return s_has_last_frame;
}

uint8_t R1Link_CopyLastRxFrame(uint8_t frame7[R1_LINK_FRAME_BYTES])    /* 复制最新任务帧；成功返回 1 */
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

uint8_t R1Link_SendLastRxFrameToR1(void)    /* 将最新任务帧经 USART10 发回 R1；成功返回 1 */
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
