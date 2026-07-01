/**
 * @file r1_link.c
 * @brief USART10 收 R1 线协议帧：任务帧、信令帧、三区放料/指令帧
 *
 * === 业务调用链 ===
 * USART10 IRQ -> HAL_UART_RxCpltCallback -> R1Link_OnRxByte(b)
 *   按字节依次尝试 4 种帧解析器拼帧
 *
 *   -> r1_r2_connect_rx_feed_byte -> r1_link_on_mission_frame
 *      -> r1_r2_connect_mission_decode -> s_has_new=1
 *      (Mission 由主循环 R1Link_TakeMission 消费)
 *
 *
 *   -> r1_link_z3_put_rx_feed_byte -> r1_link_on_z3_put_frame
 *      -> r1_zone3_parse_from_link_z3_put -> AppZone3_PostR1Cmd
 *
 *   -> r1_link_z3_cmd_rx_feed_byte -> r1_link_on_z3_cmd_frame
 *      -> STOP/GET_KFS/放料: r1_zone3_parse_from_* -> AppZone3_PostR1Cmd
 */

#include "r1_link.h"

#include <string.h>

#include "app_zone3.h"
#include "main.h"
#include "r1_link_dedup.h"
#include "r1_link_z3_cmd.h"
#include "r1_zone3_parse.h"
#include "usart.h"

/** Keil Watch 捕获收到的各种帧数据，用于调试和分析 */
volatile r1_link_debug_t g_r1_link_dbg;

static r1_r2_connect_rx_ctx_t s_rx_ctx;
static r1_link_z3_put_rx_ctx_t s_z3_put_rx_ctx;
static r1_link_z3_cmd_rx_ctx_t s_z3_stop_rx_ctx;

static app_zone2_mission_t s_last_mission;
static volatile uint8_t s_has_new;

static uint8_t s_last_frame7[R1_R2_CONNECT_FRAME_BYTES];
static volatile uint8_t s_has_last_frame;

static volatile uint32_t s_frame_ok;
static volatile uint32_t s_frame_err;
static volatile uint32_t s_z3_put_ok;
static volatile uint32_t s_z3_put_err;
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


static void r1_link_debug_capture_z3_put(const uint8_t frame4[R1_LINK_Z3_PUT_FRAME_BYTES],
                                         uint8_t decode_rc,
                                         const r1_link_z3_put_cmd_t *cmd)
{
    (void)memcpy((void *)g_r1_link_dbg.frame_z3_put_rx, frame4, (size_t)R1_LINK_Z3_PUT_FRAME_BYTES);
    g_r1_link_dbg.z3_put_decode_rc = decode_rc;
    g_r1_link_dbg.z3_put_tick++;

    if (decode_rc == 0U && cmd != NULL)
    {
        g_r1_link_dbg.z3_put_cmd = *cmd;
    }
    else
    {
        (void)memset((void *)&g_r1_link_dbg.z3_put_cmd, 0, sizeof(g_r1_link_dbg.z3_put_cmd));
    }
}

static void r1_link_debug_capture_z3_stop(const uint8_t frame5[R1_LINK_Z3_CMD_FRAME_BYTES],
                                          uint8_t decode_rc,
                                          uint8_t cmd_id,
                                          uint8_t accepted)
{
    (void)memcpy((void *)g_r1_link_dbg.frame_z3_stop_rx, frame5,
                 (size_t)R1_LINK_Z3_CMD_FRAME_BYTES);
    g_r1_link_dbg.z3_stop_decode_rc = decode_rc;
    g_r1_link_dbg.z3_stop_tick++;
    g_r1_link_dbg.z3_stop_cmd_id = (decode_rc == 0U) ? cmd_id : 0U;
    g_r1_link_dbg.z3_stop_accepted = accepted;
}

static void r1_link_on_mission_frame(const uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES])    /* 解析线协议帧 */
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
        uint32_t fp = R1LinkDedup_FpZ2Mission(frame7, R1_R2_CONNECT_FRAME_BYTES);

        if (R1LinkDedup_IsDuplicate(r1_link_dedup_ch_z2_mission, fp, HAL_GetTick()) != 0U)
        {
            r1_link_debug_capture_frame(frame7, rc, &wire, &z2);
            return;
        }

        r1_link_wire_to_zone2(&wire, &z2);
        r1_link_wire_to_zone2(&wire, &s_last_mission);
        s_has_new = 1U;    /* 更新标志 */
        s_frame_ok++;
    }
    else
    {
        s_frame_err++;
    }

    r1_link_debug_capture_frame(frame7, rc, &wire, (rc == 0U) ? &z2 : NULL);
}


static void r1_link_on_z3_put_frame(const uint8_t frame4[R1_LINK_Z3_PUT_FRAME_BYTES])
{
    r1_link_z3_put_cmd_t cmd;
    uint8_t rc;

    rc = r1_link_z3_put_frame_decode(frame4, &cmd);
    r1_link_debug_capture_z3_put(frame4, rc, (rc == 0U) ? &cmd : NULL);

    if (rc == 0U)
    {
        s_z3_put_ok++;
        r1_zone3_parse_from_link_z3_put(cmd.cmd_id, frame4[1]);
    }
    else
    {
        s_z3_put_err++;
    }
}

static void r1_link_on_z3_cmd_frame(const uint8_t frame5[R1_LINK_Z3_CMD_FRAME_BYTES])    /* 处理Z3指令帧 STOP / GET_KFS / 放料 */
{
    r1_link_z3_cmd_frame_t decoded;
    uint8_t rc;

    rc = r1_link_z3_cmd_frame_decode(frame5, &decoded);
    if (rc != 0U)
    {
        r1_link_debug_capture_z3_stop(frame5, rc, 0U, 0U);
        s_z3_stop_err++;
        return;
    }

    if (decoded.cmd_id == (uint8_t)APP_Z3_CMD_STOP_ACTION)
    {
        r1_link_debug_capture_z3_stop(frame5, rc, decoded.cmd_id, 1U);
        s_z3_stop_ok++;
        r1_zone3_parse_from_usart10_stop();
        return;
    }

    if (decoded.cmd_id <= (uint8_t)APP_Z3_CMD_PUT_KFS_P4 ||
        decoded.cmd_id == (uint8_t)R1_LINK_Z3_CMD_WIRE_GET_KFS_G1 ||
        decoded.cmd_id == (uint8_t)R1_LINK_Z3_CMD_WIRE_GET_KFS_G2 ||
        decoded.cmd_id == (uint8_t)APP_Z3_CMD_UP_R1)
    {
        r1_link_debug_capture_z3_stop(frame5, rc, decoded.cmd_id, 1U);
        s_z3_stop_ok++;
        r1_zone3_parse_from_link_z3_cmd(decoded.cmd_id, decoded.put_sub);
        return;
    }

    r1_link_debug_capture_z3_stop(frame5, rc, decoded.cmd_id, 0U);
    s_z3_stop_err++;
}

void R1Link_OnRxByte(uint8_t b) /* 接收 1 字节，解析各种帧 */
{
    uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES];
    uint8_t frame4_z3_put[R1_LINK_Z3_PUT_FRAME_BYTES];
    uint8_t frame5_stop[R1_LINK_Z3_CMD_FRAME_BYTES];

    if (r1_r2_connect_rx_feed_byte(&s_rx_ctx, b, frame7) != 0U)
    {
        r1_link_on_mission_frame(frame7);
        return;
    }

    if (r1_link_z3_put_rx_feed_byte(&s_z3_put_rx_ctx, b, frame4_z3_put) != 0U)
    {
        r1_link_on_z3_put_frame(frame4_z3_put);
        return;
    }

    if (r1_link_z3_cmd_rx_feed_byte(&s_z3_stop_rx_ctx, b, frame5_stop) != 0U)
    {
        r1_link_on_z3_cmd_frame(frame5_stop);
    }
}

void R1Link_Init(void)
{
    r1_r2_connect_rx_reset(&s_rx_ctx);
    r1_link_z3_put_rx_reset(&s_z3_put_rx_ctx);
    r1_link_z3_cmd_rx_reset(&s_z3_stop_rx_ctx);
    (void)memset(&s_last_mission, 0, sizeof(s_last_mission));
    (void)memset(s_last_frame7, 0, sizeof(s_last_frame7));
    (void)memset((void *)&g_r1_link_dbg, 0, sizeof(g_r1_link_dbg));
    s_has_new = 0U;
    s_has_last_frame = 0U;
    s_frame_ok = 0U;
    s_frame_err = 0U;
    s_z3_put_ok = 0U;
    s_z3_put_err = 0U;
    s_z3_stop_ok = 0U;
    s_z3_stop_err = 0U;
    R1LinkDedup_Reset();
}

void R1Link_ErrorRecover(void)
{
    r1_r2_connect_rx_reset(&s_rx_ctx);
    r1_link_z3_put_rx_reset(&s_z3_put_rx_ctx);
    r1_link_z3_cmd_rx_reset(&s_z3_stop_rx_ctx);
    R1LinkDedup_Reset();
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




uint32_t R1Link_FrameOkCount(void)    /* 线协议帧解析成功计数 */
{
    return s_frame_ok;
}

uint32_t R1Link_FrameErrCount(void)    /* 线协议帧解析失败计数 */
{
    return s_frame_err;
}



uint32_t R1Link_Z3PutOkCount(void)
{
    return s_z3_put_ok;
}

uint32_t R1Link_Z3PutErrCount(void)
{
    return s_z3_put_err;
}

uint32_t R1Link_Z3StopOkCount(void)
{
    return s_z3_stop_ok;
}

uint32_t R1Link_Z3StopErrCount(void)
{
    return s_z3_stop_err;
}

uint8_t R1Link_HasLastRxFrame(void)    /* 是否已有最后一次接收的线协议帧 */
{
    return s_has_last_frame;
}

uint8_t R1Link_CopyLastRxFrame(uint8_t frame7[R1_LINK_FRAME_BYTES])    /* 复制最后一帧线协议帧到 frame7 失败返回 0 */
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

uint8_t R1Link_SendLastRxFrameToR1(void)    /* 发送最后一帧线协议帧到 R1 失败返回 0 */
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
