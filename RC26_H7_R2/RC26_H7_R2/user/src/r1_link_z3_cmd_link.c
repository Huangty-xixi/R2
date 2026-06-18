/**
 * @file r1_link_z3_cmd_link.c
 * @brief USART1 收 R1 三区 EE..FF 指令帧链路层
 *
 * === 业务调用链 ===
 * HAL_UART_RxCpltCallback(huart1)
 *   -> R1LinkZ3CmdLink_OnRxByte(b)
 *   -> r1_link_z3_cmd_rx_feed_byte()                    // 5字节帧拼装
 *   -> r1_link_z3_cmd_link_on_frame(frame)
 *   -> r1_link_z3_cmd_frame_decode()                     // 校验SYNC/chk/cmd_id/put_sub
 *   -> r1_zone3_parse_from_link_z3_cmd(cmd_id, put_sub)
 *   -> r1_zone3_link_z3_cmd_wire_to_z3()                 // wire_id -> zone3 cmd
 *   -> r1_zone3_parse_post(id, raw_cmd, put_sub)
 *   -> AppZone3_PostR1Cmd(&z3)
 */

#include "r1_link_z3_cmd_link.h"

#include "r1_zone3_parse.h"

#include <string.h>

volatile r1_link_z3_cmd_link_dbg_t g_r1_link_z3_cmd_link_dbg;

static r1_link_z3_cmd_rx_ctx_t s_rx_ctx;

static uint8_t s_last_data;
static volatile uint8_t s_has_new;

static volatile uint32_t s_frame_ok;
static volatile uint32_t s_frame_err;

static void r1_link_z3_cmd_link_on_frame(const uint8_t frame[R1_LINK_Z3_CMD_FRAME_BYTES])
{
    r1_link_z3_cmd_frame_t decoded;
    uint8_t rc;

    rc = r1_link_z3_cmd_frame_decode(frame, &decoded);

    (void)memcpy((void *)g_r1_link_z3_cmd_link_dbg.frame_rx, frame,
                 (size_t)R1_LINK_Z3_CMD_FRAME_BYTES);
    g_r1_link_z3_cmd_link_dbg.decode_rc = rc;
    g_r1_link_z3_cmd_link_dbg.frame_tick++;

    if (rc == 0U)
    {
        s_last_data = decoded.cmd_id;
        g_r1_link_z3_cmd_link_dbg.cmd_id = decoded.cmd_id;
        g_r1_link_z3_cmd_link_dbg.put_sub = decoded.put_sub;
        s_has_new = 1U;
        s_frame_ok++;
        r1_zone3_parse_from_link_z3_cmd(decoded.cmd_id, decoded.put_sub);
    }
    else
    {
        g_r1_link_z3_cmd_link_dbg.cmd_id = 0U;
        g_r1_link_z3_cmd_link_dbg.put_sub = 0U;
        s_frame_err++;
    }
}

void R1LinkZ3CmdLink_Init(void)
{
    r1_link_z3_cmd_rx_reset(&s_rx_ctx);
    (void)memset((void *)&g_r1_link_z3_cmd_link_dbg, 0, sizeof(g_r1_link_z3_cmd_link_dbg));
    s_has_new = 0U;
    s_frame_ok = 0U;
    s_frame_err = 0U;
}

void R1LinkZ3CmdLink_ErrorRecover(void)
{
    r1_link_z3_cmd_rx_reset(&s_rx_ctx);
}

void R1LinkZ3CmdLink_OnRxByte(uint8_t b)
{
    uint8_t frame[R1_LINK_Z3_CMD_FRAME_BYTES];

    if (r1_link_z3_cmd_rx_feed_byte(&s_rx_ctx, b, frame) != 0U)
    {
        r1_link_z3_cmd_link_on_frame(frame);
    }
}

uint8_t R1LinkZ3CmdLink_HasNewData(void)
{
    return s_has_new;
}

uint8_t R1LinkZ3CmdLink_TakeData(uint8_t *out)
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

uint8_t R1LinkZ3CmdLink_PeekData(uint8_t *out)
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

uint32_t R1LinkZ3CmdLink_FrameOkCount(void)
{
    return s_frame_ok;
}

uint32_t R1LinkZ3CmdLink_FrameErrCount(void)
{
    return s_frame_err;
}
