/**
 * @file r1_usart3_proto.c
 * @brief EE + cmd_id(1B) + chk(EE^cmd_id) + FF  【cmd_id 1~5 = 左/中/右/STOP/上R1】
 */
/*---------------------------------------------------------------------
 * USART3 4字节信令 接收链路（代码级衔接）
 * 1. 串口中断收到字节 → HAL_UART_RxCpltCallback()
 * 2. 调用 R1Usart3Link_OnRxByte() 入口函数
 * 3. 调用 r1_usart3_rx_feed_byte() 逐字节拼帧
 * 4. 收满4字节 + 头尾同步正确 → return 1
 * 5. if() 判断成立 → 执行 r1_usart3_link_on_frame()
 * 6. 调用 r1_usart3_frame_decode() 解码数据
 * 7. 解码成功 rc=0 → 执行 r1_zone3_parse_from_usart3() 最终指令
 *---------------------------------------------------------------------*/

#include "r1_usart3_proto.h"

#include <string.h>

static uint8_t r1_usart3_calc_chk(uint8_t data)
{
    return (uint8_t)(R1_USART3_SYNC1 ^ data);
}

void r1_usart3_rx_reset(r1_usart3_rx_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }
    ctx->idx = 0U;
    (void)memset(ctx->buf, 0, sizeof(ctx->buf));
}

void r1_usart3_frame_pack(uint8_t data, uint8_t frame4[R1_USART3_FRAME_BYTES])
{
    if (frame4 == NULL || data == 0U || data > R1_USART3_WIRE_CMD_ID_MAX)
    {
        return;
    }

    frame4[0] = R1_USART3_SYNC1;
    frame4[1] = data;
    frame4[2] = r1_usart3_calc_chk(data);
    frame4[3] = R1_USART3_SYNC2;
}

uint8_t r1_usart3_frame_decode(const uint8_t frame4[R1_USART3_FRAME_BYTES], uint8_t *out_data)
{
    if (frame4 == NULL || out_data == NULL)
    {
        return 4U;
    }

    if (frame4[0] != R1_USART3_SYNC1 || frame4[3] != R1_USART3_SYNC2)
    {
        return 1U;
    }

    if (frame4[2] != r1_usart3_calc_chk(frame4[1]))
    {
        return 2U;
    }

    if (frame4[1] == 0U || frame4[1] > R1_USART3_WIRE_CMD_ID_MAX)
    {
        return 3U;
    }

    *out_data = frame4[1];
    return 0U;
}

uint8_t r1_usart3_rx_feed_byte(r1_usart3_rx_ctx_t *ctx, uint8_t b, uint8_t frame4[R1_USART3_FRAME_BYTES])
{
    if (ctx == NULL || frame4 == NULL)
    {
        return 0U;
    }

    if (ctx->idx == 0U)
    {
        if (b == R1_USART3_SYNC1)
        {
            ctx->buf[0] = b;
            ctx->idx = 1U;
        }
        return 0U;
    }

    if (ctx->idx < R1_USART3_FRAME_BYTES)
    {
        ctx->buf[ctx->idx] = b;
        ctx->idx++;

        if (ctx->idx == R1_USART3_FRAME_BYTES)
        {
            ctx->idx = 0U;
            if (ctx->buf[0] == R1_USART3_SYNC1 && ctx->buf[R1_USART3_FRAME_BYTES - 1U] == R1_USART3_SYNC2)
            {
                (void)memcpy(frame4, ctx->buf, (size_t)R1_USART3_FRAME_BYTES);
                return 1U;
            }
            if (b == R1_USART3_SYNC1)
            {
                ctx->buf[0] = b;
                ctx->idx = 1U;
            }
        }
        return 0U;
    }

    return 0U;
}
