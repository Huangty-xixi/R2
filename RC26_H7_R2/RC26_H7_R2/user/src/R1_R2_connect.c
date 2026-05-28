/**
 * @file R1_R2_connect.c    R1/R2 7 字节任务帧协议（见 R1_R2_connect.h）；UART 钩子调用 r1_r2_connect_rx_feed_byte。
 * @brief R1/R2 7 字节任务帧协议（见 R1_R2_connect.h）。UART 钩子调用 r1_r2_connect_rx_feed_byte。
 */
/***********************************************************************
 * USART10 7字节任务帧接收完整链路（关键函数）
 * 流程：串口字节接收 → 帧拼接 → 帧校验 → 数据解码 → 任务执行
 ***********************************************************************
 *
 * 1. 【入口】串口收到字节 → 调用：R1Link_OnRxByte()
 * 2. 【帧拼接】逐字节喂入状态机：r1_r2_connect_rx_feed_byte()
 * 3. 【帧校验】收满7字节 + 同步头正确 → 触发帧处理
 * 4. 【帧处理】调用：r1_link_on_mission_frame()
 * 5. 【数据解码】7字节帧解析为任务结构体：r1_r2_connect_mission_decode()
 * 6. 【数据就绪】标记新任务可用：s_has_new = 1
 * 7. 【上层取数】获取解码后任务：R1Link_TakeMission()
 * 8. 【最终执行】业务任务执行：app_zone2_mission_apply()
 *
 * 核心衔接点：
 * - 状态机上下文：s_rx_ctx（全局静态，保证帧连续）
 * - 帧格式：SYNC1 + 5字节有效数据 + SYNC2（固定7字节）
 * - 中断安全：读取任务时关中断保护数据
 * - 最终调用： → 执行实际命令
 ***********************************************************************/
#include "R1_R2_connect.h"

#include <string.h>

static r1_r2_connect_hooks_t s_hooks;
static uint8_t s_hook_decoded_set;

static uint8_t nibble_valid(uint8_t n)
{
    return (uint8_t)(n <= 12U);
}

void r1_r2_connect_payload_pack(const uint8_t path7[R1_R2_CONNECT_PATH_SLOTS],
                                const uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS],
                                uint8_t payload5[R1_R2_CONNECT_PAYLOAD_BYTES])
{
    payload5[0] = (uint8_t)((path7[0] << 4) | (path7[1] & 0x0FU));
    payload5[1] = (uint8_t)((path7[2] << 4) | (path7[3] & 0x0FU));
    payload5[2] = (uint8_t)((path7[4] << 4) | (path7[5] & 0x0FU));
    payload5[3] = (uint8_t)((path7[6] << 4) | (kfs3[0] & 0x0FU));
    payload5[4] = (uint8_t)((kfs3[1] << 4) | (kfs3[2] & 0x0FU));
}

void r1_r2_connect_payload_unpack(const uint8_t payload5[R1_R2_CONNECT_PAYLOAD_BYTES],
                                  uint8_t path7[R1_R2_CONNECT_PATH_SLOTS],
                                  uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS])
{
    path7[0] = (uint8_t)((payload5[0] >> 4) & 0x0FU);
    path7[1] = (uint8_t)(payload5[0] & 0x0FU);
    path7[2] = (uint8_t)((payload5[1] >> 4) & 0x0FU);
    path7[3] = (uint8_t)(payload5[1] & 0x0FU);
    path7[4] = (uint8_t)((payload5[2] >> 4) & 0x0FU);
    path7[5] = (uint8_t)(payload5[2] & 0x0FU);
    path7[6] = (uint8_t)((payload5[3] >> 4) & 0x0FU);
    kfs3[0] = (uint8_t)(payload5[3] & 0x0FU);
    kfs3[1] = (uint8_t)((payload5[4] >> 4) & 0x0FU);
    kfs3[2] = (uint8_t)(payload5[4] & 0x0FU);
}

void r1_r2_connect_frame_pack(const uint8_t path7[R1_R2_CONNECT_PATH_SLOTS],
                              const uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS],
                              uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES])
{
    frame7[0] = R1_R2_FRAME_SYNC1;
    r1_r2_connect_payload_pack(path7, kfs3, &frame7[1]);
    frame7[6] = R1_R2_FRAME_SYNC2;
}

uint8_t r1_r2_connect_frame_unpack(const uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES],
                                   uint8_t path7[R1_R2_CONNECT_PATH_SLOTS],
                                   uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS])
{
    uint8_t i;

    if (frame7[0] != R1_R2_FRAME_SYNC1 || frame7[6] != R1_R2_FRAME_SYNC2)
        return 1U;

    r1_r2_connect_payload_unpack(&frame7[1], path7, kfs3);

    for (i = 0U; i < R1_R2_CONNECT_PATH_SLOTS; i++)
    {
        if (nibble_valid(path7[i]) == 0U)
            return 2U;
    }
    {
        uint8_t seen_zero = 0U;
        for (i = 0U; i < R1_R2_CONNECT_PATH_SLOTS; i++)
        {
            if (path7[i] == 0U)
                seen_zero = 1U;
            else if (seen_zero != 0U)
                return 5U;
        }
    }
    for (i = 0U; i < R1_R2_CONNECT_KFS_SLOTS; i++)
    {
        if (nibble_valid(kfs3[i]) == 0U)
            return 3U;
    }
    {
        uint8_t seen_zero = 0U;
        for (i = 0U; i < R1_R2_CONNECT_KFS_SLOTS; i++)
        {
            if (kfs3[i] == 0U)
                seen_zero = 1U;
            else if (seen_zero != 0U)
                return 6U;
        }
    }
    return 0U;
}

void r1_r2_connect_mission_encode(const r1_r2_mission_t *m, uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES])
{
    uint8_t path7[R1_R2_CONNECT_PATH_SLOTS];
    uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS];
    uint8_t i;
    uint8_t pn;
    uint8_t kn;
    uint8_t v;

    if (m == NULL)
    {
        (void)memset(frame7, 0, (size_t)R1_R2_CONNECT_FRAME_BYTES);
        return;
    }

    pn = m->path_n;
    if (pn == 0U || pn > R1_R2_CONNECT_PATH_SLOTS)
    {
        pn = 0U;
        for (i = 0U; i < R1_R2_CONNECT_PATH_SLOTS && m->path[i] != 0U; i++)
            pn++;
    }

    for (i = 0U; i < R1_R2_CONNECT_PATH_SLOTS; i++)
    {
        if (i < pn)
        {
            v = m->path[i];
            path7[i] = (v <= 12U) ? v : 0U;
        }
        else
        {
            path7[i] = 0U;
        }
    }

    kn = m->kfs_n;
    if (kn == 0U || kn > R1_R2_CONNECT_KFS_SLOTS)
    {
        kn = 0U;
        for (i = 0U; i < R1_R2_CONNECT_KFS_SLOTS && m->kfs[i] != 0U; i++)
            kn++;
    }

    for (i = 0U; i < R1_R2_CONNECT_KFS_SLOTS; i++)
    {
        if (i < kn)
        {
            v = m->kfs[i];
            kfs3[i] = (v <= 12U) ? v : 0U;
        }
        else
        {
            kfs3[i] = 0U;
        }
    }

    r1_r2_connect_frame_pack(path7, kfs3, frame7);
}

uint8_t r1_r2_connect_mission_decode(const uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES], r1_r2_mission_t *out)
{
    uint8_t path7[R1_R2_CONNECT_PATH_SLOTS];
    uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS];
    uint8_t rc;
    uint8_t i;
    uint8_t pi;
    uint8_t ki;

    if (out == NULL)
        return 4U;

    (void)memset(out, 0, sizeof(*out));
    rc = r1_r2_connect_frame_unpack(frame7, path7, kfs3);
    if (rc != 0U)
        return rc;

    pi = 0U;
    for (i = 0U; i < R1_R2_CONNECT_PATH_SLOTS; i++)
    {
        if (path7[i] == 0U)
            break;
        if (pi < R1_R2_CONNECT_MAX_PATH)
            out->path[pi++] = path7[i];
    }
    out->path_n = pi;

    ki = 0U;
    for (i = 0U; i < R1_R2_CONNECT_KFS_SLOTS; i++)
    {
        if (kfs3[i] == 0U)
            break;
        if (ki < R1_R2_CONNECT_MAX_KFS)
            out->kfs[ki++] = kfs3[i];
    }
    out->kfs_n = ki;

    return 0U;
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
    uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES];

    if (out == NULL || buf == NULL)
        return;

    (void)memset(out, 0, sizeof(*out));

    if (payload_bit_len >= (uint16_t)(R1_R2_CONNECT_FRAME_BYTES * 8U) &&
        buf[0] == R1_R2_FRAME_SYNC1 && buf[R1_R2_CONNECT_FRAME_BYTES - 1U] == R1_R2_FRAME_SYNC2)
    {
        (void)r1_r2_connect_mission_decode(buf, out);
        return;
    }

    if (payload_bit_len >= 40U && (payload_bit_len / 8U) >= R1_R2_CONNECT_PAYLOAD_BYTES)
    {
        frame7[0] = R1_R2_FRAME_SYNC1;
        (void)memcpy(&frame7[1], buf, (size_t)R1_R2_CONNECT_PAYLOAD_BYTES);
        frame7[6] = R1_R2_FRAME_SYNC2;
        (void)r1_r2_connect_mission_decode(frame7, out);
    }
}

void r1_r2_connect_decode_and_dispatch(const uint8_t *buf, uint16_t payload_bit_len)
{
    r1_r2_mission_t m;
    uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES];
    uint8_t rc;

    if (buf == NULL)
        return;

    if (payload_bit_len >= (uint16_t)(R1_R2_CONNECT_FRAME_BYTES * 8U) &&
        buf[0] == R1_R2_FRAME_SYNC1 && buf[R1_R2_CONNECT_FRAME_BYTES - 1U] == R1_R2_FRAME_SYNC2)
    {
        rc = r1_r2_connect_mission_decode(buf, &m);
    }
    else if (payload_bit_len >= 40U && (payload_bit_len / 8U) >= R1_R2_CONNECT_PAYLOAD_BYTES)
    {
        frame7[0] = R1_R2_FRAME_SYNC1;
        (void)memcpy(&frame7[1], buf, (size_t)R1_R2_CONNECT_PAYLOAD_BYTES);
        frame7[6] = R1_R2_FRAME_SYNC2;
        rc = r1_r2_connect_mission_decode(frame7, &m);
    }
    else
    {
        return;
    }

    if (rc == 0U && s_hook_decoded_set != 0U && s_hooks.on_decoded != NULL)
        s_hooks.on_decoded(&m, s_hooks.user);
}

void r1_r2_connect_rx_reset(r1_r2_connect_rx_ctx_t *ctx)
{
    if (ctx == NULL)
        return;
    ctx->idx = 0U;
    (void)memset(ctx->buf, 0, sizeof(ctx->buf));
}

uint8_t r1_r2_connect_rx_feed_byte(r1_r2_connect_rx_ctx_t *ctx, uint8_t b, uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES])    //每收 1 字节调用；返回 1 表示 frame7 已收齐
{
    if (ctx == NULL || frame7 == NULL)
        return 0U;

    if (ctx->idx == 0U)
    {
        if (b == R1_R2_FRAME_SYNC1)
        {
            ctx->buf[0] = b;
            ctx->idx = 1U;  
        }
        return 0U;
    }

    if (ctx->idx < R1_R2_CONNECT_FRAME_BYTES)
    {
        ctx->buf[ctx->idx] = b;
        ctx->idx++;

        if (ctx->idx == R1_R2_CONNECT_FRAME_BYTES)
        {
            ctx->idx = 0U;
            if (ctx->buf[0] == R1_R2_FRAME_SYNC1 && ctx->buf[R1_R2_CONNECT_FRAME_BYTES - 1U] == R1_R2_FRAME_SYNC2)
            {
                (void)memcpy(frame7, ctx->buf, (size_t)R1_R2_CONNECT_FRAME_BYTES);
                return 1U;   //返回 1 表示 frame7 已收齐
            }
            if (b == R1_R2_FRAME_SYNC1)
            {
                ctx->buf[0] = b;
                ctx->idx = 1U;
            }
        }
        return 0U;
    }

    return 0U;
}
