/**
 * @file    upper_pc_protocol.c
 * @brief   R2 串口协议解析 — STM32 端实现（工程内文件名：上位机协议）
 *
 * 用法:
 *   1. rc_init(uart1_putc, HAL_GetTick)；ODOM 帧会更新内部 latest_odom，读数用 rc_get_latest_odom / rc_odom_is_valid
 *   2. 如需 ODOM 回调，rc_init 后 rc_set_odom_callback(cb)；未注册则仅更新 latest_odom
 *   3. UART RX 或虚拟串口收包处: rc_feed_byte(rx_data);
 *   4. 主循环: rc_poll();
 */
#include "upper_pc_protocol.h"
#include <string.h>

/* ---------- 内部状态 ---------- */
static void  (*uart_send)(uint8_t byte) = NULL;
static uint32_t (*get_ms)(void) = NULL;

static rc_odom_t latest_odom;
static uint32_t   odom_last_ms = 0;
#define ODOM_TIMEOUT_MS  2000

static rc_odom_callback_t          cb_odom = NULL;
static rc_path_callback_t          cb_path = NULL;
static rc_kfs_callback_t           cb_kfs  = NULL;
static rc_zone_i_path_callback_t   cb_zone_i_path = NULL;

/* 接收缓冲区 */
static uint8_t  rx_buf[RC_FRAME_MAX_SIZE];
static uint16_t rx_idx = 0;
static uint8_t  rx_sync = 0;  /* 0=找同步 1=找到 SYNC1 2=找到 SYNC2 */

/* 临时解析缓冲区 */
static uint8_t  payload[RC_FRAME_MAX_PAYLOAD];

/* ---------- 内部函数 ---------- */
//异或校验
static uint8_t calc_chk(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t chk = cmd;
    chk ^= (uint8_t)(len & 0xFF);
    chk ^= (uint8_t)((len >> 8) & 0xFF);
    for (uint16_t i = 0; i < len; i++)
        chk ^= data[i];
    return chk;
}

//发送帧函数（单帧发送）
static void send_frame(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    if (!uart_send) return;
    uint8_t chk = calc_chk(cmd, data, len);
    uart_send(RC_SYNC1);
    uart_send(RC_SYNC2);
    uart_send(cmd);
    uart_send((uint8_t)(len & 0xFF));
    uart_send((uint8_t)((len >> 8) & 0xFF));
    for (uint16_t i = 0; i < len; i++)
        uart_send(data[i]);
    uart_send(chk);
}

//小端浮点数转换
static float unpack_float_le(const uint8_t *p)
{
    union { float f; uint32_t u; } conv;
    conv.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
           | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return conv.f;
}

//小端浮点数打包
static void pack_float_le(float f, uint8_t *out)
{
    union { float f; uint32_t u; } conv;
    conv.f = f;
    out[0] = (uint8_t)(conv.u & 0xFF);
    out[1] = (uint8_t)((conv.u >> 8) & 0xFF);
    out[2] = (uint8_t)((conv.u >> 16) & 0xFF);
    out[3] = (uint8_t)((conv.u >> 24) & 0xFF);
}

//里程计处理
static void handle_odom(const uint8_t *data, uint16_t len)
{
    if (len < RC_ODOM_PAYLOAD_SIZE) return;
    latest_odom.x     = unpack_float_le(data);
    latest_odom.y     = unpack_float_le(data + 4);
    latest_odom.z     = unpack_float_le(data + 8);
    latest_odom.roll  = unpack_float_le(data + 12);
    latest_odom.pitch = unpack_float_le(data + 16);
    latest_odom.yaw   = unpack_float_le(data + 20);
    odom_last_ms = get_ms ? get_ms() : 0;
    if (cb_odom) {
        cb_odom((const rc_odom_t *)&latest_odom);
    }
}

//路径处理
static void handle_path(const uint8_t *data, uint16_t len)
{
    if (len < 1) return;
    rc_path_t path;
    path.num = data[0];
    if (path.num > 16) path.num = 16;
    uint16_t pos = 1;
    for (uint8_t i = 0; i < path.num && pos + 8 <= len; i++) {
        path.points[i].x = unpack_float_le(data + pos);
        path.points[i].y = unpack_float_le(data + pos + 4);
        pos += 8;
    }
    if (cb_path) cb_path(&path);
}

//KFS处理
static void handle_kfs(const uint8_t *data, uint16_t len)
{
    if (len < 1) return;
    rc_kfs_t kfs;
    kfs.num = data[0];
    if (kfs.num > 8) kfs.num = 8;
    uint16_t pos = 1;
    for (uint8_t i = 0; i < kfs.num && pos + 13 <= len; i++) {
        kfs.detections[i].id = data[pos++];
        kfs.detections[i].x  = unpack_float_le(data + pos); pos += 4;
        kfs.detections[i].y  = unpack_float_le(data + pos); pos += 4;
        kfs.detections[i].z  = unpack_float_le(data + pos); pos += 4;
    }
    if (cb_kfs) cb_kfs(&kfs);
}

//I区路径处理
static void handle_zone_i_path(const uint8_t *data, uint16_t len)
{
    if (len < 3) return;
    rc_zone_i_path_t zp;
    zp.start_block = data[0];
    zp.end_block   = data[1];
    zp.num_blocks  = data[2];
    if (zp.num_blocks > 32) zp.num_blocks = 32;
    uint16_t pos = 3;
    for (uint8_t i = 0; i < zp.num_blocks && pos < len; i++)
        zp.block_ids[i] = data[pos++];
    if (cb_zone_i_path) cb_zone_i_path(&zp);
}

//帧分发
static void dispatch_frame(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    switch (cmd) {
    case RC_CMD_ODOM:        handle_odom(data, len);        break;
    case RC_CMD_PATH:        handle_path(data, len);        break;
    case RC_CMD_KFS:         handle_kfs(data, len);         break;
    case RC_CMD_ZONE_I_PATH: handle_zone_i_path(data, len); break;
    default: break;
    }
}

/* ---------- 公开 API ---------- */

void rc_init(void (*send_fn)(uint8_t byte), uint32_t (*ms_fn)(void))
{
    uart_send = send_fn;
    get_ms = ms_fn;
    (void)memset((void *)&latest_odom, 0, sizeof(latest_odom));
    rx_idx = 0;
    rx_sync = 0;
}

void rc_set_odom_callback(rc_odom_callback_t cb)           { cb_odom = cb; }
void rc_set_path_callback(rc_path_callback_t cb)           { cb_path = cb; }
void rc_set_kfs_callback(rc_kfs_callback_t cb)             { cb_kfs  = cb; }
void rc_set_zone_i_path_callback(rc_zone_i_path_callback_t cb) { cb_zone_i_path = cb; }

void rc_feed_byte(uint8_t byte)
{
    switch (rx_sync) {
    case 0:
        if (byte == RC_SYNC1) { rx_buf[0] = byte; rx_idx = 1; rx_sync = 1; }
        break;
    case 1:
        if (byte == RC_SYNC2) { rx_buf[1] = byte; rx_idx = 2; rx_sync = 2; }
        else { rx_sync = 0; if (byte == RC_SYNC1) { rx_buf[0] = byte; rx_idx = 1; rx_sync = 1; } }
        break;
    case 2:
        rx_buf[rx_idx++] = byte;
        if (rx_idx >= RC_FRAME_HEADER_SIZE) {
            uint8_t  cmd = rx_buf[2];
            uint16_t len = (uint16_t)rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
            if (len > RC_FRAME_MAX_PAYLOAD) { rx_sync = 0; break; }
            uint16_t frame_size = RC_FRAME_HEADER_SIZE + len + 1;
            if (rx_idx >= frame_size) {
                uint8_t chk = calc_chk(cmd, rx_buf + RC_FRAME_HEADER_SIZE, len);
                if (chk == rx_buf[frame_size - 1]) {
                    dispatch_frame(cmd, rx_buf + RC_FRAME_HEADER_SIZE, len);
                }
                rx_sync = 0;
            }
        }
        break;
    }
}

void rc_poll(void)
{
    /* 可在主循环中扩展，如心跳检测 */
}

const rc_odom_t *rc_get_latest_odom(void)
{
    return (const rc_odom_t *)&latest_odom;
}

uint8_t rc_odom_is_valid(void)
{
    if (!get_ms) return 0;
    return (get_ms() - odom_last_ms) < ODOM_TIMEOUT_MS;
}

/* ---------- 发送函数 ---------- */

void rc_send_ack(uint8_t cmd, uint8_t code)
{
    uint8_t pld[2] = { cmd, code };
    send_frame(RC_CMD_ACK, pld, 2);
}

void rc_send_status(rc_state_t state)
{
    uint8_t pld[1] = { (uint8_t)state };
    send_frame(RC_CMD_STATUS, pld, 1);
}

void rc_send_zone_i_info(uint8_t num, const rc_zone_i_kfs_t *kfs_list)
{
    if (num > 16) num = 16;
    uint8_t pld[RC_FRAME_MAX_PAYLOAD];
    pld[0] = num;
    for (uint8_t i = 0; i < num; i++) {
        pld[1 + i * 2]     = kfs_list[i].block_id;
        pld[1 + i * 2 + 1] = kfs_list[i].kfs_type;
    }
    send_frame(RC_CMD_ZONE_I_INFO, pld, 1 + num * 2);
}

void rc_send_dock_ok(void)
{
    send_frame(RC_CMD_DOCK_OK, NULL, 0);
}

void rc_send_go_zone_i(void)
{
    send_frame(RC_CMD_GO_ZONE_I, NULL, 0);
}
