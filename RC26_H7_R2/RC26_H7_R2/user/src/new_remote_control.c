/**
 * @file    new_remote_control.c
 * @brief   R2 机器人通信协议 - STM32 端实现
 *
 * ===================== 使用方法 =====================
 *   1. 调用 rc_init_with_uart(&huart10); 初始化（推荐）
 *      或调用 rc_init(uart1_putc, HAL_GetTick); 初始化（自定义方式）
 *   2. 注册回调: rc_set_odom_callback(my_odom_handler);
 *   3. 主循环中调用: rc_poll();
 */
#include "new_remote_control.h"
#include <string.h>

/* ==================== 内部状态变量 ==================== */

/** UART发送函数指针（初始化时传入） */
static void  (*uart_send)(uint8_t byte) = NULL;

/** 获取毫秒数函数指针（初始化时传入） */
static uint32_t (*get_ms)(void) = NULL;

/** UART句柄指针（用于rc_init_with_uart） */
UART_HandleTypeDef *huart_ptr = NULL;

/** UART接收缓存（用于rc_init_with_uart - USART10） */
uint8_t uart_rx_byte = 0;

/** 最近一次收到的里程计数据 */
static rc_odom_t  latest_odom;

/** 里程计数据更新时间戳（毫秒） */
static uint32_t   odom_last_ms = 0;

/** 里程计数据超时时间（毫秒），超过此时间未更新则认为数据无效 */
#define ODOM_TIMEOUT_MS  2000

/* ==================== 回调函数指针 ==================== */
static rc_odom_callback_t          cb_odom = NULL;
static rc_path_callback_t          cb_path = NULL;
static rc_kfs_callback_t           cb_kfs  = NULL;
static rc_zone_i_path_callback_t   cb_zone_i_path = NULL;

/* ==================== 接收缓冲区 ==================== */
/** 接收数据缓冲区 */
static uint8_t  rx_buf[RC_FRAME_MAX_SIZE];

/** 当前接收位置索引 */
static uint16_t rx_idx = 0;

/**
 * @brief 接收同步状态机
 * @note  0=未同步, 1=已找到SYNC1, 2=已找到SYNC2
 */
static uint8_t  rx_sync = 0;

/* ==================== 发送缓冲区 ==================== */
/** 发送数据缓冲区（用于构造发送帧） */
static uint8_t  payload[RC_FRAME_MAX_PAYLOAD];

/* ==================== 内部辅助函数 ==================== */

static uint8_t calc_chk(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t chk = cmd;
    chk ^= (uint8_t)(len & 0xFF);
    chk ^= (uint8_t)((len >> 8) & 0xFF);
    for (uint16_t i = 0; i < len; i++)
        chk ^= data[i];
    return chk;
}

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

static float unpack_float_le(const uint8_t *p)
{
    union { float f; uint32_t u; } conv;
    conv.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
           | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return conv.f;
}

/* ==================== 数据处理函数 ==================== */

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
    if (cb_odom) cb_odom(&latest_odom);
}

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

/* ==================== 外部接口函数实现 ==================== */

void rc_init(void (*send_fn)(uint8_t byte), uint32_t (*ms_fn)(void))
{
    uart_send = send_fn;
    get_ms = ms_fn;
    memset(&latest_odom, 0, sizeof(latest_odom));
    rx_idx = 0;
    rx_sync = 0;
}

static void rc_uart_send_wrapper(uint8_t byte)
{
    if (huart_ptr) {
        HAL_UART_Transmit(huart_ptr, &byte, 1, HAL_MAX_DELAY);
    }
}

void rc_init_with_uart(UART_HandleTypeDef *huart)
{
    huart_ptr = huart;
    uart_send = rc_uart_send_wrapper;
    get_ms = HAL_GetTick;
    memset(&latest_odom, 0, sizeof(latest_odom));
    rx_idx = 0;
    rx_sync = 0;
    HAL_UART_Receive_IT(huart_ptr, &uart_rx_byte, 1);
}

void rc_set_odom_callback(rc_odom_callback_t cb)           { cb_odom = cb; }
void rc_set_path_callback(rc_path_callback_t cb)           { cb_path = cb; }
void rc_set_kfs_callback(rc_kfs_callback_t cb)             { cb_kfs  = cb; }
void rc_set_zone_i_path_callback(rc_zone_i_path_callback_t cb) { cb_zone_i_path = cb; }

void rc_feed_byte(uint8_t byte)
{
    switch (rx_sync) {

    case 0:
        if (byte == RC_SYNC1) {
            rx_buf[0] = byte;
            rx_idx = 1;
            rx_sync = 1;
        }
        break;

    case 1:
        if (byte == RC_SYNC2) {
            rx_buf[1] = byte;
            rx_idx = 2;
            rx_sync = 2;
        } else {
            rx_sync = 0;
            if (byte == RC_SYNC1) {
                rx_buf[0] = byte;
                rx_idx = 1;
                rx_sync = 1;
            }
        }
        break;

    case 2:
        rx_buf[rx_idx++] = byte;
        if (rx_idx >= RC_FRAME_HEADER_SIZE) {
            uint8_t  cmd = rx_buf[2];
            uint16_t len = (uint16_t)rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
            if (len > RC_FRAME_MAX_PAYLOAD) {
                rx_sync = 0;
                break;
            }
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
}

const rc_odom_t *rc_get_latest_odom(void)
{
    return &latest_odom;
}

uint8_t rc_odom_is_valid(void)
{
    if (!get_ms) return 0;
    return (get_ms() - odom_last_ms) < ODOM_TIMEOUT_MS;
}

/* ==================== 发送函数实现 ==================== */

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
