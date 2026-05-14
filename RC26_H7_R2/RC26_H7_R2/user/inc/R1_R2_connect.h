/**
 * @file R1_R2_connect.h
 * @brief R1 与 R2 之间固定 7 字节链路帧（协议层，与 UART 底层无关）。
 *
 * @par 线格式（共 7 字节）
 * - 字节 0：帧头 0xAA（R1_R2_FRAME_SYNC1）
 * - 字节 1～5：载荷 40 bit，无分隔；前 28 bit 为 7 个「路径桩」半字节，后 12 bit 为 3 个「KFS 桩」半字节。
 * - 字节 6：帧尾 0xBB（R1_R2_FRAME_SYNC2）
 * - 每个桩占 4 bit，取值 1～12 表示桩 1～桩 12；0 表示该槽未使用（仅允许出现在路径或 KFS 的尾部补位，中间不能 0 后再非 0）。
 * - 每个载荷字节：高 4 bit 为前一个桩号，低 4 bit 为后一个桩号（与 payload_pack 实现一致）。
 *
 * @par 数据结构 r1_r2_mission_t（给上层用）
 * - path_n：有效路径桩个数 0～7。只读 path[0]～path[path_n-1]，不要把 path 后半段的 0 当成路径桩。
 * - kfs_n：有效 KFS 个数 0～3。只读 kfs[0]～kfs[kfs_n-1]。
 * - path[]、kfs[]：缓冲区大于线长，未用元素为 0；长度一律以 path_n、kfs_n 为准。
 *
 * @par API 分层（按需选用）
 * 1. 只编/解中间 5 字节载荷（不含 AA/BB）：
 *    - r1_r2_connect_payload_pack / r1_r2_connect_payload_unpack
 *    入参 path7[7]、kfs3[3] 为「线逻辑」固定槽，未用槽填 0。
 * 2. 整帧 7 字节（AA + 5 + BB）：
 *    - r1_r2_connect_frame_pack / r1_r2_connect_frame_unpack
 *    frame_unpack 返回 0 成功；1 帧头尾错误；2 路径半字节>12；3 KFS 半字节>12；5 路径中间出现非法 0；6 KFS 同理。
 * 3. 与 r1_r2_mission_t 互转（推荐日常收发用）：
 *    - r1_r2_connect_mission_encode：内存任务 → 7 字节线帧。
 *      若 path_n 为 1～7，则按 path[0..path_n-1] 上帧；若 path_n 为 0 或非法，则自动从 path[] 开头数到第一个 0 作为长度。kfs_n 同理（1～3 或自动扫 kfs[]）。
 *    - r1_r2_connect_mission_decode：7 字节线帧 → 填 out；成功时设置 path_n、kfs_n，且只写入有效前缀（例如线里 3,5,7,0,0,0,0 则 path_n=3，仅 path[0..2] 有意义）。out==NULL 返回 4。
 * 4. 钩子（解码成功后统一回调）：
 *    - r1_r2_connect_set_hooks：注册 on_decoded(mission, user)；传 NULL 清空。
 *    - r1_r2_connect_decode_and_dispatch：按 payload_bit_len 判断 buf 是「整 7 字节带 AA/BB」还是「仅 5 字节载荷」；仅解码成功时调用 on_decoded。
 *    - r1_r2_connect_decode_bits：同上规则写 out，不调钩子。
 * 5. UART 按字节收齐一帧：
 *    - r1_r2_connect_rx_reset：丢同步或上电时复位接收上下文。
 *    - r1_r2_connect_rx_feed_byte：每收 1 字节调用；返回 1 表示 frame7 已是一帧完整 AA..BB，再调用 mission_decode 或自行处理。
 *
 * @par 典型流程
 * - R2 发送：填好 r1_r2_mission_t（建议设 path_n/kfs_n），mission_encode 得到 frame7[7]，由 UART DMA 等发送 7 字节。
 * - R2 接收：RX 中断/任务里 feed_byte；返回 1 后 mission_decode(frame7, &m)，再把 m.path_n、m.kfs_n 与 path[]、kfs[] 拷入 app_zone2_mission_t 等。
 *
 * @note 本头含中文注释；若 Keil 源码页为 GB936 且出现乱码，请在 Keil 中将本文件另存为系统默认 ANSI/GBK 后再编译。
 */
#ifndef R1_R2_CONNECT_H
#define R1_R2_CONNECT_H

#include <stdint.h>

#define R1_R2_FRAME_SYNC1           0xAAU
#define R1_R2_FRAME_SYNC2           0xBBU
#define R1_R2_CONNECT_PATH_SLOTS    7U
#define R1_R2_CONNECT_KFS_SLOTS     3U
#define R1_R2_CONNECT_PAYLOAD_BYTES 5U
#define R1_R2_CONNECT_FRAME_BYTES   7U

#define R1_R2_CONNECT_MAX_PATH      16U
#define R1_R2_CONNECT_MAX_KFS       12U

typedef struct {
    uint8_t path_n; /* 有效路径桩数 0～7，只使用 path[0..path_n-1] */
    uint8_t kfs_n;   /* 有效 KFS 数 0～3，只使用 kfs[0..kfs_n-1] */
    uint8_t path[R1_R2_CONNECT_MAX_PATH];
    uint8_t kfs[R1_R2_CONNECT_MAX_KFS];
} r1_r2_mission_t;

typedef void (*r1_r2_hook_decoded_fn)(const r1_r2_mission_t *mission, void *user);

typedef struct {
    r1_r2_hook_decoded_fn on_decoded;
    void *user;
} r1_r2_connect_hooks_t;

/** 仅编/解 5 字节载荷（无 AA/BB）；path7、kfs3 各为线侧固定 7+3 半字节，未用填 0 */
void r1_r2_connect_payload_pack(const uint8_t path7[R1_R2_CONNECT_PATH_SLOTS],
                                const uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS],
                                uint8_t payload5[R1_R2_CONNECT_PAYLOAD_BYTES]);

void r1_r2_connect_payload_unpack(const uint8_t payload5[R1_R2_CONNECT_PAYLOAD_BYTES],
                                uint8_t path7[R1_R2_CONNECT_PATH_SLOTS],
                                uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS]);

/** 整帧 AA + 5 字节载荷 + BB */
void r1_r2_connect_frame_pack(const uint8_t path7[R1_R2_CONNECT_PATH_SLOTS],
                              const uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS],
                              uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES]);

/** 解整帧到 path7、kfs3；返回 0 成功，非 0 见文件头「frame_unpack」说明 */
uint8_t r1_r2_connect_frame_unpack(const uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES],
                                   uint8_t path7[R1_R2_CONNECT_PATH_SLOTS],
                                   uint8_t kfs3[R1_R2_CONNECT_KFS_SLOTS]);

/** 任务结构体 → 7 字节线帧；path_n/kfs_n 合法则优先用，否则按 path/kfs 遇 0 截断计数 */
void r1_r2_connect_mission_encode(const r1_r2_mission_t *m, uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES]);

/** 7 字节线帧 → 任务；填写 path_n、kfs_n 与有效前缀 */
uint8_t r1_r2_connect_mission_decode(const uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES], r1_r2_mission_t *out);

void r1_r2_connect_set_hooks(const r1_r2_connect_hooks_t *hooks);

/** 兼容：buf 为整 7 字节帧，或仅 5 字节载荷（由 payload_bit_len 区分）；结果写 out，不回调 */
void r1_r2_connect_decode_bits(const uint8_t *buf, uint16_t payload_bit_len, r1_r2_mission_t *out);

/** 同 decode_bits，解码成功且已 set_hooks 时调用 on_decoded */
void r1_r2_connect_decode_and_dispatch(const uint8_t *buf, uint16_t payload_bit_len);

typedef struct {
    uint8_t buf[R1_R2_CONNECT_FRAME_BYTES];
    uint8_t idx;
} r1_r2_connect_rx_ctx_t;

void r1_r2_connect_rx_reset(r1_r2_connect_rx_ctx_t *ctx);
/** 每收 1 字节调用；返回 1 表示 frame7 已为完整 AA..BB 一帧，再 mission_decode */
uint8_t r1_r2_connect_rx_feed_byte(r1_r2_connect_rx_ctx_t *ctx, uint8_t b, uint8_t frame7[R1_R2_CONNECT_FRAME_BYTES]);

#endif /* R1_R2_CONNECT_H */
