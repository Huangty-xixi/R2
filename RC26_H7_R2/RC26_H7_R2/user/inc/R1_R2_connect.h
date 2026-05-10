/**
 * @file R1_R2_connect.h
 * @brief R1 载荷：路径 nibbles + 12bit 全 0 分隔 + KFS nibbles（高位在先）。校验由上层自行补充。
 */
#ifndef R1_R2_CONNECT_H
#define R1_R2_CONNECT_H

#include <stdint.h>

#define R1_R2_CONNECT_MAX_PATH 16U
#define R1_R2_CONNECT_MAX_KFS 12U

typedef struct {
    uint8_t path[R1_R2_CONNECT_MAX_PATH];
    uint8_t kfs[R1_R2_CONNECT_MAX_KFS];
} r1_r2_mission_t;

void r1_r2_connect_decode_bits(const uint8_t *buf, uint16_t payload_bit_len, r1_r2_mission_t *out);

typedef void (*r1_r2_hook_decoded_fn)(const r1_r2_mission_t *mission, void *user);

typedef struct {
    r1_r2_hook_decoded_fn on_decoded;
    void *user;
} r1_r2_connect_hooks_t;

void r1_r2_connect_set_hooks(const r1_r2_connect_hooks_t *hooks);
void r1_r2_connect_decode_and_dispatch(const uint8_t *buf, uint16_t payload_bit_len);

#endif /* R1_R2_CONNECT_H */
