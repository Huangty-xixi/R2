/**
 * @file r1_link_dedup.h
 * @brief R1 红外/线协议短窗去重（双串口或重发时业务只触发一次）
 */
#ifndef R1_LINK_DEDUP_H
#define R1_LINK_DEDUP_H

#include <stdint.h>

typedef enum
{
    r1_link_dedup_ch_z3_cmd = 0,
    r1_link_dedup_ch_z2_mission,
    r1_link_dedup_ch_count,
} r1_link_dedup_channel_t;

#ifndef R1_LINK_DEDUP_WINDOW_MS
#define R1_LINK_DEDUP_WINDOW_MS  (3000U) /* 三区指令去重窗口 3s，防止 R1 重发触发重复执行 */
#endif

void R1LinkDedup_Reset(void);

/** @return 1=重复应丢弃，0=可接受 */
uint8_t R1LinkDedup_IsDuplicate(r1_link_dedup_channel_t ch, uint32_t fingerprint, uint32_t now_ms);

uint32_t R1LinkDedup_FpZ3Cmd(uint8_t zone3_cmd_id, uint8_t put_sub);
uint32_t R1LinkDedup_FpZ2Mission(const uint8_t *frame7, uint8_t frame_len);

extern volatile uint32_t g_r1_link_dedup_drop_count[r1_link_dedup_ch_count];

#endif /* R1_LINK_DEDUP_H */
