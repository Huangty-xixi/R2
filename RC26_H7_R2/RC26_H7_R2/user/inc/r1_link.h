/**
 * @file r1_link.h
 * @brief R2 经 USART10 收 R1 七字节任务帧与四字节信令帧，解码缓存供业务层取用。
 * @note 本头含中文注释；若 Keil 源码页为 GB936 且出现乱码，请在 Keil 中将本文件另存为系统默认 ANSI/GBK 后再编译。
 */
// 用法示例：
// app_zone2_mission_t m;
// if (R1Link_TakeMission(&m))
//     app_zone2_mission_apply(&m);
//
// if (R1Link_TakeAndApply()) { /* 已 apply */ }

#ifndef R1_LINK_H
#define R1_LINK_H

#include <stdint.h>
#include "app_zone2.h"
#include "R1_R2_connect.h"
#include "r1_link_sig.h"

#define R1_LINK_FRAME_BYTES R1_R2_CONNECT_FRAME_BYTES

/** Keil Watch：最近一帧解码前线数据与解码后任务快照 */
typedef struct {
    uint8_t frame_rx[R1_LINK_FRAME_BYTES]; /**< 解码前：收齐的 7 字节 AA..BB */
    uint8_t decode_rc;                     /**< mission_decode 返回值，0=成功 */
    uint8_t frame_tick;                    /**< 每解一帧 +1（调试用） */
    r1_r2_mission_t wire;                  /**< 解码后：协议层 */
    app_zone2_mission_t zone2;             /**< 解码后：转 zone2（仅 decode_rc==0 有效） */
    uint8_t frame_sig_rx[R1_LINK_SIG_FRAME_BYTES]; /**< 最近一帧信令 CC..DD */
    uint8_t sig_decode_rc;                 /**< sig 解码返回值，0=成功 */
    uint8_t sig_tick;                      /**< 每解一帧信令 +1 */
} r1_link_debug_t;

extern volatile r1_link_debug_t g_r1_link_dbg;

void R1Link_Init(void);

void R1Link_ErrorRecover(void);

/** HAL 收字节回调内调用（USART10） */
void R1Link_OnRxByte(uint8_t b);

uint8_t R1Link_HasNewMission(void);

/** 取走新任务到 out；无新任务或 out==NULL 返回 0 */
uint8_t R1Link_TakeMission(app_zone2_mission_t *out);

/** 窥视新任务到 out，不清标志 */
uint8_t R1Link_PeekMission(app_zone2_mission_t *out);

/** 取走新任务并 app_zone2_mission_apply；成功返回 1 */
uint8_t R1Link_TakeAndApply(void);

/** 是否有已收齐的最近一帧 7 字节线数据（与解码成败无关） */
uint8_t R1Link_HasLastRxFrame(void);

/** 复制最近收齐的 7 字节到 frame7；frame7 为缓冲区；返回 1 表示复制成功 */
uint8_t R1Link_CopyLastRxFrame(uint8_t frame7[R1_LINK_FRAME_BYTES]);

/** 将最近收齐的 7 字节经 USART10 发回 R1；成功返回 1 */
uint8_t R1Link_SendLastRxFrameToR1(void);

uint32_t R1Link_FrameOkCount(void);

uint32_t R1Link_FrameErrCount(void);

/** 是否有未取走的 R1 释放信令 */
uint8_t R1Link_HasNewSig(void);

/** 取走最新信令；无新信令或 out==NULL 返回 0 */
uint8_t R1Link_TakeSig(r1_link_sig_cmd_t *out);

/** R2 经 USART10 发释放信令给 R1；成功返回 1 */
uint8_t R1Link_SendSig(r1_link_sig_cmd_t cmd);

/** 信令帧接收成功次数 */
uint32_t R1Link_SigOkCount(void);

/** 信令帧接收失败次数 */
uint32_t R1Link_SigErrCount(void);

#endif /* R1_LINK_H */
