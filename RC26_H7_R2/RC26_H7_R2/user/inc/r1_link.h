/**
 * @file r1_link.h
 * @brief R2 三区线协议帧与红外信令帧分离   
 * @note Keil Watch 捕获收到的各种帧数据，用于调试和分析    
 */

#ifndef R1_LINK_H
#define R1_LINK_H

#include <stdint.h>
#include "app_zone2.h"
#include "R1_R2_connect.h"
#include "r1_link_z3_put.h"
#include "r1_link_sig.h"
#include "r1_link_z3_cmd.h"

#define R1_LINK_FRAME_BYTES R1_R2_CONNECT_FRAME_BYTES

/** Keil Watch 捕获收到的帧数据，用于调试和分析 */
typedef struct {
    uint8_t frame_rx[R1_LINK_FRAME_BYTES]; /**< 收到的帧数据 7 字节 AA..BB 或 4 字节 CC..DD 或 4 字节 55..AA 或 4 字节 EE..FF 或 4 字节 01..CD */
    uint8_t decode_rc;                     /**< mission_decode 或 sig_frame_decode 或 z3_put_frame_decode 或 z3_stop_frame_decode 杩斿洖鍊硷紝0=鎴愬姛 */
    uint8_t frame_tick;                    /**< 帧计数器 +1 */
    uint8_t frame_sig_rx[R1_LINK_SIG_FRAME_BYTES]; /**< 信号帧 */
    uint8_t sig_decode_rc;                 /**< 信号帧解码 0=成功 */
    uint8_t sig_tick;                      /**< 信号帧计数 +1 */
    r1_r2_mission_t wire;                  /**< 线协议帧解码结果 */
    app_zone2_mission_t zone2;             /**< zone2 解码结果 decode_rc==0 时有效 */
    uint8_t frame_z3_put_rx[R1_LINK_Z3_PUT_FRAME_BYTES]; /**< 三区线协议帧 */
    uint8_t z3_put_decode_rc;              /**< 三区线协议帧解码结果 0=成功 */
    uint8_t z3_put_tick;                   /**< 三区线协议帧计数器 +1 */
    r1_link_z3_put_cmd_t z3_put_cmd;       /**< 三区线协议帧命令 */
    uint8_t frame_z3_stop_rx[R1_LINK_Z3_CMD_FRAME_BYTES]; /**< 最近一帧 STOP EE..FF */
    uint8_t z3_stop_decode_rc;             /**< 线协议 decode 返回值，0=成功 */
    uint8_t z3_stop_tick;                  /**< 三区线协议帧计数器 +1 */
    uint8_t z3_stop_cmd_id;                /**< 帧内 cmd_id（仅 z3_stop_decode_rc==0 有效） */
    uint8_t z3_stop_accepted;              /**< 1=已通过 cmd_id=4 并 Post STOP */
} r1_link_debug_t;

extern volatile r1_link_debug_t g_r1_link_dbg;

void R1Link_Init(void);

void R1Link_ErrorRecover(void);

/** HAL 收字节回调内调用（USART10） */
void R1Link_OnRxByte(uint8_t b);

uint8_t R1Link_HasNewMission(void);

/** 读取线协议帧解码结果 失败返回 0 */
uint8_t R1Link_TakeMission(app_zone2_mission_t *out);

/** 预读取线协议帧解码结果 */
uint8_t R1Link_PeekMission(app_zone2_mission_t *out);

/** 读取线协议帧解码结果并应用 */
uint8_t R1Link_TakeAndApply(void);

/** 清除新任务标志(不消费数据)，用于zone1释放后重置以接收第二轮帧 */
void R1Link_ClearNewMission(void);

/** 是否有未读取的线协议帧 */
uint8_t R1Link_HasLastRxFrame(void);

/** 复制最后一帧线协议帧到 frame7 失败返回 0 */
uint8_t R1Link_CopyLastRxFrame(uint8_t frame7[R1_LINK_FRAME_BYTES]);

/** 发送最后一帧线协议帧到 R1 失败返回 0 */
uint8_t R1Link_SendLastRxFrameToR1(void);

uint32_t R1Link_FrameOkCount(void);

uint32_t R1Link_FrameErrCount(void);

/** 是否有未读取的红外信令帧 */

/** 读取红外信令帧解码结果 失败返回 0 */

/** 发送红外信令帧到 R1 失败返回 0 */

/** 红外信令帧解码成功计数 */

/** 红外信令帧解码失败计数 */

/** 三区线协议帧解码成功计数 */
uint8_t R1Link_HasNewSig(void);
uint8_t R1Link_TakeSig(r1_link_sig_cmd_t *out);
uint32_t R1Link_SigOkCount(void);
uint32_t R1Link_SigErrCount(void);

uint32_t R1Link_Z3PutOkCount(void);

/** 三区线协议帧解码失败计数 */
uint32_t R1Link_Z3PutErrCount(void);

uint32_t R1Link_Z3StopOkCount(void);

uint32_t R1Link_Z3StopErrCount(void);

#endif /* R1_LINK_H */
