/**
 * @file rc_odom_snap.h
 * @brief 上位机里程计快照：协议栈在 rc_init 时已注册 RcOdomSnap_OnRcOdom；其它模块用 RcOdomSnap_Read 读取。
 *
 * 若仍需自定义 ODOM 回调，可在 rc_init 之后调用 rc_set_odom_callback；注意会覆盖默认注册，
 * 请在自定义回调内先调用 RcOdomSnap_OnRcOdom(odom) 再写业务逻辑。
 */
#ifndef RC_ODOM_SNAP_H
#define RC_ODOM_SNAP_H

#include "upper_pc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 与 rc_odom_callback_t 同签名，供 rc_set_odom_callback 使用；收到 ODOM 时写入快照
 */
void RcOdomSnap_OnRcOdom(const rc_odom_t *odom);

/**
 * @brief 拷贝最近一次快照（从未收到 ODOM 前内容为零；无新包时仍为上一帧）
 */
void RcOdomSnap_Read(rc_odom_t *out);

#ifdef __cplusplus
}
#endif

#endif
