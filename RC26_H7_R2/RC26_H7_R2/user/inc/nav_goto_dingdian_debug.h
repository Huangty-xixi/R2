/**
 * @file nav_goto_dingdian_debug.h
 * @brief 定点导航调试：Watch 变量 a/b 触发以当前车心为锚点的 1m 方框路径
 *
 * a=1（b=0）：(x+1,y) → (x+1,y+1) → (x,y+1) → (x,y)
 * b=1（a=0）：(x+1,y+1) → (x,y+1) → (x+1,y) → (x,y)
 * a、b 同时为 1：无效，不启动
 */
#ifndef NAV_GOTO_DINGDIAN_DEBUG_H
#define NAV_GOTO_DINGDIAN_DEBUG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Watch 可调：默认 0；置 1 触发对应路径（跑完自动清 0） */
typedef struct {
    volatile uint8_t a;
    volatile uint8_t b;
} nav_goto_dingdian_debug_t;

extern volatile nav_goto_dingdian_debug_t g_nav_goto_dingdian_debug;

/** 全自动空闲时轮询；在 odom_nav_goto_service_tick 之前调用 */
void nav_goto_dingdian_debug_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* NAV_GOTO_DINGDIAN_DEBUG_H */
