/**
 * @file common.h
 * @brief 工程内通用小工具（浮点限幅、角度折叠、单调毫秒、二维矢量限幅等）。
 */
#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 将角度（度）折到约 [-180, 180] */
float wrap_deg_180(float deg);

/** @brief 单调毫秒（HAL_GetTick） */
uint32_t common_now_ms(void);

/** @brief 将 v 限制在 [lo, hi]（需 lo <= hi） */
float clampf(float v, float lo, float hi);

/**
 * @brief 将 (vx,vy) 的模长限制在 vmax 以内，方向不变；超过则等比例缩放。
 * @param vmax 最大合速度，应 > 0
 */
void vec2_limit(float *vx, float *vy, float vmax);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */
