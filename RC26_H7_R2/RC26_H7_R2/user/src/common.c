/**
 * @file common.c
 * @brief @ref common.h й╣ож
 */
#include "common.h"

#include "main.h"

#include <math.h>

float wrap_deg_180(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

uint32_t common_now_ms(void)
{
    return HAL_GetTick();
}

float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

void vec2_limit(float *vx, float *vy, float vmax)
{
    const float mag = sqrtf((*vx) * (*vx) + (*vy) * (*vy));

    if (mag > vmax && mag > 1e-6f)
    {
        const float s = vmax / mag;
        *vx *= s;
        *vy *= s;
    }
}
