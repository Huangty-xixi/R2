/**
 * @file odom_center_offset.c
 * @brief 雷达相对车心刚性偏置：车体系前/左常数，按红蓝区旋到世界系（与 odom_nav_goto 车体系约定一致）
 */
#include "odom_center_offset.h"

#include "app_init.h"
#include "common.h"

#include <math.h>
#include <stddef.h>

static void odom_center_offset_body_to_world(uint8_t is_red_side,
                                             float yaw_deg,
                                             float body_fwd_m,
                                             float body_left_m,
                                             float *world_dx_m,
                                             float *world_dy_m)
{
    const float psi = yaw_deg * (M_PI_F / 180.0f);
    const float s = sinf(psi);
    const float c = cosf(psi);

    if (world_dx_m == NULL || world_dy_m == NULL)
    {
        return;
    }

    /* 与 odom_nav_goto_run 红/蓝 #if 分支：前向、左向单位向量旋到世界 (dx,dy) */
    if (is_red_side != 0U)
    {
        *world_dx_m = -s * body_fwd_m - c * body_left_m;
        *world_dy_m = c * body_fwd_m + s * body_left_m;
    }
    else
    {
        *world_dx_m = s * body_fwd_m + c * body_left_m;
        *world_dy_m = c * body_fwd_m - s * body_left_m;
    }
}

void odom_center_offset_radar_to_center_ex(uint8_t is_red_side,
                                           float radar_x_m,
                                           float radar_y_m,
                                           float yaw_deg,
                                           float *center_x_m,
                                           float *center_y_m)
{
    float dx;
    float dy;

    if (center_x_m == NULL || center_y_m == NULL)
    {
        return;
    }

    odom_center_offset_body_to_world(is_red_side,
                                     yaw_deg,
                                     ODOM_CENTER_OFFSET_RADAR_FWD_M,
                                     ODOM_CENTER_OFFSET_RADAR_LEFT_M,
                                     &dx,
                                     &dy);
    *center_x_m = radar_x_m - dx;
    *center_y_m = radar_y_m - dy;
}

void odom_center_offset_radar_to_center(float radar_x_m,
                                        float radar_y_m,
                                        float yaw_deg,
                                        float *center_x_m,
                                        float *center_y_m)
{
#if APP_ZONE2_RED_SIDE
    odom_center_offset_radar_to_center_ex(1U, radar_x_m, radar_y_m, yaw_deg, center_x_m, center_y_m);
#else
    odom_center_offset_radar_to_center_ex(0U, radar_x_m, radar_y_m, yaw_deg, center_x_m, center_y_m);
#endif
}

void odom_center_offset_center_to_radar_ex(uint8_t is_red_side,
                                           float center_x_m,
                                           float center_y_m,
                                           float yaw_deg,
                                           float *radar_x_m,
                                           float *radar_y_m)
{
    float dx;
    float dy;

    if (radar_x_m == NULL || radar_y_m == NULL)
    {
        return;
    }

    odom_center_offset_body_to_world(is_red_side,
                                     yaw_deg,
                                     ODOM_CENTER_OFFSET_RADAR_FWD_M,
                                     ODOM_CENTER_OFFSET_RADAR_LEFT_M,
                                     &dx,
                                     &dy);
    *radar_x_m = center_x_m + dx;
    *radar_y_m = center_y_m + dy;
}

void odom_center_offset_center_to_radar(float center_x_m,
                                        float center_y_m,
                                        float yaw_deg,
                                        float *radar_x_m,
                                        float *radar_y_m)
{
#if APP_ZONE2_RED_SIDE
    odom_center_offset_center_to_radar_ex(1U, center_x_m, center_y_m, yaw_deg, radar_x_m, radar_y_m);
#else
    odom_center_offset_center_to_radar_ex(0U, center_x_m, center_y_m, yaw_deg, radar_x_m, radar_y_m);
#endif
}

void odom_center_offset_odom_to_center(const rc_odom_t *radar_odom,
                                       float *center_x_m,
                                       float *center_y_m)
{
    if (radar_odom == NULL)
    {
        return;
    }
    odom_center_offset_radar_to_center(radar_odom->x,
                                       radar_odom->y,
                                       radar_odom->yaw,
                                       center_x_m,
                                       center_y_m);
}

uint8_t odom_center_offset_latest_center(float *center_x_m, float *center_y_m)
{
    const rc_odom_t *p;

    if ((center_x_m == NULL) || (center_y_m == NULL))
    {
        return 0U;
    }
    if (rc_odom_is_valid() == 0U)
    {
        return 0U;
    }
    p = rc_get_latest_odom();
    odom_center_offset_odom_to_center(p, center_x_m, center_y_m);
    return 1U;
}
