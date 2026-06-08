/**
 * @file odom_center_offset.c
 * @brief 四场向固定表：蓝/红各 (dx,dy)，由车体系前0.09、左-0.12 在 0/90/180/-90 算出
 */
#include "odom_center_offset.h"

#include "app_init.h"
#include "common.h"

#include <stddef.h>

/* 车心 -> 雷达 世界 (dx,dy)；索引为 FRONT/BACK/LEFT/RIGHT */
static const float s_blue_dx_m[4] = {-0.13f, 0.13f, 0.07f, -0.07f};
static const float s_blue_dy_m[4] = {0.07f, -0.07f, 0.13f, -0.13f};

/* 红区：与旧红区旋转关系在 FRONT/BACK/LEFT/RIGHT 四向一致 */
static const float s_red_dx_m[4] = {0.13f, -0.13f, -0.07f, 0.07f};
static const float s_red_dy_m[4] = {0.07f, -0.07f, 0.13f, -0.13f};

odom_center_offset_dir_t odom_center_offset_dir_from_yaw_deg(float yaw_deg)
{
    const float y = wrap_deg_180(yaw_deg);

    if (y > 135.0f || y <= -135.0f)
    {
        return ODOM_CENTER_OFFSET_DIR_BACK;
    }
    if (y > 45.0f)
    {
        return ODOM_CENTER_OFFSET_DIR_LEFT;
    }
    if (y > -45.0f)
    {
        return ODOM_CENTER_OFFSET_DIR_FRONT;
    }
    return ODOM_CENTER_OFFSET_DIR_RIGHT;
}

void odom_center_offset_table_lookup_ex(uint8_t is_red_side,
                                        odom_center_offset_dir_t dir,
                                        float *dx_m,
                                        float *dy_m)
{
    uint8_t i;

    if (dx_m == NULL || dy_m == NULL)
    {
        return;
    }
    if ((unsigned)dir > (unsigned)ODOM_CENTER_OFFSET_DIR_RIGHT)
    {
        dir = ODOM_CENTER_OFFSET_DIR_FRONT;
    }
    i = (uint8_t)dir;
    if (is_red_side != 0U)
    {
        *dx_m = s_red_dx_m[i];
        *dy_m = s_red_dy_m[i];
    }
    else
    {
        *dx_m = s_blue_dx_m[i];
        *dy_m = s_blue_dy_m[i];
    }
}

void odom_center_offset_radar_to_center_by_dir_ex(uint8_t is_red_side,
                                                  float radar_x_m,
                                                  float radar_y_m,
                                                  odom_center_offset_dir_t dir,
                                                  float *center_x_m,
                                                  float *center_y_m)
{
    float dx;
    float dy;

    if (center_x_m == NULL || center_y_m == NULL)
    {
        return;
    }
    odom_center_offset_table_lookup_ex(is_red_side, dir, &dx, &dy);
    *center_x_m = radar_x_m - dx;
    *center_y_m = radar_y_m - dy;
}

void odom_center_offset_radar_to_center_ex(uint8_t is_red_side,
                                           float radar_x_m,
                                           float radar_y_m,
                                           float yaw_deg,
                                           float *center_x_m,
                                           float *center_y_m)
{
    const odom_center_offset_dir_t dir = odom_center_offset_dir_from_yaw_deg(yaw_deg);

    odom_center_offset_radar_to_center_by_dir_ex(is_red_side,
                                                 radar_x_m,
                                                 radar_y_m,
                                                 dir,
                                                 center_x_m,
                                                 center_y_m);
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

void odom_center_offset_center_to_radar_by_dir_ex(uint8_t is_red_side,
                                                  float center_x_m,
                                                  float center_y_m,
                                                  odom_center_offset_dir_t dir,
                                                  float *radar_x_m,
                                                  float *radar_y_m)
{
    float dx;
    float dy;

    if (radar_x_m == NULL || radar_y_m == NULL)
    {
        return;
    }
    odom_center_offset_table_lookup_ex(is_red_side, dir, &dx, &dy);
    *radar_x_m = center_x_m + dx;
    *radar_y_m = center_y_m + dy;
}

void odom_center_offset_center_to_radar_ex(uint8_t is_red_side,
                                           float center_x_m,
                                           float center_y_m,
                                           float yaw_deg,
                                           float *radar_x_m,
                                           float *radar_y_m)
{
    const odom_center_offset_dir_t dir = odom_center_offset_dir_from_yaw_deg(yaw_deg);

    odom_center_offset_center_to_radar_by_dir_ex(is_red_side,
                                                 center_x_m,
                                                 center_y_m,
                                                 dir,
                                                 radar_x_m,
                                                 radar_y_m);
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
