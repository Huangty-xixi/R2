/**
 * @file odom_center_offset.h
 * @brief 雷达与车心偏置：仅四场向（0/90/180/-90）固定世界系增量，红蓝各一张表
 *
 * 车体系标定：前 +0.09 m、左 -0.12 m。场上航向用 ODOM yaw 量化到最近直角；
 * 摆头目标 86.7 等属陀螺小偏置，与本模块查表无关。
 */
#ifndef ODOM_CENTER_OFFSET_H
#define ODOM_CENTER_OFFSET_H

#include <stdint.h>

#include "upper_pc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 车心 -> 雷达，车体系标定值（用于文档/重算四向表） */
#ifndef ODOM_CENTER_OFFSET_RADAR_FWD_M
#define ODOM_CENTER_OFFSET_RADAR_FWD_M  (0.11f)
#endif
#ifndef ODOM_CENTER_OFFSET_RADAR_LEFT_M
#define ODOM_CENTER_OFFSET_RADAR_LEFT_M (-0.09f)
#endif

/** 四场向：与 YawHeadingCtrl FRONT/BACK/LEFT/RIGHT 语义一致（量化用 0/±90/180） */
typedef enum {
    ODOM_CENTER_OFFSET_DIR_FRONT = 0,
    ODOM_CENTER_OFFSET_DIR_BACK  = 1,
    ODOM_CENTER_OFFSET_DIR_LEFT  = 2,
    ODOM_CENTER_OFFSET_DIR_RIGHT = 3,
} odom_center_offset_dir_t;

/**
 * @brief ODOM yaw（度）量化到四场向之一：[-45,45]前、(45,135]左、(135,180]|(-180,-135]后、其余右
 */
odom_center_offset_dir_t odom_center_offset_dir_from_yaw_deg(float yaw_deg);

/**
 * @brief 取该场向下「车心->雷达」世界系 (dx,dy)；is_red_side 1=红 0=蓝
 */
void odom_center_offset_table_lookup_ex(uint8_t is_red_side,
                                        odom_center_offset_dir_t dir,
                                        float *dx_m,
                                        float *dy_m);

void odom_center_offset_radar_to_center_ex(uint8_t is_red_side,
                                           float radar_x_m,
                                           float radar_y_m,
                                           float yaw_deg,
                                           float *center_x_m,
                                           float *center_y_m);

void odom_center_offset_radar_to_center_by_dir_ex(uint8_t is_red_side,
                                                  float radar_x_m,
                                                  float radar_y_m,
                                                  odom_center_offset_dir_t dir,
                                                  float *center_x_m,
                                                  float *center_y_m);

void odom_center_offset_radar_to_center(float radar_x_m,
                                        float radar_y_m,
                                        float yaw_deg,
                                        float *center_x_m,
                                        float *center_y_m);

void odom_center_offset_center_to_radar_ex(uint8_t is_red_side,
                                           float center_x_m,
                                           float center_y_m,
                                           float yaw_deg,
                                           float *radar_x_m,
                                           float *radar_y_m);

void odom_center_offset_center_to_radar_by_dir_ex(uint8_t is_red_side,
                                                  float center_x_m,
                                                  float center_y_m,
                                                  odom_center_offset_dir_t dir,
                                                  float *radar_x_m,
                                                  float *radar_y_m);

void odom_center_offset_center_to_radar(float center_x_m,
                                        float center_y_m,
                                        float yaw_deg,
                                        float *radar_x_m,
                                        float *radar_y_m);

void odom_center_offset_odom_to_center(const rc_odom_t *radar_odom,
                                       float *center_x_m,
                                       float *center_y_m);

uint8_t odom_center_offset_latest_center(float *center_x_m, float *center_y_m);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_CENTER_OFFSET_H */
