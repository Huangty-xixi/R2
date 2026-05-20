/**
 * @file odom_center_offset.h
 * @brief 雷达 ODOM 与车体中心刚性偏置（车体系前/左常数，红蓝区世界系映射与 odom_nav_goto 一致）
 */
#ifndef ODOM_CENTER_OFFSET_H
#define ODOM_CENTER_OFFSET_H

#include <stdint.h>

#include "upper_pc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 车心 -> 雷达，车体系：前向（米），左向（米）；负左向表示雷达在中心右侧 */
#ifndef ODOM_CENTER_OFFSET_RADAR_FWD_M
#define ODOM_CENTER_OFFSET_RADAR_FWD_M  (0.09f)
#endif
#ifndef ODOM_CENTER_OFFSET_RADAR_LEFT_M
#define ODOM_CENTER_OFFSET_RADAR_LEFT_M (-0.12f)
#endif

/**
 * @brief 雷达世界坐标 + 航向 -> 车心世界坐标
 * @param is_red_side 1=红区半场，0=蓝区；与 APP_ZONE2_RED_SIDE / handle_odom 一致
 */
void odom_center_offset_radar_to_center_ex(uint8_t is_red_side,
                                           float radar_x_m,
                                           float radar_y_m,
                                           float yaw_deg,
                                           float *center_x_m,
                                           float *center_y_m);

/** 使用编译期 APP_ZONE2_RED_SIDE 选择红/蓝映射 */
void odom_center_offset_radar_to_center(float radar_x_m,
                                        float radar_y_m,
                                        float yaw_deg,
                                        float *center_x_m,
                                        float *center_y_m);

/**
 * @brief 车心世界坐标 + 航向 -> 雷达世界坐标（导航目标为车心时可选）
 */
void odom_center_offset_center_to_radar_ex(uint8_t is_red_side,
                                           float center_x_m,
                                           float center_y_m,
                                           float yaw_deg,
                                           float *radar_x_m,
                                           float *radar_y_m);

void odom_center_offset_center_to_radar(float center_x_m,
                                        float center_y_m,
                                        float yaw_deg,
                                        float *radar_x_m,
                                        float *radar_y_m);

/** 从 ODOM 结构体算车心；out 指针不可为 NULL */
void odom_center_offset_odom_to_center(const rc_odom_t *radar_odom,
                                       float *center_x_m,
                                       float *center_y_m);

/**
 * @brief 最新有效 ODOM -> 车心
 * @return 1 成功，0 ODOM 无效或输出指针为空
 */
uint8_t odom_center_offset_latest_center(float *center_x_m, float *center_y_m);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_CENTER_OFFSET_H */
