#ifndef __MAP_H__
#define __MAP_H__

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
   场地 map 坐标（里程计/分区）：原点为红区左下角，x 向右，y 向场上方，单位 mm。
   取值范围与 map.c 中 FIELD / MF_* 一致（本工程半幅红区 0~6000 × 0~12100）。
   ================================================================ */

#define MAP_FIELD_Y_MAX_MM   12100
#define MAP_RED_RIGHT_X_MM   6000   /* 红区靠中线一侧（+x_map 边界），与 MF_X_MAX 一致 */

/* ================================================================
   红区二区「规划坐标」(与 R1 路径、场地示意图一致，例：R2 经 1 区后从格栅上方进入)

   原点：南北中线与红区上沿（靠近 1 区 / 入口侧）的交点。
   +X_red：沿上沿指向图左（深入红区）。图上从左到右用户标注列「3 → 2 → 1」对应
           map 中梅花桩列：MF_Block_1(左列)=列3，MF_Block_2(中列)=列2，MF_Block_3(右列)=列1。
   +Y_red：指向图下（沿格栅向纵深）。
   桩顶标高仍为 200 / 400 / 600 mm（三层），与 app_zone2 中 tier 表一致。

   与场地 map_mm 互算（用于把 R1 规划点换成 odom_nav_goto 目标）：
     x_map_mm = MAP_RED_RIGHT_X_MM - x_red_mm
     y_map_mm = MAP_FIELD_Y_MAX_MM - y_red_mm
   ================================================================ */
/* 从红区规划坐标转换到map坐标 */
static inline int32_t map_x_mm_from_red_plan_mm(int32_t x_red_mm)
{
    return (int32_t)MAP_RED_RIGHT_X_MM - x_red_mm;
}

/* 从红区规划坐标转换到map坐标 */
static inline int32_t map_y_mm_from_red_plan_mm(int32_t y_red_mm)
{
    return (int32_t)MAP_FIELD_Y_MAX_MM - y_red_mm;
}

/* 从map坐标转换到红区规划坐标 */
static inline int32_t map_red_plan_x_mm_from_map_mm(int32_t x_map_mm)
{
    return (int32_t)MAP_RED_RIGHT_X_MM - x_map_mm;
}

/* 从map坐标转换到红区规划坐标 */
static inline int32_t map_red_plan_y_mm_from_map_mm(int32_t y_map_mm)
{
    return (int32_t)MAP_FIELD_Y_MAX_MM - y_map_mm;
}

/* 主区域 */
typedef enum _Major_Zone {
    MZ_None = 0,
    MZ_Martial_Club,
    MZ_Meihua_Forest,
    MZ_Arena,
} Major_Zone;

/* 武馆区域 */
typedef enum _Martial_Club_Zone {
    MC_None = 0,
    MC_R1_Start_Zone,
    MC_R2_Start_Zone,
    MC_Staff_Rack,
    MC_Spearhead_Rack,
    MC_Motion_Area,
} Martial_Club_Zone;

/* 梅林区域 */
typedef enum _Meihua_Forest_Zone {
    MF_None = 0,
    MF_Entrance_Zone,
    MF_Path_Way,
    MF_Block_1,
    MF_Block_2,
    MF_Block_3,
    MF_Block_4,
    MF_Block_5,
    MF_Block_6,
    MF_Block_7,
    MF_Block_8,
    MF_Block_9,
    MF_Block_10,
    MF_Block_11,
    MF_Block_12,
} Meihua_Forest_Zone;

/* 竞技场区域 */
typedef enum _Arena_Zone {
    AR_None = 0,
    AR_Ramp,
    AR_Retry_Zone,
    AR_Used_Weapon_Area,
    AR_TTT_Rack,
} Arena_Zone;

/* 区域矩形 */
typedef struct {
    int32_t x_min;
    int32_t x_max;
    int32_t y_min;
    int32_t y_max;
} Zone_Rect;

/* 地图位置 */
typedef struct {
    Major_Zone          zone_major;
    Martial_Club_Zone   zone_martial_club;
    Meihua_Forest_Zone  zone_meihua_forest;
    Arena_Zone          zone_arena;
} Map_Location;

/* 定位地图位置 */
Map_Location map_locate(int32_t x_mm, int32_t y_mm);

/* 判断矩形是否包含点 */
bool         map_rect_contains(const Zone_Rect *r, int32_t x, int32_t y);

/* ================================================================
   二区梅花桩（转了180.png 作业视角，与 map.c MF_BLOCK 格心 mm/1000 一致）

   红区：原点在红区左下，+x 向右（朝场外右侧），+y 向上。
         各行桩号 1-2-3 / 4-5-6 / …（靠中场列 x 小）。
   蓝区：原点在蓝区右下，+x 向左（朝场外左侧），+y 向上。
         各行桩号镜像排布（第 2 行 6-5-4），同号桩顶高度与红区相同。
   下标 [0] 占位；[1]..[12] = R1 桩号。
   ================================================================ */

#define MAP_ZONE2_PILE_TABLE_LEN 13U

extern const float MAP_RED_PILE_CX_M[MAP_ZONE2_PILE_TABLE_LEN];
extern const float MAP_RED_PILE_CY_M[MAP_ZONE2_PILE_TABLE_LEN];
extern const float MAP_BLUE_PILE_CX_M[MAP_ZONE2_PILE_TABLE_LEN];
extern const float MAP_BLUE_PILE_CY_M[MAP_ZONE2_PILE_TABLE_LEN];
extern const uint16_t MAP_ZONE2_PILE_HEIGHT_MM[MAP_ZONE2_PILE_TABLE_LEN];

/** is_red_side 非 0 用红表，否则蓝表；pile 1..12；成功返回 1 */
uint8_t map_zone2_pile_center_m(uint8_t is_red_side, uint8_t pile, float *cx_m, float *cy_m);

uint16_t map_zone2_pile_height_mm(uint8_t pile);

#endif
