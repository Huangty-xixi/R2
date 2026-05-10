#include "map.h"

/* ---- 场地总尺寸 ---- */
#define FIELD_X_MIN  0
#define FIELD_X_MAX  6000
#define FIELD_Y_MIN  0
#define FIELD_Y_MAX  MAP_FIELD_Y_MAX_MM

/* ================================================================
   武馆 Martial Club
   ================================================================ */
#define MC_X_MIN  0
#define MC_X_MAX  6000
#define MC_Y_MIN  0
#define MC_Y_MAX  2000

/* -- R1 起始区 -- */
#define MC_R1_X_MIN  5000
#define MC_R1_X_MAX  6000
#define MC_R1_Y_MIN  0
#define MC_R1_Y_MAX  1000

/* -- R2 起始区 -- */
#define MC_R2_X_MIN  1000
#define MC_R2_X_MAX  1800
#define MC_R2_Y_MIN  0
#define MC_R2_Y_MAX  800

/* -- 棍架 (Staff Rack) -- */
#define MC_STAFF_X_MIN  3000
#define MC_STAFF_X_MAX  3800
#define MC_STAFF_Y_MIN  0
#define MC_STAFF_Y_MAX  300

/* -- 枪头架 (Spearhead Rack) -- */
#define MC_SPEAR_X_MIN  0
#define MC_SPEAR_X_MAX  150
#define MC_SPEAR_Y_MIN  350
#define MC_SPEAR_Y_MAX  1550

/* -- 运动区 (Motion Area) -- */
#define MC_MOTION_X_MIN  2000
#define MC_MOTION_X_MAX  5500
#define MC_MOTION_Y_MIN  2000
#define MC_MOTION_Y_MAX  4000

/* ================================================================
   梅林 Meihua Forest
   ================================================================ */

#define MF_X_MIN  0
#define MF_X_MAX  6000
#define MF_Y_MIN  2000
#define MF_Y_MAX  7400

/* -- 入口区 -- */
#define MF_ENTRY_X_MIN  1200
#define MF_ENTRY_X_MAX  4800
#define MF_ENTRY_Y_MIN  2000
#define MF_ENTRY_Y_MAX  3200

/* -- 通道 (Path Way) -- */
#define MF_PATH_X_MIN  7500
#define MF_PATH_X_MAX  13000
#define MF_PATH_Y_MIN  500
#define MF_PATH_Y_MAX  1500

/* -- 12个梅花桩 (Blocks) -- */
#define MF_BLOCK_1_X_MIN   1200
#define MF_BLOCK_1_X_MAX   2400
#define MF_BLOCK_1_Y_MIN   3200
#define MF_BLOCK_1_Y_MAX   4400

#define MF_BLOCK_2_X_MIN   2400
#define MF_BLOCK_2_X_MAX   3600
#define MF_BLOCK_2_Y_MIN   3200
#define MF_BLOCK_2_Y_MAX   4400

#define MF_BLOCK_3_X_MIN   3600
#define MF_BLOCK_3_X_MAX   4800
#define MF_BLOCK_3_Y_MIN   3200
#define MF_BLOCK_3_Y_MAX   4400

#define MF_BLOCK_4_X_MIN   1200
#define MF_BLOCK_4_X_MAX   2400
#define MF_BLOCK_4_Y_MIN   4400
#define MF_BLOCK_4_Y_MAX   5600

#define MF_BLOCK_5_X_MIN   2400
#define MF_BLOCK_5_X_MAX   3600
#define MF_BLOCK_5_Y_MIN   4400
#define MF_BLOCK_5_Y_MAX   5600

#define MF_BLOCK_6_X_MIN   3600
#define MF_BLOCK_6_X_MAX   4800
#define MF_BLOCK_6_Y_MIN   4400
#define MF_BLOCK_6_Y_MAX   5600

#define MF_BLOCK_7_X_MIN   1200
#define MF_BLOCK_7_X_MAX   2400
#define MF_BLOCK_7_Y_MIN   5600
#define MF_BLOCK_7_Y_MAX   6800

#define MF_BLOCK_8_X_MIN   2400
#define MF_BLOCK_8_X_MAX   3600
#define MF_BLOCK_8_Y_MIN   5600
#define MF_BLOCK_8_Y_MAX   6800

#define MF_BLOCK_9_X_MIN   3600
#define MF_BLOCK_9_X_MAX   4800
#define MF_BLOCK_9_Y_MIN   5600
#define MF_BLOCK_9_Y_MAX   6800

#define MF_BLOCK_10_X_MIN  1200
#define MF_BLOCK_10_X_MAX  2400
#define MF_BLOCK_10_Y_MIN  6800
#define MF_BLOCK_10_Y_MAX  8000

#define MF_BLOCK_11_X_MIN  2400
#define MF_BLOCK_11_X_MAX  3600
#define MF_BLOCK_11_Y_MIN  6800
#define MF_BLOCK_11_Y_MAX  8000

#define MF_BLOCK_12_X_MIN  3600
#define MF_BLOCK_12_X_MAX  4800
#define MF_BLOCK_12_Y_MIN  6800
#define MF_BLOCK_12_Y_MAX  8000

/* ================================================================
   Arena
   ================================================================ */

#define AR_X_MIN  0
#define AR_X_MAX  6000
#define AR_Y_MIN  9450
#define AR_Y_MAX  12000

/* -- 斜坡 (Ramp) -- */
#define AR_RAMP_X_MIN  4500
#define AR_RAMP_X_MAX  6000
#define AR_RAMP_Y_MIN  9300
#define AR_RAMP_Y_MAX  10800

/* -- 重试区 (Retry Zone) -- */
#define AR_RETRY_X_MIN  5000
#define AR_RETRY_X_MAX  6000
#define AR_RETRY_Y_MIN  11000
#define AR_RETRY_Y_MAX  12000

/* -- 已用兵器区 (Used Weapon Area) -- */
#define AR_USED_X_MIN  1000
#define AR_USED_X_MAX  2500
#define AR_USED_Y_MIN  9450
#define AR_USED_Y_MAX  9750

/* -- TTT架 (TTT Rack) -- */
#define AR_TTT_X_MIN  0
#define AR_TTT_X_MAX  135
#define AR_TTT_Y_MIN  9900
#define AR_TTT_Y_MAX  11520

void map_init(void)
{
}

bool map_rect_contains(const Zone_Rect *r, int32_t x, int32_t y)//判断矩形是否包含点
{
    return (x >= r->x_min && x <= r->x_max &&
            y >= r->y_min && y <= r->y_max);
}

Map_Location map_locate(int32_t x_mm, int32_t y_mm)//定位地图位置
{
    Map_Location loc;
    loc.zone_major = MZ_None;
    loc.zone_martial_club = MC_None;
    loc.zone_meihua_forest = MF_None;
    loc.zone_arena = AR_None;

    {
        const Zone_Rect sub[] = {
            {MC_R1_X_MIN, MC_R1_X_MAX, MC_R1_Y_MIN, MC_R1_Y_MAX},
            {MC_R2_X_MIN, MC_R2_X_MAX, MC_R2_Y_MIN, MC_R2_Y_MAX},
            {MC_STAFF_X_MIN, MC_STAFF_X_MAX, MC_STAFF_Y_MIN, MC_STAFF_Y_MAX},
            {MC_SPEAR_X_MIN, MC_SPEAR_X_MAX, MC_SPEAR_Y_MIN, MC_SPEAR_Y_MAX},
        };
        const Martial_Club_Zone zones[] = {
            MC_R1_Start_Zone, MC_R2_Start_Zone,
            MC_Staff_Rack, MC_Spearhead_Rack,
        };
        for (int i = 0; i < 4; i++) {
            if (map_rect_contains(&sub[i], x_mm, y_mm)) {
                loc.zone_major = MZ_Martial_Club;
                loc.zone_martial_club = zones[i];
                return loc;
            }
        }
    }

    {
        if (x_mm >= MC_X_MIN && x_mm <= MC_X_MAX &&
            y_mm >= MC_Y_MIN && y_mm <= MC_Y_MAX) {
            loc.zone_major = MZ_Martial_Club;
            loc.zone_martial_club = MC_Motion_Area;
            return loc;
        }
    }

    {
        const Zone_Rect sub[] = {
            {MF_PATH_X_MIN, MF_PATH_X_MAX, MF_PATH_Y_MIN, MF_PATH_Y_MAX},
            {MF_BLOCK_1_X_MIN, MF_BLOCK_1_X_MAX, MF_BLOCK_1_Y_MIN, MF_BLOCK_1_Y_MAX},
            {MF_BLOCK_2_X_MIN, MF_BLOCK_2_X_MAX, MF_BLOCK_2_Y_MIN, MF_BLOCK_2_Y_MAX},
            {MF_BLOCK_3_X_MIN, MF_BLOCK_3_X_MAX, MF_BLOCK_3_Y_MIN, MF_BLOCK_3_Y_MAX},
            {MF_BLOCK_4_X_MIN, MF_BLOCK_4_X_MAX, MF_BLOCK_4_Y_MIN, MF_BLOCK_4_Y_MAX},
            {MF_BLOCK_5_X_MIN, MF_BLOCK_5_X_MAX, MF_BLOCK_5_Y_MIN, MF_BLOCK_5_Y_MAX},
            {MF_BLOCK_6_X_MIN, MF_BLOCK_6_X_MAX, MF_BLOCK_6_Y_MIN, MF_BLOCK_6_Y_MAX},
            {MF_BLOCK_7_X_MIN, MF_BLOCK_7_X_MAX, MF_BLOCK_7_Y_MIN, MF_BLOCK_7_Y_MAX},
            {MF_BLOCK_8_X_MIN, MF_BLOCK_8_X_MAX, MF_BLOCK_8_Y_MIN, MF_BLOCK_8_Y_MAX},
            {MF_BLOCK_9_X_MIN, MF_BLOCK_9_X_MAX, MF_BLOCK_9_Y_MIN, MF_BLOCK_9_Y_MAX},
            {MF_BLOCK_10_X_MIN, MF_BLOCK_10_X_MAX, MF_BLOCK_10_Y_MIN, MF_BLOCK_10_Y_MAX},
            {MF_BLOCK_11_X_MIN, MF_BLOCK_11_X_MAX, MF_BLOCK_11_Y_MIN, MF_BLOCK_11_Y_MAX},
            {MF_BLOCK_12_X_MIN, MF_BLOCK_12_X_MAX, MF_BLOCK_12_Y_MIN, MF_BLOCK_12_Y_MAX},
        };
        const Meihua_Forest_Zone zones[] = {
            MF_Path_Way,
            MF_Block_1, MF_Block_2, MF_Block_3, MF_Block_4,
            MF_Block_5, MF_Block_6, MF_Block_7, MF_Block_8,
            MF_Block_9, MF_Block_10, MF_Block_11, MF_Block_12,
        };
        for (int i = 0; i < 13; i++) {
            if (map_rect_contains(&sub[i], x_mm, y_mm)) {
                loc.zone_major = MZ_Meihua_Forest;
                loc.zone_meihua_forest = zones[i];
                return loc;
            }
        }
    }

    {
        if (x_mm >= MF_X_MIN && x_mm <= MF_X_MAX &&
            y_mm >= MF_Y_MIN && y_mm <= MF_Y_MAX) {
            loc.zone_major = MZ_Meihua_Forest;
            loc.zone_meihua_forest = MF_Entrance_Zone;
            return loc;
        }
    }

    {
        const Zone_Rect sub[] = {
            {AR_RAMP_X_MIN, AR_RAMP_X_MAX, AR_RAMP_Y_MIN, AR_RAMP_Y_MAX},
            {AR_RETRY_X_MIN, AR_RETRY_X_MAX, AR_RETRY_Y_MIN, AR_RETRY_Y_MAX},
            {AR_USED_X_MIN, AR_USED_X_MAX, AR_USED_Y_MIN, AR_USED_Y_MAX},
            {AR_TTT_X_MIN, AR_TTT_X_MAX, AR_TTT_Y_MIN, AR_TTT_Y_MAX},
        };
        const Arena_Zone zones[] = {
            AR_Ramp, AR_Retry_Zone, AR_Used_Weapon_Area, AR_TTT_Rack,
        };
        for (int i = 0; i < 4; i++) {
            if (map_rect_contains(&sub[i], x_mm, y_mm)) {
                loc.zone_major = MZ_Arena;
                loc.zone_arena = zones[i];
                return loc;
            }
        }
    }

    {
        if (x_mm >= AR_X_MIN && x_mm <= AR_X_MAX &&
            y_mm >= AR_Y_MIN && y_mm <= AR_Y_MAX) {
            loc.zone_major = MZ_Arena;
            loc.zone_arena = AR_None;
            return loc;
        }
    }

    return loc;
}
