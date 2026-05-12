/**
 * @file app_zone2.h
 * @brief 二区梅花桩流程：抬车上台面、走路径、夹隔壁格秘籍、换路径下一格时再对齐车头和高度。
 *
 * path[] / kfs[]：红区标号同总图 34980efe1d98381ef7d7d0f5281d48a5.png（3,2,1 / 6,5,4 / …）；蓝区 1..12 与 mf 逐行一致。不下发 0 结尾时填 path_n / kfs_n；为 0 时仍按数组遇 0 截断。
 */
#ifndef APP_ZONE2_H
#define APP_ZONE2_H

#include <stdint.h>

#include "odom_nav_goto.h"

/**
 * 红/蓝半场抽签：当前二区逻辑与坐标按哪一侧写死。
 * 1 = 红区半幅（默认）；0 = 蓝区半幅——app_zone2.c 里会对梅林目标点 x 做镜像（见 APP_ZONE2_MIRROR_X_M）。
 * 红区 path/kfs 桩号为示意图编号（顶行 3、2、1…）；蓝区 R1 发号为 1..12 与物理格逐行一致，且桩顶高度按该桩号查表（与红区同一 s_pile_height_mm，下标即 mf）。
 * 可在工程选项里 -DAPP_ZONE2_RED_SIDE=0，或改此处默认值。
 * upper_pc_protocol.c 中 ODOM（雷达融合位姿）xy 解包与本宏一致：红 x=-p1/y=p0，蓝 x=p1/y=p0。
 */
#ifndef APP_ZONE2_RED_SIDE
#define APP_ZONE2_RED_SIDE 1
#endif

/** 与红区 map 半幅 x 0~6000mm 对应：目标 x_m 蓝侧镜像为 (MIRROR_X_M - x_m)。与 map.h MAP_RED_RIGHT_X_MM/1000 一致时应为 6.0f */
#ifndef APP_ZONE2_MIRROR_X_M
#define APP_ZONE2_MIRROR_X_M (6.0f)
#endif

#define APP_ZONE2_MAX_PATH 16U
#define APP_ZONE2_MAX_KFS  12U

/** 与 @ref odom_nav_goto_err_t 一致，便于 nav_poll 直接返回 @ref odom_nav_goto_run 的码 */
typedef odom_nav_goto_err_t app_zone2_nav_poll_result_t;

/**
 * 车头相对场地的前后左右；钩子里再换算成你要的 yaw。
 * SKIP = 不叫车往四向里转（一区取台面秘籍时用得多：姿态交给取件那段代码自己管）。
 */
typedef enum {
    APP_ZONE2_FIELD_FACE_SKIP = 0,
    APP_ZONE2_FIELD_FRONT,
    APP_ZONE2_FIELD_BACK,
    APP_ZONE2_FIELD_LEFT,
    APP_ZONE2_FIELD_RIGHT,
} app_zone2_field_dir_t;

/**
 * 邻格取秘籍：当前站立 path 桩顶 tier 与秘籍桩 tier 的高低关系（传给 request_get_kfs）。
 */
typedef enum {
    APP_ZONE2_GET_KFS_HIGH_TO_LOW = 0, /* 高桩取低：站立桩顶高于邻格秘籍桩 */
    APP_ZONE2_GET_KFS_LOW_TO_HIGH,     /* 低桩取高 */
} app_zone2_get_kfs_rel_t;

/**
 * 注册二区回调（上电或进入流程前调用一次即可）。未实现的指针可传 NULL，对应步骤不会调用。
 */
void app_zone2_init_hooks(
    void (*nav_set_target)(float x_m, float y_m),
    app_zone2_nav_poll_result_t (*nav_poll)(void),
    void (*request_mount_pile)(void),
    void (*request_dismount_pile)(void),
    void (*request_face_field_dir)(app_zone2_field_dir_t dir),
    void (*request_get_kfs)(app_zone2_get_kfs_rel_t rel));

typedef struct {
    /** 有效 path 条数（桩号 1..12，不含 0）。R1 不下发 0 结尾时必写；写 0 表示沿用 path[] 遇 0 截断（兼容） */
    uint8_t path_n;
    /** 有效 kfs 条数。不下发 0 结尾时必写；写 0 表示沿用 kfs[] 遇 0 截断 */
    uint8_t kfs_n;
    uint8_t path[APP_ZONE2_MAX_PATH];
    uint8_t kfs[APP_ZONE2_MAX_KFS];
} app_zone2_mission_t;

void app_zone2_set_robot_tier(uint8_t tier012);

void app_zone2_mission_clear(void);
void app_zone2_mission_apply(const app_zone2_mission_t *m);

void app_zone2_poll(void);

uint8_t app_zone2_is_busy(void);
uint8_t app_zone2_is_done(void);

#endif /* APP_ZONE2_H */
