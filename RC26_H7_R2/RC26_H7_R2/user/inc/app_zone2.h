/**
 * @file app_zone2.h
 * @brief 二区梅花桩流程：抬车上台面、走路径、夹隔壁格秘籍、换路径下一格时再对齐车头和高度。
 *
 * path[] / kfs[] 使用示意图桩号（顶行 3、2、1 见 app_zone2.c 的 user_pile_to_mf）。
 */
#ifndef APP_ZONE2_H
#define APP_ZONE2_H

#include <stdint.h>

/**
 * 红/蓝半场抽签：当前二区逻辑与坐标按哪一侧写死。
 * 1 = 红区半幅（默认）；0 = 蓝区半幅——app_zone2.c 里会对梅林目标点 x 做镜像（见 APP_ZONE2_MIRROR_X_M）。
 * 可在工程选项里 -DAPP_ZONE2_RED_SIDE=0，或改此处默认值。
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

typedef enum {
    APP_ZONE2_NAV_MOVING = 0,
    APP_ZONE2_NAV_ARRIVED,
} app_zone2_nav_poll_result_t;

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

typedef struct {
    void (*nav_set_target)(float x_m, float y_m);
    app_zone2_nav_poll_result_t (*nav_poll)(void);

    /** 上桩（原「上台阶」，对齐下一档层高） */
    void (*request_mount_pile)(void *user);
    /** 下桩（原「下台阶」） */
    void (*request_dismount_pile)(void *user);

    /** 转到相对场地的某一固定朝向（由上层把四向映射到 IMU yaw / 半自动转向） */
    void (*request_face_field_dir)(app_zone2_field_dir_t dir);

    void (*request_get_kfs)(uint8_t station_user_pile, uint8_t neighbor_user_pile, uint8_t kfs_list_index,
                            void *user);

    void *user;
} app_zone2_hooks_t;

typedef struct {
    uint8_t path[APP_ZONE2_MAX_PATH];
    uint8_t kfs[APP_ZONE2_MAX_KFS];
} app_zone2_mission_t;

void app_zone2_set_hooks(const app_zone2_hooks_t *hooks);
void app_zone2_set_robot_tier(uint8_t tier012);

void app_zone2_mission_clear(void);
void app_zone2_mission_apply(const app_zone2_mission_t *m);

void app_zone2_poll(void);

uint8_t app_zone2_is_busy(void);
uint8_t app_zone2_is_done(void);

#endif /* APP_ZONE2_H */
