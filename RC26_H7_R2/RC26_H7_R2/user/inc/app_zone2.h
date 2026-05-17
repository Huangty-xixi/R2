/**
 * @file app_zone2.h
 * @brief 二区梅花桩流程：抬车上台面、走路径、夹隔壁格秘籍、换路径下一格时再对齐车头和高度。
 * @ref app_zone2_scheduling 见调度与状态机说明。
 *
 * path[] / kfs[]：R1 梅花桩号 1..12（红蓝同号同坐标，见 app_zone2.c s_user_pile_*）。不下发 0 结尾时填 path_n / kfs_n；为 0 时仍按数组遇 0 截断。
 */
#ifndef APP_ZONE2_H
#define APP_ZONE2_H

#include <stdint.h>

#include "app_init.h"

#include "odom_nav_goto.h"

/**
 * @anchor app_zone2_scheduling
 * @brief 二区调度流程（系统层 + 机内状态机，与 app_zone2.c 一致）
 *
 * @par 系统层：调用关系与周期
 * - 初始化：上电后由 App_Init() 调用 app_zone2_init_hooks()，注册导航 set/poll、上桩/下桩、
 *   上/下桩流程忙查询、车头对场地方向、取 KFS、取 KFS 流程忙查询等回调（见 app_init.c）。
 * - 任务装载：上层在拿到 R1 下发的 path[]/kfs[] 后须调用 app_zone2_mission_apply()，内部置
 *   s_has_mission 并进入机内首状态；未 apply 则 app_zone2_poll() 首行即 return，流程不推进。
 * - 周期推进：Motion_Task 在「control_mode=全自动」且 **app_flow_mode==app_flow_zone2** 时，
 *   每周期调用 app_zone2_poll() 推进状态机（与 Process_Flow 同思路，非 CH6 单步阻塞）；
 *   CH6 高档仅用于进入二区模式；急停/遥控仍会 mission_clear。
 * - Can_Task 调用 manual_chassis_function；其中与 odom_nav_goto_run 一并每周期 AppYawHeadingCtrl_Run()，供二区/上坡/一区航向 PD。
 *
 * @par 发令门控（实现于 app_zone2.c：app_zone2_motion_gate_ok）
 * 仅当「当前为全自动档」，且（**app_flow_zone2**；或 **app_flow_none** 且 **flow_mode==flow_none**）时，
 * 才允许向钩子下发 mount/dismount/摆头/get_kfs 等；其它 app_flow_zone1/zone3 占位阶段会关二区钩子。
 *
 * @par 机内主状态机（app_zone2_mission_apply 之后）
 * - Z2_IDLE / Z2_DONE：无任务或整局结束；app_zone2_is_done() 在 s_has_mission 且 Z2_DONE 时为真。
 * - apply 分支：
 *   - 若 path[0] 上仍有待取秘籍（存在 kfs[j]==path[0] 且未完成）→ 先走一区台面序：
 *     Z2_ZONE1_KFS_TURN（SKIP 摆头节拍）→ Z2_ZONE1_KFS_RUN → 回到 TURN 直至 path[0] 上秘籍取完，
 *     再进入 Z2_ENTER_UP。
 *   - 否则 → Z2_ENTER_UP（可先 request_mount 对齐 path[0] 层档）→ Z2_ENTER_NAV（下发桩心）→
 *     Z2_ENTER_WAIT_NAV（nav_poll 至到点）→ Z2_KFS_TURN。
 * - 梅林上循环：
 *   - Z2_KFS_TURN：对当前 path[path_idx] 与邻格待取秘籍桩计算 field_dir，发 request_face_field_dir，等 face_yaw_is_busy==0（航向 PD 在 manual_chassis_function）后 → Z2_KFS_RUN。
 *   - Z2_KFS_RUN：发 request_get_kfs(rel)，rel 由两桩顶高度档推算；完成后置位 kfs 掩码 → 回 Z2_KFS_TURN。
 *   - 若当前桩无未完邻格秘籍：path_idx++；若 path 走完且末桩为 200mm 的 10/12/6 → Z2_LAST_DOWN_TURN
 *    （10/12 朝场后，6 桩红区朝场左、蓝区朝场右）→ Z2_LAST_DOWN_DISMOUNT 一次下地面 → Z2_DONE；
 *     其它末桩 → 直接 Z2_DONE。
 * - Z2_PATH_NEXT_PILE（换 path 桩）：
 *   先摆头（邻格走向或 BACK，若层高需下桩则取反朝向），再按 user_pile_tier_delta 逐档
 *   request_mount / request_dismount；摆头完成且层档与目标桩一致后 → Z2_ENTER_NAV 去下一桩中心。
 *
 * @par 钩子与数据流摘要
 * - nav_set_target + nav_poll：梅林桩心坐标；poll 返回 ODOM_NAV_GOTO_ERR_OK_ARRIVED 表示到点。
 * - request_mount_pile / request_dismount_pile：层高 ±1 档（与 Process 上/下台阶语义对接）。
 * - mount_pile_is_busy / dismount_pile_is_busy：与上/下桩 request 配套的流程忙查询；在等待完成阶段若未注册则视为一直忙（防一拍误过），须与 request 成对注册。
 * - request_face_field_dir：车头对场地前/后/左/右或 SKIP（RunFieldDir 只设目标场向，不写电机）。
 * - face_yaw_is_busy：摆头后仅查询忙闲；周期 PD 在 chassis.c manual_chassis_function 内 AppYawHeadingCtrl_Run。
 * - request_get_kfs(rel)：邻格取秘籍；rel 为 APP_ZONE2_GET_KFS_HIGH_TO_LOW / LOW_TO_HIGH。
 * - get_kfs_is_busy：与 request_get_kfs 配套的流程忙查询；在等待完成阶段若未注册则视为一直忙（防一拍误过），须与 request 成对注册。
 * - app_zone2_set_robot_tier：可选，由上层在已知初始层高时同步 s_robot_tier；未调时主要由
 *   上/下桩完成节拍在机内维护 tier。
 */

/**
 * 蓝区 nav_set_pile_center_m：odom 目标 x = MIRROR_X_M - map_x（红蓝半场 x 对调，桩表不重复）。
 * 摆头/邻格 dx,dy 用统一 map 坐标，不镜像。半幅 6000mm 时为 6.0f。
 */
#ifndef APP_ZONE2_MIRROR_X_M
#define APP_ZONE2_MIRROR_X_M (6.0f)
#endif

/** APP_ZONE2_DBG_FAKE_MISSION、APP_ZONE2_RED_SIDE 默认值见 app_init.h */

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
    void (*request_get_kfs)(app_zone2_get_kfs_rel_t rel),
    uint8_t (*face_yaw_is_busy)(void),
    uint8_t (*mount_pile_is_busy)(void),
    uint8_t (*dismount_pile_is_busy)(void),
    uint8_t (*get_kfs_is_busy)(void));

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
/** 装载任务并启动机内状态机；须先于二区轮询调用，见 @ref app_zone2_scheduling */
void app_zone2_mission_apply(const app_zone2_mission_t *m);

#if APP_ZONE2_DBG_FAKE_MISSION
/** 仅调试：写入 app_init.h 中 APP_ZONE2_DBG_FAKE_* 假 path/kfs，不 apply */
void app_zone2_debug_fake_mission_get(app_zone2_mission_t *m);
#endif

/** 二区状态机周期推进，见 @ref app_zone2_scheduling */
void app_zone2_poll(void);

uint8_t app_zone2_is_busy(void);
uint8_t app_zone2_is_done(void);

/**
 * Keil Watch
 * - turn_busy：1=转向进行中
 * - turn_dir：与最近一次 request_face_field_dir 相同（3左 4右）
 * - robot_tier：当前车层档 0/1/2（200/400/600mm 档）
 * - tier_cha：换桩时目标桩层档减 robot_tier；>0 上桩 <0 下桩 0 不升降
 */
typedef struct {
    uint8_t turn_busy;
    uint8_t turn_dir;
    uint8_t robot_tier;
    int8_t tier_cha;
} app_zone2_debug_t;

extern volatile app_zone2_debug_t g_app_zone2_debug;

#endif /* APP_ZONE2_H */
