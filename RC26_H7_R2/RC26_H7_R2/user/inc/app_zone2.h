/**
 * @file app_zone2.h
 * @brief 二区梅花桩流程：抬车上台面、走路径、夹隔壁格秘籍、换路径下一格时再对齐车头和高度。
 * @ref app_zone2_scheduling 见调度与状态机说明。
 *
 * path[] / kfs[]：红区标号同总图 34980efe1d98381ef7d7d0f5281d48a5.png（3,2,1 / 6,5,4 / …）；蓝区 1..12 与 mf 逐行一致。不下发 0 结尾时填 path_n / kfs_n；为 0 时仍按数组遇 0 截断。
 */
#ifndef APP_ZONE2_H
#define APP_ZONE2_H

#include <stdint.h>

#include "odom_nav_goto.h"

/**
 * @anchor app_zone2_scheduling
 * @brief 二区调度流程（系统层 + 机内状态机，与 app_zone2.c 一致）
 *
 * @par 系统层：调用关系与周期
 * - 初始化：上电后由 AppHook_Init() 调用 app_zone2_init_hooks()，注册导航 set/poll、上桩/下桩、
 *   车头对场地方向、取 KFS 等回调（见 app_hook_init.c）。
 * - 任务装载：上层在拿到 R1 下发的 path[]/kfs[] 后须调用 app_zone2_mission_apply()，内部置
 *   s_has_mission 并进入机内首状态；未 apply 则 app_zone2_poll() 首行即 return，流程不推进。
 * - 周期推进：Motion_Task 在「control_mode=全自动」且「full_auto_mode=二区模式」且「CH6 最大」时
 *   每周期调用 app_zone2_poll()；CH6 最小会 app_zone2_mission_clear() 并退出二区模式。
 *   遥控/急停分支里同样会 mission_clear（与取秘籍/上坡等全自动互斥由遥控位定义）。
 * - Can_Task 中二区不在 full_auto_zone2_mode 分支内重复执行，与 Motion_Task 分工一致。
 *
 * @par 发令门控（实现于 app_zone2.c：app_zone2_motion_gate_ok）
 * 仅当「当前为全自动档」且 full_auto_mode 为「无独占流程」或「二区模式」时，才允许向钩子下发
 * mount/dismount/摆头/get_kfs 等，避免与其它全自动子流程抢发令。
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
 *   - Z2_KFS_TURN：对当前 path[path_idx] 与邻格待取秘籍桩计算 field_dir，发 request_face_field_dir，
 *     等「下游空闲」后 → Z2_KFS_RUN。
 *   - Z2_KFS_RUN：发 request_get_kfs(rel)，rel 由两桩顶高度档推算；完成后置位 kfs 掩码 → 回 Z2_KFS_TURN。
 *   - 若当前桩无未完邻格秘籍：path_idx++；若 path 走完 → Z2_DONE；否则 → Z2_PATH_NEXT_PILE。
 * - Z2_PATH_NEXT_PILE（换 path 桩）：
 *   先摆头（邻格走向或 BACK，若层高需下桩则取反朝向），再按 user_pile_tier_delta 逐档
 *   request_mount / request_dismount；摆头完成且层档与目标桩一致后 → Z2_ENTER_NAV 去下一桩中心。
 *
 * @par 钩子与数据流摘要
 * - nav_set_target + nav_poll：梅林桩心坐标；poll 返回 ODOM_NAV_GOTO_ERR_OK_ARRIVED 表示到点。
 * - request_mount_pile / request_dismount_pile：层高 ±1 档（与 Process 上/下台阶语义对接）。
 * - request_face_field_dir：车头对场地前/后/左/右或 SKIP（一区台面段）。
 * - request_get_kfs(rel)：邻格取秘籍；rel 为 APP_ZONE2_GET_KFS_HIGH_TO_LOW / LOW_TO_HIGH。
 * - app_zone2_set_robot_tier：可选，由上层在已知初始层高时同步 s_robot_tier；未调时主要由
 *   上/下桩完成节拍在机内维护 tier。
 */

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
/** 装载任务并启动机内状态机；须先于二区轮询调用，见 @ref app_zone2_scheduling */
void app_zone2_mission_apply(const app_zone2_mission_t *m);

/** 每周期推进状态机；由 Motion_Task 在二区模式下调用，见 @ref app_zone2_scheduling */
void app_zone2_poll(void);

uint8_t app_zone2_is_busy(void);
uint8_t app_zone2_is_done(void);

#endif /* APP_ZONE2_H */
