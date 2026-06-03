/**
 * @file app_zone2.h
 * @brief 二区梅花桩流程：抬车上台面、走路径、夹隔壁格秘籍、换路径下一格时再对齐车头和高度。
 * @ref app_zone2_scheduling 见调度与状态机说明。
 *
 * path[] / kfs[]：R1 梅花桩号 1..12；格心/桩高见 map.h MAP_*_PILE_* / MAP_ZONE2_PILE_HEIGHT_MM。不下发 0 结尾时填 path_n / kfs_n；为 0 时仍按数组遇 0 截断。
 */
#ifndef APP_ZONE2_H
#define APP_ZONE2_H

#include <stdint.h>

#include "app_init.h"

#include "odom_nav_goto.h"

/**
 * @anchor app_zone2_scheduling
 * @brief 二区流程（系统层 + 调度层 + 执行层，与 app_zone2.c 一致）
 *
 * @par 机内分层（app_zone2.c）
 * - **调度层** `z2_sched_*`：根据 path/kfs 与 s_major 决定下一步主状态（path_idx、
 *   换桩、末桩下地面、一区台面序等）；不直接写电机，只切换 Z2_* 并调用执行层。
 * - **执行层** `z2_exec_*`：在门控允许时直接调用 nav / 摆头 / 上桩 / 下桩 / 取秘籍，
 *   轮询 busy 直至 Z2_EXEC_DONE；含导航段、子步摆头/回桩心等。
 * - **步骤记录层** `z2_step_*`：记录当前固定流程动作（step_kind/from/to/kfs/cha），用于 Watch 对齐；不做 Mission 校验。
 * - **入口**：app_zone2_poll() → 任务/节拍前置 → z2_sched_poll() 按 s_major 分发调度函数。
 *
 * @par 系统层：调用关系与周期
 * - 执行层在 app_zone2.c 内直接调用 odom_nav_goto_*、Process_*、YawHeadingCtrl_*（无钩子注册）。
 * - 任务装载：上层在拿到 R1 下发的 path[]/kfs[] 后须调用 app_zone2_mission_apply()，内部置
 *   s_has_mission 并进入机内首状态；未 apply 则 app_zone2_poll() 首行即 return，流程不推进。
 * - 周期推进：Motion_Task 在「control_mode=全自动」且 **app_flow_mode==app_flow_zone2** 时，
 *   每周期调用 app_zone2_poll() 推进状态机（与 Process_Flow 同思路，非 CH6 单步阻塞）；
 *   CH6 高档仅用于进入二区模式；急停/遥控仍会 mission_clear。
 * - Can_Task 调用 manual_chassis_function；其中与 odom_nav_goto_run 一并每周期 YawHeadingCtrl_Run()，供二区/上坡/一区航向 PD。
 *
 * @par 发令门控（实现于 app_zone2.c：z2_exec_motion_gate_ok）
 * 仅当「当前为全自动档」，且（**app_flow_zone2**；或 **app_flow_none** 且 **flow_mode==flow_none**）时，
 * 才允许下发 mount/dismount/摆头/get_kfs 等；其它 app_flow_zone1/zone3 占位阶段会关二区发令。
 *
 * @par 机内主状态机（app_zone2_mission_apply 之后）
 * - Z2_IDLE / Z2_DONE：无任务或整局结束；app_zone2_is_done() 在 s_has_mission 且 Z2_DONE 时为真。
 * - apply 后首段 Z2_ENTRY_NAV → Z2_ENTRY_WAIT_NAV：导航至 APP_ZONE2_ENTRY_NAV_* 并 main_lift→p3。
 * - 到点后 **二区预备流程** Z2_GROUND_PREP：桩1/3 地面取 KFS 最高档 p4，桩2 低取高 p3
 *   （优先级 2>1>3；桩2+桩3 时预备只取2，上桩后再取3）；预备不含上桩。
 * - 预备结束 begin_main_flow：Z2_MAIN_PREP_NAV → 桩2预备位 (3.0,2.6) → Z2_ENTER_UP 上桩
 *   →（若桩3延后）Z2_DEFERRED_KFS_GET → Z2_ENTER_NAV（path[0] **格心** map.c）→ Z2_KFS_TURN …
 * - 梅林上循环：
 *   - Z2_KFS_RUN：摆头结束后再 Process_GetKFS；至 chassis_forward 结束提前回 TURN；尾段不占 Vy，可与回中导航并行。
 *   - Z2_KFS_TURN：摆头(Vx)+回中(Vy/Vw)；再取下一件须上一件 GetKFS 全流程结束。
@ *   - kfs_done_mask 在整段 GetKFS 结束后由 tail_service 置位。
 *   - 若当前桩无未完邻格秘籍：path_idx++；若 path 走完且末桩为 200mm 的 10/12/6 → Z2_LAST_DOWN_TURN
 *    （10/12 朝场后，6 桩红区朝场左、蓝区朝场右）→ Z2_LAST_DOWN_DISMOUNT 一次下地面 → Z2_LAST_UPSLOPE
 *    （先到 PROCESS_UPSLOPE_P1_*，车头 FRONT，上坡）→ Z2_LAST_EXIT_NAV/WAIT_NAV → Z2_DONE；
 *     其它末桩 → 直接 Z2_DONE。
 * - Z2_PATH_NEXT_PILE（换 path 桩）：
 *   下发摆头后即可回当前桩（from）桩心（VX 摆头与 VY/VW 导航分轴并行）；若 cha!=0，则等摆头完成后再按 cha 上/下桩，层档对齐后 Z2_ENTER_NAV 去下一桩（to）桩心。
 * - Z2_LAST_DOWN_TURN：摆头 → 回末桩桩心 → 一次下地面 → 上坡 → 终点导航。
 *
 * @par 执行层与数据流摘要
 * - 每段导航：nav_leg_start → poll 至 ARRIVED 或 TIMEOUT 后进入下一步（二者等价）；ODOM/BAD_CONFIG 记入 nav_fail_rc 并结束整局；
 *   poll 使用 odom_nav_goto_peek_last_run_result（见 odom_nav_goto_service_tick）。
 * - Process_UpStairs / Process_DownStairs：层高 ±1 档；忙查询 Process_*_IsBusy。
 * - YawHeadingCtrl_RunFieldDir / IsBusy：车头对场地四向或 SKIP；周期 PD 在 manual_chassis_function 内 Run。
 * - Process_GetKFS(rel) / Process_GetKFS_IsBusy：主循环高/低取；地面预备桩2 低取高 p3，
 *   桩1/桩3 为 GROUND_HIGHEST（主轴 p4）。
 * - app_zone2_set_robot_tier：可选，由上层在已知初始层高时同步 s_robot_tier；未调时主要由
 *   上/下桩完成节拍在机内维护 tier。
 */

/** APP_ZONE2_DBG_FAKE_MISSION、APP_ZONE2_RED_SIDE 默认值见 app_init.h */

#define APP_ZONE2_MAX_PATH 16U
#define APP_ZONE2_MAX_KFS  12U

/** 与 @ref odom_nav_goto_err_t 一致；nav_poll 为 peek 上一拍底盘 run 的码 */
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
 * 邻格取秘籍：站立桩与邻格秘籍桩 tier 关系，或地面预备专用档位（传给 Process_GetKFS）。
 * 梅林主循环仅用 HIGH_TO_LOW / LOW_TO_HIGH；地面预备桩1/3 用 GROUND_HIGHEST，桩2 用 LOW_TO_HIGH。
 */
typedef enum {
    APP_ZONE2_GET_KFS_HIGH_TO_LOW = 0, /* 高桩取低：站立桩顶高于邻格秘籍桩，主轴 p0 */
    APP_ZONE2_GET_KFS_LOW_TO_HIGH,     /* 低桩取高，主轴 p3（含地面预备桩2） */
    APP_ZONE2_GET_KFS_GROUND_HIGHEST,  /* 地面预备桩1/桩3：最高档，主轴 p4 */
} app_zone2_get_kfs_rel_t;

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
 * Keil Watch（仅观测，不参与控制）
 * - poll_major：z2_sched_poll 当前主状态，数值同 z2_major_t（0 IDLE … 13 LAST_DOWN_DISMOUNT）
 * - nav_poll_rc：最近一次 odom_nav_goto_peek_last_run_result() 返回值，同 odom_nav_goto_err_t；
 *   本周期未调用 nav_poll 时为 APP_ZONE2_DEBUG_NAV_POLL_RC_NONE
 * - step_kind/step_seq：当前脚本动作步骤与切换序号；只用于观测和调度对齐，不做 Mission 校验。
 */
#define APP_ZONE2_DEBUG_NAV_POLL_RC_NONE 0xFFFFFFFFu

#define APP_ZONE2_DEBUG_STEP_NONE             0U//无动作
#define APP_ZONE2_DEBUG_STEP_ENTRY_NAV        14U//入口预定位导航
#define APP_ZONE2_DEBUG_STEP_GROUND_PREP_NAV   1U  /* 二区预备：导航至桩 1/2/3 预备位 */
#define APP_ZONE2_DEBUG_STEP_GROUND_PREP_GET   2U  /* 二区预备：地面 GetKFS */
#define APP_ZONE2_DEBUG_STEP_ENTER_MOUNT      3U//进入上桩
#define APP_ZONE2_DEBUG_STEP_NAV_TO_PILE      4U//进入导航
#define APP_ZONE2_DEBUG_STEP_FACE_KFS         5U//取件转弯
#define APP_ZONE2_DEBUG_STEP_GET_KFS          6U//取件运行
#define APP_ZONE2_DEBUG_STEP_FACE_NEXT        7U//换路径下一格时再对齐车头和高度
#define APP_ZONE2_DEBUG_STEP_RECENTER         8U//回桩心
#define APP_ZONE2_DEBUG_STEP_STAIR            9U//上桩      
#define APP_ZONE2_DEBUG_STEP_LAST_FACE        10U//末桩转弯
#define APP_ZONE2_DEBUG_STEP_LAST_RECENTER    11U//末桩回中
#define APP_ZONE2_DEBUG_STEP_GROUND_DISMOUNT  12U//下地面   
#define APP_ZONE2_DEBUG_STEP_DONE             13U//完成
#define APP_ZONE2_DEBUG_STEP_UPSLOPE          15U//upslope
#define APP_ZONE2_DEBUG_STEP_EXIT_NAV         16U//exit nav

typedef struct {
    volatile uint32_t poll_major;//主状态机

    volatile uint32_t nav_poll_rc;//最近一次 odom_nav_goto_peek_last_run_result() 返回值，同 odom_nav_goto_err_t；
    volatile uint32_t nav_fail_rc;//最近一次导航段失败码；NONE 表示无失败
    volatile uint32_t nav_session;//本段 session；peek 须一致，等同单独调试一段 nav
    volatile uint32_t nav_armed;//1=本段导航已 arm，未 ARRIVED/失败前不下一段
    volatile uint32_t nav_leg_running;//1=本段导航仍在进行；0=本段结束（ARRIVED 或 TIMEOUT，调度进下一步）；ODOM/BAD_CONFIG 仍结束整局
    volatile float nav_target_x_m;//目标 x 坐标
    volatile float nav_target_y_m;//目标 y 坐标
    volatile float nav_dist_m;//目标距离
    volatile float nav_vy_cmd;//目标 y 速度
    volatile float nav_vw_cmd;//目标 w 速度

    volatile uint32_t odom_session;//odom 当前 session

    volatile uint32_t step_kind;//当前脚本动作步骤，见 APP_ZONE2_DEBUG_STEP_*
    volatile uint32_t step_seq;//脚本动作步骤切换序号
    volatile uint32_t step_from_pile;//动作来源桩号
    volatile uint32_t step_to_pile;//动作目标桩号
    volatile uint32_t step_kfs_pile;//当前秘籍所在桩号
    volatile uint32_t step_kfs_idx;//当前秘籍索引
    volatile int32_t step_tier_delta;//目标层高差：正上桩，负下桩，0 不换层
    volatile uint32_t step_face_dir;//当前摆头方向，数值同 app_zone2_field_dir_t

    volatile uint32_t override_axis_mask;//底盘覆盖掩码
    volatile uint32_t override_priority;//底盘覆盖最高优先级
    volatile uint32_t override_priority_vx;//底盘 VX 轴优先级
    volatile uint32_t override_priority_vy;//底盘 VY 轴优先级
    volatile uint32_t override_priority_vw;//底盘 VW 轴优先级
    volatile float override_vx;//底盘覆盖速度
    volatile float override_vy;//底盘覆盖速度
    volatile float override_vw;//底盘覆盖速度

    volatile uint32_t process_busy_mask;//进程忙掩码
} app_zone2_debug_t;

extern volatile app_zone2_debug_t g_app_zone2_debug;

#endif /* APP_ZONE2_H */
