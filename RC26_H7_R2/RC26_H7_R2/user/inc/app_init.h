#ifndef APP_INIT_H
#define APP_INIT_H

/**
 * 应用层编译开关（默认值）。可在 Keil「C/C++」预处理器宏定义中用 -D宏名=值 覆盖。
 *
 * 分区说明：按功能模块 + 主要消费源文件划分；部分宏经 app_zone2.h / odom_nav_goto.h /
 * remote_control.h / Sensor_Task.h 间接传递，见各宏注释。
 */

/* ==========================================================================
 * 二区自动任务（app_zone2）
 * 主消费：user/src/app_zone2.c、user/inc/app_zone2.h
 * 关联：user/src/upper_pc_protocol.c（ODOM 坐标与半场一致）
 *       user/src/odom_nav_goto.c（半场相关导航偏置）
 *       user/src/map.c（经 map_zone2_pile_center_m 传 APP_ZONE2_RED_SIDE）
 *       user/src/Process_Flow.c、yaw_heading_ctrl.c（摆头/取 KFS，非本区宏）
 * ========================================================================== */

/** 半场：1=红区，0=蓝区。须与场地一致。二区末桩 6 下地：红 LEFT、蓝 RIGHT；一区阶段2/10 转 90°、阶段4/5 横移/重试随半场镜像。 */
#ifndef APP_ZONE2_RED_SIDE
#define APP_ZONE2_RED_SIDE 0
#endif

/** 置 1：无任务时 app_zone2_poll 自动装载内置假 path/kfs（仅调试用，正式赛务必置 0） */
#ifndef APP_ZONE2_DBG_FAKE_MISSION
#define APP_ZONE2_DBG_FAKE_MISSION 1
#endif

#if APP_ZONE2_DBG_FAKE_MISSION
/** 调试假任务 path/kfs：改此处即可；app_zone2_debug_fake_mission_get 与 poll 自动装载均读下列宏 */
#ifndef APP_ZONE2_DBG_FAKE_PATH_N
#define APP_ZONE2_DBG_FAKE_PATH_N 5U
#endif
#ifndef APP_ZONE2_DBG_FAKE_KFS_N
#define APP_ZONE2_DBG_FAKE_KFS_N 1U
#endif
#ifndef APP_ZONE2_DBG_FAKE_PATH_LIST
#define APP_ZONE2_DBG_FAKE_PATH_LIST 2U, 5U, 8U,7U, 10U
#endif
#ifndef APP_ZONE2_DBG_FAKE_KFS_LIST
#define APP_ZONE2_DBG_FAKE_KFS_LIST 1U
#endif
#endif /* APP_ZONE2_DBG_FAKE_MISSION */

/** 二区自动首段：上桩/取 path[0] 矿粉前，先导航到此点（米，与 odom 一致） */
#ifndef APP_ZONE2_ENTRY_NAV_X_M
#define APP_ZONE2_ENTRY_NAV_X_M 3.0f
#endif
#ifndef APP_ZONE2_ENTRY_NAV_Y_M 
#define APP_ZONE2_ENTRY_NAV_Y_M 2.6f
#endif

/** 二区预备流程：桩 1/2/3 地面取 KFS 预备位 y（米）；与上桩预备位 APP_ZONE2_ENTRY_NAV_* 一致 */
#ifndef APP_ZONE2_GROUND_PREP_Y_M
#define APP_ZONE2_GROUND_PREP_Y_M APP_ZONE2_ENTRY_NAV_Y_M
#endif
#ifndef APP_ZONE2_GROUND_PREP_PILE1_X_M
#define APP_ZONE2_GROUND_PREP_PILE1_X_M 1.8f
#endif
#ifndef APP_ZONE2_GROUND_PREP_PILE3_X_M
#define APP_ZONE2_GROUND_PREP_PILE3_X_M 4.2f
#endif

/* Zone2 tail: after leaving pile 6/10/12, Process_UpSlope first navigates here. */
#ifndef PROCESS_UPSLOPE_P1_X_M
#define PROCESS_UPSLOPE_P1_X_M 5.15f
#endif
#ifndef PROCESS_UPSLOPE_P1_Y_M
#define PROCESS_UPSLOPE_P1_Y_M 8.65f
#endif

/* Zone2 tail: after upslope completes, navigate here and finish zone2. */
#ifndef APP_ZONE2_EXIT_NAV_X_M
#define APP_ZONE2_EXIT_NAV_X_M 2.42f
#endif
#ifndef APP_ZONE2_EXIT_NAV_Y_M
#define APP_ZONE2_EXIT_NAV_Y_M 11.64f
#endif

/** 二区每个主状态开始前等待毫秒数；0=关闭（单步调试可设 3000） */
#ifndef APP_ZONE2_STEP_PRE_DELAY_MS
#define APP_ZONE2_STEP_PRE_DELAY_MS 0U
#endif

/* ==========================================================================
 * 里程计到点导航（odom_nav_goto）
 * 主消费：user/src/odom_nav_goto.c、user/inc/odom_nav_goto.h
 * 关联：user/src/chassis.c（半自动下调试 poll）、user/src/app_zone2.c（二区导航）
 * ========================================================================== */

/** 置 1：里程计到点导航附加观测与 Watch 调试（g_odom_nav_goto_dbg / odom_nav_goto_poll_debug） */
#ifndef ODOM_NAV_GOTO_WATCH_DEBUG
#define ODOM_NAV_GOTO_WATCH_DEBUG 1
#endif

/** 置 1：定点导航调试（g_nav_goto_dingdian_debug.a/b 方框路径，见 nav_goto_dingdian_debug.c） */
#ifndef ODOM_NAV_GOTO_DINGDIAN_DEBUG
#define ODOM_NAV_GOTO_DINGDIAN_DEBUG 1
#endif

/* ==========================================================================
 * 遥控链路（remote_control）
 * 主消费：user/src/remote_control.c、user/inc/remote_control.h
 * 关联：user/src/Can_Task.c（丢链保护分支）
 * ========================================================================== */

/** 遥控链路丢失保护：1=开启，0=关闭 */
#ifndef REMOTE_LOST_PROTECT_ENABLE
#define REMOTE_LOST_PROTECT_ENABLE 0
#endif

/** 遥控链路看门狗自测：1=开启，0=关闭（仅 remote_control.c 内自测逻辑） */
#ifndef REMOTE_LINK_TEST_ENABLE
#define REMOTE_LINK_TEST_ENABLE 0
#endif

/* ==========================================================================
 * 流程控制 — 下台阶（Process_Flow）
 * 主消费：user/src/Process_Flow.c、user/inc/Process_Flow.h
 * 关联：user/src/app_zone2.c（二区直接调用 Process_DownStairs 等）
 * ========================================================================== */

/**
 * 下台阶方案：改这一个宏即可（Keil 可用 -DPROCESS_FLOW_DOWNSTAIRS_PLAN=2 覆盖）
 *   0 = PlanA  快抬升+后退→停→快降
 *   1 = PlanB  PlanA俯仰+wait后倒车测距(突增或3s)→fall_fast
 *   2 = PlanC  先前进→再后退→抬升→再退→快降（默认）
 */
#ifndef PROCESS_FLOW_DOWNSTAIRS_PLAN
#define PROCESS_FLOW_DOWNSTAIRS_PLAN 1
#endif

/* ==========================================================================
 * 传感器任务（Sensor_Task）
 * 主消费：user/src/Sensor_Task.c、user/inc/Sensor_Task.h
 * ========================================================================== */

/**
 * 姿态角写入 g_sensor_task_data.imu 的来源：
 *   1 = HI14 IMU 帧（IMU_ParseFrameIfReady）
 *   0 = 上位机 ODOM 的 roll/pitch/yaw（雷达/融合），加计陀螺磁力仍来自 IMU
 */
#ifndef RC_USE_IMU_ATTITUDE
#define RC_USE_IMU_ATTITUDE 0
#endif

/* ==========================================================================
 * 电机（motor）
 * 主消费：user/src/motor.c（Motor_OverTemp_SimpleTest）
 * ========================================================================== */

/** 电机过温保护单元测试：1=编译进 Motor_OverTemp_SimpleTest 分支 */
#ifndef MOTOR_OVERTEMP_TEST_ENABLE
#define MOTOR_OVERTEMP_TEST_ENABLE 0
#endif

void App_Init(void);

#endif /* APP_INIT_H */
