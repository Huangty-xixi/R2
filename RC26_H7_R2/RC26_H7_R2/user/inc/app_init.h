#ifndef APP_INIT_H
#define APP_INIT_H

/**
 * 应用层编译开关（默认值）。可在 Keil「C/C++」预处理器宏定义中用 -D宏名=值 覆盖。
 */

/** 二区半场：1=红区，0=蓝区。须与场地一致。末桩 6 下地：RIGHT(4)，yaw -90°（与 5→6 向右一致）。 */
#ifndef APP_ZONE2_RED_SIDE
#define APP_ZONE2_RED_SIDE 0
#endif

/** 置 1：无任务时 app_zone2_poll 自动装载内置假 path/kfs（仅调试用） */
#ifndef APP_ZONE2_DBG_FAKE_MISSION
#define APP_ZONE2_DBG_FAKE_MISSION 1
#endif

#if APP_ZONE2_DBG_FAKE_MISSION
/** 调试假任务 path/kfs：改此处即可，app_zone2_debug_fake_mission_get 与 poll 自动装载均读下列宏 */
#ifndef APP_ZONE2_DBG_FAKE_PATH_N
#define APP_ZONE2_DBG_FAKE_PATH_N 3U
#endif
#ifndef APP_ZONE2_DBG_FAKE_KFS_N
#define APP_ZONE2_DBG_FAKE_KFS_N 0U
#endif
#ifndef APP_ZONE2_DBG_FAKE_PATH_LIST
#define APP_ZONE2_DBG_FAKE_PATH_LIST 2U, 5U, 6U
#endif
#ifndef APP_ZONE2_DBG_FAKE_KFS_LIST
#define APP_ZONE2_DBG_FAKE_KFS_LIST 0U
#endif
#endif /* APP_ZONE2_DBG_FAKE_MISSION */

/** 二区每个主状态开始前等待毫秒数；0=关闭（调试可设 3000） */
#ifndef APP_ZONE2_STEP_PRE_DELAY_MS
#define APP_ZONE2_STEP_PRE_DELAY_MS 250U
#endif

/** 置 1：里程计到点导航附加观测与调试钩子 */
#ifndef ODOM_NAV_GOTO_WATCH_DEBUG
#define ODOM_NAV_GOTO_WATCH_DEBUG 1
#endif

/** 遥控链路丢失保护：1=开启，0=关闭 */
#ifndef REMOTE_LOST_PROTECT_ENABLE
#define REMOTE_LOST_PROTECT_ENABLE 1
#endif

/** 遥控链路看门狗自测：1=开启，0=关闭 */
#ifndef REMOTE_LINK_TEST_ENABLE
#define REMOTE_LINK_TEST_ENABLE 0
#endif

/** 电机过温保护单元测试（Motor_OverTemp_SimpleTest）：1=编译进测试分支 */
#ifndef MOTOR_OVERTEMP_TEST_ENABLE
#define MOTOR_OVERTEMP_TEST_ENABLE 0
#endif

/**
 * 下台阶方案：改这一个宏即可（Keil 可用 -DPROCESS_FLOW_DOWNSTAIRS_PLAN=2 覆盖）
 *   0 = PlanA  快抬升+后退→停→快降
 *   1 = PlanB  后退至测距突增/超时→抬升→再退→快降（默认）
 *   2 = PlanC  先前进→再后退→抬升→再退→快降
 */
#ifndef PROCESS_FLOW_DOWNSTAIRS_PLAN
#define PROCESS_FLOW_DOWNSTAIRS_PLAN 2
#endif

/**
 * 姿态角写入 g_sensor_task_data.imu 的来源：1=HI14 IMU 帧；0=上位机 ODOM 的 roll/pitch/yaw（雷达/融合），加计陀螺磁力仍来自 IMU。
 */
#ifndef RC_USE_IMU_ATTITUDE
#define RC_USE_IMU_ATTITUDE 0
#endif

void App_Init(void);

#endif /* APP_INIT_H */
