#ifndef APP_INIT_H
#define APP_INIT_H

/**
 * 应用层参数与开关（默认值），可在 Keil C/C++ 预定义宏里用 -D宏=值 覆盖。
 *
 * 模块说明：按模块分块 + 标注受影响的源文件，部分宏经 app_zone2.h / odom_nav_goto.h /
 * remote_control.h / Sensor_Task.h 间接传递，详见注释。
 */

/* ==========================================================================
 * 区域二_自动控制（app_zone2）
 * 影响：user/src/app_zone2.c、user/inc/app_zone2.h
 * 传递：user/src/upper_pc_protocol.c（ODOM 坐标与进场一致）、
 *       user/src/odom_nav_goto.c（进场相关的偏移）、
 *       user/src/map.c（用 map_zone2_pile_center_m 和 APP_ZONE2_RED_SIDE）、
 *       user/src/Process_Flow.c、yaw_heading_ctrl.c（掉头/取 KFS，非标准坐标）
 * ========================================================================== */

/* ==========================================================================
 * 比赛模式选择（三选一，Keil -D 覆盖）
 *   APP_MATCH_SKILL_Z12  = 1 → 技能赛 1+2 区
 *   APP_MATCH_SKILL_Z3   = 1 → 技能赛 3 区
 *   APP_MATCH_ARENA      = 1 → 竞技赛
 * 默认全 0，暂不做功能调用，仅预置开关。
 * ========================================================================== */
#ifndef APP_MATCH_SKILL_Z12
#define APP_MATCH_SKILL_Z12 0
#endif
#ifndef APP_MATCH_SKILL_Z3
#define APP_MATCH_SKILL_Z3  0
#endif
#ifndef APP_MATCH_ARENA
#define APP_MATCH_ARENA     0
#endif

/** 红方/蓝方：1=红方，0=蓝方（进场方向一致。末桩 6 下地时，蓝 LEFT，红 RIGHT；桩 2/10 转 90°、桩 4/5 上/下坡以场前为基准。） */
#ifndef APP_ZONE2_RED_SIDE
#define APP_ZONE2_RED_SIDE 0
#endif

/** 置 1 调试时 app_zone2_poll 自动装载假 path/kfs（默认关，正式比赛置 0）。 */
#ifndef APP_ZONE2_DBG_FAKE_MISSION
#define APP_ZONE2_DBG_FAKE_MISSION 1U
#endif

#if APP_ZONE2_DBG_FAKE_MISSION
/** 假数据：改此处即可，app_zone2_debug_fake_mission_get 在 poll 自动装载并运行。 */
#ifndef APP_ZONE2_DBG_FAKE_PATH_N
#define APP_ZONE2_DBG_FAKE_PATH_N 5U
#endif
#ifndef APP_ZONE2_DBG_FAKE_KFS_N
#define APP_ZONE2_DBG_FAKE_KFS_N 3U
#endif
#ifndef APP_ZONE2_DBG_FAKE_PATH_LIST
#define APP_ZONE2_DBG_FAKE_PATH_LIST 2U,5U,8U,7U,10U
#endif
#ifndef APP_ZONE2_DBG_FAKE_KFS_LIST
#define APP_ZONE2_DBG_FAKE_KFS_LIST 2U,5U,8U
#endif
#endif /* APP_ZONE2_DBG_FAKE_MISSION */

/** 区域二入口导航：上桩/取 path[0] 前先到入口点（米），与 odom 一致。 */
#ifndef APP_ZONE2_ENTRY_NAV_X_M
#define APP_ZONE2_ENTRY_NAV_X_M 3.0f
#endif
#ifndef APP_ZONE2_ENTRY_NAV_Y_M 
#define APP_ZONE2_ENTRY_NAV_Y_M 2.65f
#endif

/** 地面预备流程：桩 1/2/3 地面取 KFS 预备位 y（米），值与桩2预备位 APP_ZONE2_ENTRY_NAV_* 一致 */
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

/** 每个步状态开始前等待毫秒数，0=关闭，调试可设 3000。 */
#ifndef APP_ZONE2_STEP_PRE_DELAY_MS
#define APP_ZONE2_STEP_PRE_DELAY_MS 0U
#endif

/* ==========================================================================
 * 里程计定点导航（odom_nav_goto）
 * 影响：user/src/odom_nav_goto.c、user/inc/odom_nav_goto.h
 * 传递：user/src/chassis.c（自动模式轮询）、user/src/app_zone2.c（区域二走场）
 * ========================================================================== */

/** 置 1，里程计定点导航增加观察用 Watch 调试：g_odom_nav_goto_dbg / odom_nav_goto_poll_debug。 */
#ifndef ODOM_NAV_GOTO_WATCH_DEBUG
#define ODOM_NAV_GOTO_WATCH_DEBUG 1
#endif

/** 置 1，定点导航调试：g_nav_goto_dingdian_debug.a/b 走方形路径（nav_goto_dingdian_debug.c）。 */
#ifndef ODOM_NAV_GOTO_DINGDIAN_DEBUG
#define ODOM_NAV_GOTO_DINGDIAN_DEBUG 1
#endif

/* ==========================================================================
 * 遥控器链路（remote_control）
 * 影响：user/src/remote_control.c、user/inc/remote_control.h
 * 传递：user/src/Can_Task.c（模式选择分支）
 * ========================================================================== */

/** 遥控器链路失控保护：1=启用，0=关闭 */
#ifndef REMOTE_LOST_PROTECT_ENABLE
#define REMOTE_LOST_PROTECT_ENABLE 0
#endif

/** 遥控器链路串口调试：1=启用，0=关闭（见 remote_control.c 调试逻辑）。 */
#ifndef REMOTE_LINK_TEST_ENABLE
#define REMOTE_LINK_TEST_ENABLE 0
#endif

/* ==========================================================================
 * 底盘控制 & 上下台阶（Process_Flow）
 * 影响：user/src/Process_Flow.c、user/inc/Process_Flow.h
 * 传递：user/src/app_zone2.c（会直接调用 Process_DownStairs 等）
 * ========================================================================== */

/**
 * 下台阶方案选择（用一个宏即可，Keil 加 -DPROCESS_FLOW_DOWNSTAIRS_PLAN=2 覆盖）：
 *   0 = PlanA  抬升+快退、停、速降
 *   1 = PlanB  PlanA快退+wait后倒车到底(突变+3s)，fall_fast
 *   2 = PlanC  车向前、快退、抬升+快退、速降（默认）
 */
#ifndef PROCESS_FLOW_DOWNSTAIRS_PLAN
#define PROCESS_FLOW_DOWNSTAIRS_PLAN 1
#endif

/* ==========================================================================
 * 传感器数据（Sensor_Task）
 * 影响：user/src/Sensor_Task.c、user/inc/Sensor_Task.h
 * ========================================================================== */

/**
 * 动态改写 g_sensor_task_data.imu 的数据源：
 *   1 = HI14 IMU 帧（IMU_ParseFrameIfReady）
 *   0 = 定位仪 ODOM 的 roll/pitch/yaw（雷达/融合），无独立 IMU
 */
#ifndef RC_USE_IMU_ATTITUDE
#define RC_USE_IMU_ATTITUDE 0
#endif

/* ==========================================================================
 * 电机（motor）
 * 影响：user/src/motor.c（Motor_OverTemp_SimpleTest）
 * ========================================================================== */

/** 电机过温保护单元测试：1=启用 Motor_OverTemp_SimpleTest 分支 */
#ifndef MOTOR_OVERTEMP_TEST_ENABLE
#define MOTOR_OVERTEMP_TEST_ENABLE 0
#endif

void App_Init(void);

#endif /* APP_INIT_H */