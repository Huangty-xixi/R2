#ifndef APP_INIT_H
#define APP_INIT_H

/**
 * 应用层编译开关（默认值）。可在 Keil「C/C++」预处理器宏定义中用 -D宏名=值 覆盖。
 */

/** 二区 / ODOM 半场：1=红方半区（与场地、里程计解包已对齐，作基准）；0=蓝方半区（镜像半场，若异常优先核对蓝侧分支） */
#ifndef APP_ZONE2_RED_SIDE
#define APP_ZONE2_RED_SIDE 0
#endif

/** 蓝区 ODOM：yaw = wrap(unpack + 偏置度)。与到点旋转分开标定 */
#ifndef RC_ODOM_BLUE_YAW_EXTRA_DEG
#define RC_ODOM_BLUE_YAW_EXTRA_DEG 0
#endif
/** 蓝区到点：1=世界→车体系 cos/sin 里对 yaw 取反（与 RC_ODOM_BLUE_YAW_EXTRA_DEG 独立，偏置试遍仍错再试） */
#ifndef ODOM_NAV_GOTO_BLUE_NEGATE_ROT_YAW
#define ODOM_NAV_GOTO_BLUE_NEGATE_ROT_YAW 0
#endif
/** 蓝区到点：1=下发底盘前 vy 再乘 -1 */
#ifndef ODOM_NAV_GOTO_BLUE_FLIP_CMD_VY
#define ODOM_NAV_GOTO_BLUE_FLIP_CMD_VY 0
#endif
/** 蓝区到点：1=下发底盘前 vw 再乘 -1 */
#ifndef ODOM_NAV_GOTO_BLUE_FLIP_CMD_VW
#define ODOM_NAV_GOTO_BLUE_FLIP_CMD_VW 0
#endif

/** 置 1：无任务时 app_zone2_poll 自动装载内置假 path/kfs（仅调试用） */
#ifndef APP_ZONE2_DBG_FAKE_MISSION
#define APP_ZONE2_DBG_FAKE_MISSION 1
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
 * 下台阶流程：1=原流程（抬升后退→停→快降）；0=PlanB（先 vy+10 3s → vy-10 3s → 抬升等 1.5s → vy-10 2s → 快降前等 1s → 再等 wait_fall_done_ms）
 * 可在 Keil 里 -DPROCESS_FLOW_DOWNSTAIRS_PLAN_A=0 切换。
 */
#ifndef PROCESS_FLOW_DOWNSTAIRS_PLAN_A
#define PROCESS_FLOW_DOWNSTAIRS_PLAN_A 0
#endif

void App_Init(void);

#endif /* APP_INIT_H */
