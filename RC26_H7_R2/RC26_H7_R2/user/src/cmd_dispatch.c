/**
 * @file cmd_dispatch.c
 * @brief PC上位机指令分派器实现
 *
 * 收到PC串口控制指令后，写入对应电机/执行器结构体。
 * 不走 control_mode/remote_mode 体系，直接操作底层。
 */

#include "cmd_dispatch.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "Process_Flow.h"
#include "app_zone1.h"
#include "app_zone2.h"
#include "app_zone3.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"

/* ---- 全局仲裁上下文 ---- */
volatile pc_arbiter_ctx_t g_pc_arbiter = {0};

/* ---- 外部声明 ---- */
extern void Mecanum_Inverse(float vx, float vy, float vw, float *rpm);

/* ---- KFS 全局变量（定义在 kfs.c）---- */
extern Three_kfs_position three_kfs_position;
extern Kfs_spin_position  kfs_spin_position;
extern Main_lift_position main_lift_position;
extern Flexible_Mode      flexible_mode;
extern volatile Kfs_Below_Cmd kfs_below_position;
extern volatile Kfs_Above_Cmd kfs_above_position;

/* ---- 武器函数（定义在 weapon.c）---- */
extern void servo_use(void);
extern void clamp_use(void);
extern void sucker1_use(void);
extern void sucker2_use(void);
extern void sucker3_use(void);
extern void sucker4_use(void);

/* ================================================================
   cmd_dispatch — PC命令分派入口（从upper_pc_protocol.c接收循环调用）
   ================================================================ */
void cmd_dispatch(uint8_t cmd, const uint8_t *payload)
{
    switch (cmd) {

    /* ---- 0x30: 底盘速度指令 Vx,Vy,Vw (3个float, 12字节) ---- */
    case PC_CMD_CHASSIS_SPEED: {
        float vx = *(const float *)(payload + 0);
        float vy = *(const float *)(payload + 4);
        float vw = *(const float *)(payload + 8);

        Chassis.param.Vx_in = vx;
        Chassis.param.Vy_in = vy;
        Chassis.param.Vw_in = vw;

        g_pc_arbiter.pc_cmd_pending = 1U;
        break;
    }

    /* ---- 0x31: 急停 ---- */
    case PC_CMD_CHASSIS_STOP:
        Chassis.param.Vx_in = 0.0f;
        Chassis.param.Vy_in = 0.0f;
        Chassis.param.Vw_in = 0.0f;
        chassis_motor1.rpm_target = 0.0f;
        chassis_motor2.rpm_target = 0.0f;
        chassis_motor3.rpm_target = 0.0f;
        chassis_motor4.rpm_target = 0.0f;
        guide_motor1.rpm_target = 0.0f;
        guide_motor2.rpm_target = 0.0f;
        kfs_above.rpm_target = 0.0f;
        kfs_below.rpm_target = 0.0f;
        R2_lift_motor_left.rpm_target  = 0.0f;
        R2_lift_motor_right.rpm_target = 0.0f;
        g_pc_arbiter.pc_cmd_pending = 1U;
        break;

    /* ---- 0x32: KFS动作 0=停, 1=取, 2=放 ---- */
    case PC_CMD_KFS_ACTION: {
        uint8_t action = payload[0];
        if (action == 1U) {
            kfs_above.rpm_target =  50.0f;   /* 上滚轮正转 */
            kfs_below.rpm_target = -50.0f;   /* 下滚轮反转 */
        } else if (action == 2U) {
            kfs_above.rpm_target = -50.0f;   /* 反向 */
            kfs_below.rpm_target =  50.0f;
        } else {
            kfs_above.rpm_target = 0.0f;
            kfs_below.rpm_target = 0.0f;
        }
        g_pc_arbiter.pc_cmd_pending = 1U;
        break;
    }

    /* ---- 0x33: 抬升速度 1个float ---- */
    case PC_CMD_LIFT_SPEED: {
        float speed = *(const float *)payload;
        R2_lift_motor_left.rpm_target  = speed;
        R2_lift_motor_right.rpm_target = speed;
        g_pc_arbiter.pc_cmd_pending = 1U;
        break;
    }

    /* ---- 0x34: 流程触发 1=取KFS 2=放KFS 3=上台阶 4=下台阶 5=上R1 ---- */
    case PC_CMD_FLOW_ACTION: {
        uint8_t flow_id = payload[0];
        Process_Flow_ResetAll();

        if (flow_id == 1U) {
            Process_GetKFS(NULL);
        } else if (flow_id == 2U) {
            Process_PutKFS(NULL);
        } else if (flow_id == 3U) {
            Process_UpStairs(NULL);
        } else if (flow_id == 4U) {
            Process_DownStairs(NULL);
        } else if (flow_id == 5U) {
            Process_UpR1(NULL);
        }

        g_pc_arbiter.pc_cmd_pending = 1U;
        g_pc_arbiter.pc_lock_type = PC_LOCK_ACTION;
        g_pc_arbiter.action_start_tick = HAL_GetTick();
        break;
    }

    /* ---- 0x35: zone启动 1=一区 2=二区 3=三区 ---- */
    case PC_CMD_ZONE_START: {
        uint8_t zone_id = payload[0];
        if (zone_id == 1U) {
            AppZone1_Start();
        } else if (zone_id == 2U) {
            /* 二区需要R1使命，这里只做标记触发 */
            app_zone2_mission_apply();
        } else if (zone_id == 3U) {
            AppZone3_Start();
        }
        g_pc_arbiter.pc_cmd_pending = 1U;
        g_pc_arbiter.pc_lock_type = PC_LOCK_ACTION;
        g_pc_arbiter.action_start_tick = HAL_GetTick();
        break;
    }

    /* ---- 0x36: 武器 toggle [设备号：1=吸盘1 2=吸盘2 3=吸盘3 4=吸盘4 6=夹爪 7=舵机] ---- */
    case PC_CMD_WEAPON: {
        uint8_t dev = payload[0];
        switch (dev) {
        case 1U: sucker1_use(); break;
        case 2U: sucker2_use(); break;
        case 3U: sucker3_use(); break;
        case 4U: sucker4_use(); break;
        case 6U: clamp_use();   break;
        case 7U: servo_use();   break;
        default: break;
        }
        g_pc_arbiter.pc_cmd_pending = 1U;
        break;
    }

    /* ---- 0x37~0x3b: KFS档位 +/- [方向：0=减 1=加] ---- */
    case 0x37U: /* three_kfs 旋转 */
    case 0x38U: /* kfs_spin 前臂 */
    case 0x39U: /* main_lift 主轴 */
    case 0x3AU: /* 伸缩位置 */
    case 0x3BU: /* 伸缩模式 */
    case 0x3CU: /* 伸缩上下切换 */
    case 0x3DU:
    case 0x3EU:
    case 0x3FU: {
        uint8_t dir = payload[0];  /* 0=减 1=加 */
        uint8_t dev = cmd - 0x36U;

        switch (dev) {
        case 1U: /* three_kfs */
            if (dir == 0U) {
                three_kfs_position = (Three_kfs_position)(((int)three_kfs_position + 3) % 4);
            } else {
                three_kfs_position = (Three_kfs_position)(((int)three_kfs_position + 1) % 4);
            }
            break;
        case 2U: /* kfs_spin */
            if (dir == 0U) {
                kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position + 3) % 4);
            } else {
                kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position + 1) % 4);
            }
            break;
        case 3U: /* main_lift */
            if (dir == 0U) {
                main_lift_position = (Main_lift_position)(((int)main_lift_position + 4) % 5);
            } else {
                main_lift_position = (Main_lift_position)(((int)main_lift_position + 1) % 5);
            }
            break;
        case 4U: /* 伸缩位置 */
            if (dir == 0U) {
                kfs_below_position = (Kfs_Below_Cmd)((int)kfs_below_position > 0 ? (int)kfs_below_position - 1 : 0);
            } else {
                kfs_below_position = (Kfs_Below_Cmd)((int)kfs_below_position < 3 ? (int)kfs_below_position + 1 : 3);
            }
            break;
        case 5U: /* flex_mode */
            flexible_mode = (Flexible_Mode)(((int)flexible_mode + 1) % 4);
            break;
        default: break;
        }
        g_pc_arbiter.pc_cmd_pending = 1U;
        break;
    }

    default:
        break;
    }
}

/* ================================================================
   pc_arbiter_tick — 每周期仲裁（在 Can_Task 开头调用一次）
   ================================================================ */
void pc_arbiter_tick(void)
{
    uint8_t  remote_ch8_high = (RCctrl.CH8 > 1500U);    /* 遥控器在"遥控模式"档位 */
    uint8_t  remote_moving   = 0U;                       /* 遥控器摇杆是否有动作 */
    uint8_t  remote_lost     = RCctrl.rc_lost;           /* SBUS丢链标志 */
    uint8_t  pc_moving       = g_pc_arbiter.pc_cmd_pending; /* PC是否有命令 */

    /* ---- 判断遥控器是否在动 ---- */
    if (!remote_lost && remote_ch8_high) {
        /* 摇杆偏离中位超过死区 -> 遥控器在动 */
        uint16_t ch1_d = (uint16_t)(RCctrl.CH1 > (992U + ARBITER_DEADBAND)
                                 || RCctrl.CH1 < (992U - ARBITER_DEADBAND));
        uint16_t ch2_d = (uint16_t)(RCctrl.CH2 > (992U + ARBITER_DEADBAND)
                                 || RCctrl.CH2 < (992U - ARBITER_DEADBAND));
        uint16_t ch3_d = (uint16_t)(RCctrl.CH3 > (992U + ARBITER_DEADBAND)
                                 || RCctrl.CH3 < (992U - ARBITER_DEADBAND));
        uint16_t ch4_d = (uint16_t)(RCctrl.CH4 > (992U + ARBITER_DEADBAND)
                                 || RCctrl.CH4 < (992U - ARBITER_DEADBAND));
        uint16_t ch5_d = (uint16_t)(RCctrl.CH5 < 500U || RCctrl.CH5 > 1500U);
        uint16_t ch6_d = (uint16_t)(RCctrl.CH6 < 500U || RCctrl.CH6 > 1500U);
        uint16_t ch7_d = (uint16_t)(RCctrl.CH7 < 500U || RCctrl.CH7 > 1500U);

        if (ch1_d || ch2_d || ch3_d || ch4_d || ch5_d || ch6_d || ch7_d) {
            remote_moving = 1U;
        }
    }

    /* ---- 判断PC速度指令是否有有效值（非零速） ---- */
    float vx_abs = (Chassis.param.Vx_in > 0.0f) ? Chassis.param.Vx_in : -Chassis.param.Vx_in;
    float vy_abs = (Chassis.param.Vy_in > 0.0f) ? Chassis.param.Vy_in : -Chassis.param.Vy_in;
    float vw_abs = (Chassis.param.Vw_in > 0.0f) ? Chassis.param.Vw_in : -Chassis.param.Vw_in;

    if (g_pc_arbiter.pc_lock_type == PC_LOCK_VELOCITY
        && vx_abs < ARBITER_VEL_DEADBAND
        && vy_abs < ARBITER_VEL_DEADBAND
        && vw_abs < ARBITER_VEL_DEADBAND
        && !g_pc_arbiter.pc_cmd_pending) {
        /* PC在发零速，累加空闲计数 */
        g_pc_arbiter.idle_ticks++;
    } else if (g_pc_arbiter.pc_cmd_pending) {
        g_pc_arbiter.idle_ticks = 0U;
    }

    /* ---- 动作锁：检查流程是否完成 ---- */
    if (g_pc_arbiter.pc_lock_type == PC_LOCK_ACTION) {
        uint8_t flow_busy = (uint8_t)(Process_GetKFS_IsBusy()
                                   || Process_PutKFS_IsBusy()
                                   || Process_UpStairs_IsBusy()
                                   || Process_DownStairs_IsBusy()
                                   || Process_UpR1_IsBusy()
                                   || AppZone1_IsBusy()
                                   || AppZone2_IsBusy()
                                   || AppZone3_IsActive());
        if (!flow_busy) {
            /* 所有流程已完成 */
            g_pc_arbiter.pc_lock_type = PC_LOCK_NONE;
        }
        /* 保底超时 30s */
        if (HAL_GetTick() - g_pc_arbiter.action_start_tick > ARBITER_ACTION_TIMEOUT_MS) {
            g_pc_arbiter.pc_lock_type = PC_LOCK_NONE;
            Process_Flow_ResetAll();  /* 超时强制清理 */
        }
    }

    /* ---- 仲裁状态机 ---- */
    switch (g_pc_arbiter.lock) {

    case ARBITER_NONE:
        /* 无锁：谁先动锁谁 */
        if (remote_moving) {
            g_pc_arbiter.lock = ARBITER_REMOTE;
            g_pc_arbiter.idle_ticks = 0U;
        } else if (pc_moving) {
            g_pc_arbiter.lock = ARBITER_PC;
            g_pc_arbiter.idle_ticks = 0U;
            if (g_pc_arbiter.pc_lock_type == PC_LOCK_NONE) {
                g_pc_arbiter.pc_lock_type = PC_LOCK_VELOCITY;
            }
        }
        break;

    case ARBITER_REMOTE:
        /* 遥控器锁：检查遥控器是否空闲 */
        if (!remote_moving) {
            g_pc_arbiter.idle_ticks++;
        } else {
            g_pc_arbiter.idle_ticks = 0U;
        }
        if (g_pc_arbiter.idle_ticks >= ARBITER_IDLE_TIMEOUT_MS) {
            g_pc_arbiter.lock = ARBITER_NONE;
            g_pc_arbiter.idle_ticks = 0U;
        }
        break;

    case ARBITER_PC:
        /* PC锁：检查是否应该释放 */
        if (g_pc_arbiter.pc_lock_type == PC_LOCK_NONE
            && g_pc_arbiter.idle_ticks >= ARBITER_IDLE_TIMEOUT_MS) {
            g_pc_arbiter.lock = ARBITER_NONE;
            g_pc_arbiter.idle_ticks = 0U;
        }
        break;
    }

    /* ---- 同一周期两边都动：遥控器优先 ---- */
    if (remote_moving && pc_moving && g_pc_arbiter.lock == ARBITER_PC) {
        g_pc_arbiter.lock = ARBITER_REMOTE;
        g_pc_arbiter.idle_ticks = 0U;
    }

    /* ---- 清除周期标志 ---- */
    g_pc_arbiter.pc_cmd_pending = 0U;
}
