/**
 * @file cmd_dispatch.h
 * @brief PC上位机指令分派器——接收串口来的PC控制指令，写进对应电机/执行器结构体
 */

#ifndef CMD_DISPATCH_H
#define CMD_DISPATCH_H

#include <stdint.h>

/* ---- PC→STM32 命令码 ---- */
#define PC_CMD_CHASSIS_SPEED   0x30U   /* Vx,Vy,Vw 3个float */
#define PC_CMD_CHASSIS_STOP    0x31U   /* 急停 */
#define PC_CMD_KFS_ACTION      0x32U   /* 0=停 1=取 2=放 */
#define PC_CMD_LIFT_SPEED      0x33U   /* 升降速度 1个float */
#define PC_CMD_FLOW_ACTION     0x34U   /* 流程号 1=取KFS 2=放KFS 3=上台阶 4=下台阶 5=上R1 */
#define PC_CMD_ZONE_START      0x35U   /* zone号 1=一区 2=二区 3=三区 */
#define PC_CMD_WEAPON          0x36U   /* 武器 toggle [设备号] */
#define PC_CMD_KFS_POS         0x37U   /* KFS档位 [方向], 设备号=CMD-0x36 */

/* ---- 仲裁状态 ---- */
typedef enum {
    ARBITER_NONE = 0,       /* 无锁，谁先动归谁 */
    ARBITER_REMOTE,         /* 遥控器锁 */
    ARBITER_PC,             /* PC锁 */
} pc_arbiter_lock_t;

typedef enum {
    PC_LOCK_NONE = 0,       /* PC未在控制 */
    PC_LOCK_VELOCITY,       /* PC在发速度指令 */
    PC_LOCK_ACTION,         /* PC在跑动作流程 */
} pc_lock_type_t;

/* ---- PC命令上下文 ---- */
typedef struct {
    volatile pc_arbiter_lock_t lock;         /* 当前锁持有者 */
    volatile pc_lock_type_t    pc_lock_type; /* PC锁类型 */
    volatile uint32_t idle_ticks;           /* 当前持有者连续空闲的毫秒数 */
    volatile uint32_t action_start_tick;    /* 动作流程启动时刻 */
    volatile uint8_t  pc_cmd_pending;       /* 本周期是否有PC命令到达 */
    volatile uint8_t  remote_active;        /* 本周期遥控器是否有动作 */
} pc_arbiter_ctx_t;

extern volatile pc_arbiter_ctx_t g_pc_arbiter;

/* ---- API ---- */
void cmd_dispatch(uint8_t cmd, const uint8_t *payload);
void pc_arbiter_tick(void);

/* ---- 仲裁常量 ---- */
#define ARBITER_IDLE_TIMEOUT_MS    500U   /* 连续空闲超时解锁 */
#define ARBITER_ACTION_TIMEOUT_MS  30000U /* 动作流程最大保底超时 */
#define ARBITER_DEADBAND           150U   /* 摇杆中位死区 */
#define ARBITER_VEL_DEADBAND       0.005f /* 速度指令零死区 */

#endif /* CMD_DISPATCH_H */
