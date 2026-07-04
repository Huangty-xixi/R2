#ifndef __PROCESS_FLOW_H__
#define __PROCESS_FLOW_H__

#include <stdint.h>

#include "app_zone2.h"

/* 全自动流程对底盘指令的按轴覆盖控制 */
#define PROCESS_FLOW_CHASSIS_OVERRIDE_VX  (1U << 0)
#define PROCESS_FLOW_CHASSIS_OVERRIDE_VY  (1U << 1)
#define PROCESS_FLOW_CHASSIS_OVERRIDE_VW  (1U << 2)
#define PROCESS_FLOW_OVERRIDE_PRIORITY_LOW  0U
#define PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH 1U

/* 与 app_zone2_field_dir_t 完全一致，上坡摆头与二区语义统一 */
typedef app_zone2_field_dir_t ProcessFlowYawRef;

typedef struct
{
    float p1_x_m;
    float p1_y_m;
    float yaw_tol_deg;
    float vy_target;              /* 上坡纵向速度命令（对前结束后恒定） */
    uint32_t wait_after_goto_ms;  /* 到点完成后等待再摆头/上坡（ms） */
    float pitch_abs_rise_th_deg;  /* |pitch| 相对起点增大阈值（度） */
    float pitch_abs_fall_th_deg;  /* |pitch| 相对峰值回落阈值（度） */
    uint8_t fall_confirm_cnt;     /* 连续判定次数 */
    uint32_t stage_timeout_ms;
} ProcessUpSlopeTune;

typedef struct
{
    uint32_t chassis_forward_pre_ms; /* 抬升前底盘前进（ms） */
    float vy_chassis_forward_pre;    /* 抬升前底盘前进 vy */
    uint32_t wait_raise_done_ms;
    uint32_t fast_before_fall_ms;    /* 下降前快速前进时间(ms) */
    float    vy_fast_before_fall;    /* 下降前快速前进 vy */
    uint32_t wait_before_fall_ms;
    uint32_t wait_fall_done_ms;
    float vy_forward;
    uint32_t chassis_forward_post_ms; /* 落台等待结束后前进保持（ms） */
    float vy_chassis_forward_post;    /* 落台等待结束后前进 vy */
} ProcessUpstairsTune;

/* 下台阶流程参数（单一结构体，Watch 可调） */
typedef struct
{
    /* pitch 检测倒车阶段 */
    volatile float    vy_backward;               /* 倒车速度，默认 -50 */
    volatile float    pitch_abs_rise_th_deg;     /* pitch 抬起阈值(°)，默认 5.0 */
    volatile float    pitch_abs_fall_th_deg;     /* pitch 回落阈值(°)，默认 5.0 */
    volatile uint8_t  fall_confirm_cnt;          /* 回落确认次数，默认 1 */
    volatile uint32_t pitch_rise_timeout_ms;     /* pitch 抬起超时(ms)，默认 2000 */
    volatile uint32_t pitch_fall_timeout_ms;     /* pitch 回落超时(ms)，默认 2000 */
    volatile uint32_t wait_after_pitch_fall_ms;  /* pitch 回落后等待(ms)，默认 200 */

    /* 激光倒车阶段 */
    volatile float    vy_rev_fast;               /* 激光倒车快退速度，默认 -120 */
    volatile uint32_t vy_rev_fast_ms;            /* 激光倒车快退时间(ms)，默认 100 */
    volatile float    vy_rev;                    /* 激光倒车速度，默认 -40 */
    volatile uint32_t laser_rev_timeout_ms;      /* 激光倒车超时(ms)，默认 1500 */

    /* 清障 & 掉落阶段 */
    volatile uint32_t after_clear_before_fall_ms; /* 清障后等待(ms)，默认 100 */
    volatile uint32_t wait_fall_done_ms;          /* 掉落等待(ms)，默认 300 */
} ProcessDownstairsTune;


/** GetKFS 状态机各步等待时间（ms）与底盘 vy，可在线调参 */
typedef struct
{
    volatile uint32_t wait_spin_p2_ms;          /* 转臂→p2 等待(ms)，默认 231 */
    volatile uint32_t spin_front_to_p2_ms;
    volatile uint32_t chassis_forward_ms;
    volatile uint32_t wait_after_chassis_forward_ms;
    volatile uint32_t wait_before_sucker_off_ms;
    volatile uint32_t wait_after_sucker_off_ms;
    volatile uint32_t wait_after_close_s1_ms;
    volatile uint32_t wait_front_p2_done_ms;
    volatile uint32_t spin_back_to_p1_ms;
    volatile float vy_chassis_forward;        /* 取 KFS 底盘前进 vy */
} ProcessGetKfsTune;

/** PutKFS 状态机各阶段等待时间（ms），Watch 在线调参 */
typedef struct
{
    volatile uint32_t wait_extend_ms;      /* 等kfs_above=P3伸出到位(ms)，默认2000 */
    volatile uint32_t wait_sucker_close_ms;/* 关吸盘后等待(ms)，默认500 */
    volatile uint32_t wait_retract_ms;     /* 等kfs_above=P1缩回到位(ms)，默认1000 */
} ProcessPutKfsTune;

/** 上R1爬升流程参数（Watch 在线调参） */
typedef struct
{
    volatile uint32_t raise_wait_ms;      /* 抬升等待(ms) */
    volatile uint32_t fast_fwd_ms;        /* 快速前进时长(ms) */
    volatile float    fast_fwd_vy;        /* 快速前进速度 */
    volatile uint32_t slow_fwd_ms;        /* 慢速前进时长(ms) */
    volatile float    slow_fwd_vy;        /* 慢速前进速度 */
    volatile uint32_t fall_wait_ms;       /* 下降等待(ms) */
    volatile uint32_t post_fwd_ms;        /* 降后微进时长(ms) */
    volatile float    post_fwd_vy;        /* 降后微进速度 */
} ProcessUpR1Tune;

typedef struct
{
    uint8_t axis_mask;
    uint8_t priority;     /* 当前激活轴中的最高优先级，兼容旧 Watch */
    uint8_t priority_vx;  /* 按轴优先级：VX 旋转 */
    uint8_t priority_vy;  /* 按轴优先级：VY 前后 */
    uint8_t priority_vw;  /* 按轴优先级：VW 横移 */
    float vx;
    float vy;
    float vw;
} ProcessFlowChassisOverride;

typedef enum
{
    upstairs_step_chassis_forward_pre = 0,
    upstairs_step_wait_chassis_forward_pre,
    upstairs_step_idle,
    upstairs_step_wait_raise_done,
    upstairs_step_wait_fast_before_fall,
    upstairs_step_wait_before_fall,
    upstairs_step_wait_fall_done,
    upstairs_step_chassis_forward_post,
    upstairs_step_wait_chassis_forward_post
} UpstairsStep;

typedef enum
{
    downstairs_step_idle = 0,
    downstairs_step_wait_pitch_rise,
    downstairs_step_wait_pitch_fall,
    downstairs_step_wait_after_pitch_fall,
    downstairs_step_vy_rev_until_sudden,
    downstairs_step_wait_after_clear_before_fall,
    downstairs_step_wait_fall_done
} DownstairsStep;

typedef enum
{
    get_kfs_step_idle = 0,
    get_kfs_step_wait_spin_p2,          /* 转臂→p2，等 231ms */
    get_kfs_step_spin_front_to_p2,
    get_kfs_step_chassis_forward,
    get_kfs_step_wait_after_chassis_forward,
    get_kfs_step_wait_after_sucker_off,
    get_kfs_step_wait_after_close_s1,
    get_kfs_step_wait_front_p2_done,
    get_kfs_step_spin_back_to_p1,
    get_kfs_step_done
} GetKfsStep;

typedef enum
{
    put_kfs_step_idle = 0,
    put_kfs_step_extend,         /* kfs_above=P3伸出, 等2s */
    put_kfs_step_sucker_wait,    /* 关吸盘, 等500ms → kfs_above=P1缩回 */
    put_kfs_step_retract,        /* kfs_above=P1缩回+释放底盘, 等1s → three_kfs-- */
    put_kfs_step_done
} PutKfsStep;

typedef enum
{
    up_r1_step_idle = 0,
    up_r1_step_wait_raise,
    up_r1_step_fast_fwd,
    up_r1_step_slow_fwd,
    up_r1_step_fall,
    up_r1_step_wait_fall,
    up_r1_step_post_fwd,
    up_r1_step_done
} UpR1Step;

/* 调试：流程步骤追踪（用于防优化观察） */
typedef struct
{
    volatile uint32_t enable;       /* 0=关；非0=开 */
    volatile uint32_t seq;          /* 每次写入+1，便于看有没有刷新 */
    volatile uint32_t now_tick;     /* osKernelGetTickCount() */

    /* 上/下台阶步骤 */
    volatile uint32_t upstairs_step;
    volatile uint32_t downstairs_step;
    volatile uint32_t get_kfs_step;
    volatile uint32_t get_kfs_round;
    volatile uint32_t put_kfs_step;
    volatile uint32_t up_r1_step;
    volatile uint32_t upslope_step;



    /* 关键判定量快照（避免断点时变量被优化/合并） */
    volatile uint32_t lift_has_stopped;
    volatile uint32_t r2_lift_mode;
    volatile uint32_t lift_rise_fast;
    volatile uint32_t lift_fall_fast;
    volatile uint32_t lift_stop_mode;
    volatile uint32_t lift_running;

    /* 底盘覆盖输出快照 */
    volatile uint32_t axis_mask;
    volatile uint32_t priority;
    volatile uint32_t priority_vx;
    volatile uint32_t priority_vy;
    volatile uint32_t priority_vw;
    volatile float vx;
    volatile float vy;
    volatile float vw;
} ProcessFlowDebug;

extern UpstairsStep upstairs_step;
extern DownstairsStep downstairs_step;
extern GetKfsStep get_kfs_step;
extern ProcessFlowChassisOverride process_flow_chassis_override;
extern volatile ProcessFlowDebug process_flow_debug;
extern volatile ProcessUpSlopeTune g_process_upslope_tune;
extern volatile ProcessUpstairsTune g_process_upstairs_tune;
extern volatile ProcessDownstairsTune g_process_downstairs_tune;
extern volatile ProcessGetKfsTune g_process_get_kfs_tune;
extern PutKfsStep put_kfs_step;
extern volatile ProcessPutKfsTune g_process_put_kfs_tune;
extern uint8_t g_process_skip_upstairs_fwd;   /* 1=同桩跳过前进，直接raise */
extern UpR1Step up_r1_step;
extern volatile ProcessUpR1Tune g_process_up_r1_tune;

/** 按轴写入全自动流程底盘覆盖；优先级低的写入不能覆盖同轴高优先级。 */
void Process_Flow_SetChassisOverrideAxes(uint8_t axis_mask, uint8_t priority, float vx, float vy, float vw);
uint8_t Process_Flow_ChassisOverrideCanWrite(uint8_t axis_mask, uint8_t priority);
void Process_Flow_ClearChassisOverrideAxesByPriority(uint8_t axis_mask, uint8_t max_priority);
void Process_Flow_ClearChassisOverrideAxes(uint8_t axis_mask);
/** 清除全自动流程底盘三轴覆盖（与 @c Chassis_Calc 中 override 读取一致） */
void Process_Flow_ClearChassisOverride(void);

void Process_UpStairs(void);
uint8_t Process_UpStairs_IsBusy(void);
void Process_DownStairs(void);
uint8_t Process_DownStairs_IsBusy(void);
void Process_GetKFS(app_zone2_get_kfs_rel_t rel);
uint8_t Process_GetKFS_IsBusy(void);
/** 1=前顶结束且仍在后半段收臂/升主轴等；0=未进入前顶完成态或已 idle */
uint8_t Process_GetKFS_IsChassisForwardDone(void);
void Process_PutKFS(void);
uint8_t Process_PutKFS_IsBusy(void);
void Process_PutKFS_AbortAndRollback(void);
void Process_UpR1(void);
uint8_t Process_UpR1_IsBusy(void);
void Process_UpSlope(void);
uint8_t Process_UpSlope_IsBusy(void);
void Process_UpSlope_Reset(void);
void Process_Flow_ResetAll(void);
void Process_Flow_DebugSnapshot(void);

#endif