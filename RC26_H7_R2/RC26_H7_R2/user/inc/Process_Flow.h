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
    uint32_t wait_before_fall_ms;
    uint32_t wait_fall_done_ms;
    float vy_forward;
    uint32_t chassis_forward_post_ms; /* 落台等待结束后前进保持（ms） */
    float vy_chassis_forward_post;    /* 落台等待结束后前进 vy */
} ProcessUpstairsTune;

typedef struct
{
    uint32_t fast_raise_back_ms;
    uint32_t stop_before_fall_ms;
    uint32_t wait_fall_done_ms;
    float vy_backward;
    float pitch_abs_rise_th_deg;   /* Plan0：|pitch| 相对起点增大阈值（度） */
    float pitch_abs_fall_th_deg;   /* Plan0：|pitch| 相对峰值回落阈值（度） */
    uint8_t fall_confirm_cnt;      /* Plan0：俯仰回落连续判定次数 */
    uint32_t wait_after_pitch_fall_ms; /* Plan0：俯仰回落后再等待（ms） */
    float vy_backward_after_pitch;     /* Plan0：俯仰回落等待结束后的后退 vy */
} ProcessDownstairsTune;

/**
 * @brief Plan B：Plan A 俯仰段 + wait 后倒车测距；PROCESS_FLOW_DOWNSTAIRS_PLAN=1。
 *        俯仰阈值等同 @ref g_process_downstairs_tune；wait 后 vy_rev 倒车，
 *        激光突增或 laser_rev_timeout_ms 超时则停车 fall_fast。
 */
typedef struct
{
    volatile uint32_t laser_rev_timeout_ms; /* wait 后开始倒车计时，超时 fall_fast */
    volatile uint32_t vy_rev_first_ms;      /* 兼容 Watch，同 laser_rev_timeout_ms */
    volatile uint32_t wait_after_sudden_stop_ms; /* 未使用 */
    volatile uint32_t raise_hold_ms;        /* 未使用 */
    volatile uint32_t vy_rev_second_ms;     /* 未使用 */
    volatile uint32_t after_clear_before_fall_ms; /* 未使用 */
    volatile uint32_t fall_hold_ms;         /* 未使用 */
    volatile float vy_rev;                  /* wait 后倒车 vy，默认 -20 */
    volatile float vy_rev_after_raise;    /* 未使用 */
} ProcessDownstairsPlanBTune;

/**
 * @brief Plan C 下台阶：先前进再后退（timed），再抬升、再退、快降；PROCESS_FLOW_DOWNSTAIRS_PLAN=2 时使用。
 */
typedef struct
{
    volatile uint32_t vy_fwd_ms;
    volatile uint32_t vy_rev_first_ms;
    volatile uint32_t raise_hold_ms;
    volatile uint32_t vy_rev_second_ms;
    volatile uint32_t after_clear_before_fall_ms;
    volatile uint32_t fall_hold_ms;
    volatile float vy_fwd;
    volatile float vy_rev;
    volatile float vy_rev_after_raise;
} ProcessDownstairsPlanCTune;

/** GetKFS 状态机各步等待时间（ms）与底盘 vy，可在线调参 */
typedef struct
{
    volatile uint32_t spin_front_to_p2_ms;
    volatile uint32_t chassis_forward_ms;
    volatile uint32_t spin_front_to_p1_ms;
    volatile uint32_t wait_after_close_s1_ms;
    volatile uint32_t wait_main_lift_p1_ms;   /* 主轴到 p1 等待（ms） */
    volatile uint32_t wait_front_p2_done_ms;
    volatile float vy_chassis_forward;        /* 取 KFS 底盘前进 vy */
} ProcessGetKfsTune;

/** PutKFS 状态机各阶段等待时间（ms），Watch 在线调参 */
typedef struct
{
    volatile uint32_t wait_pre_first_ms;   /* 首轮等step1到位(ms) */
    volatile uint32_t wait_pre_fast_ms;    /* 后续轮等step1到位(ms) */
    volatile uint32_t wait_above_ms;       /* 等kfs_above伸出到位(ms) */
} ProcessPutKfsTune;

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
    upstairs_step_wait_before_fall,
    upstairs_step_wait_fall_done,
    upstairs_step_chassis_forward_post,
    upstairs_step_wait_chassis_forward_post
} UpstairsStep;

typedef enum
{
    downstairs_step_idle = 0,
    downstairs_step_wait_pitch_rise,      /* Plan0 */
    downstairs_step_wait_pitch_fall,      /* Plan0 */
    downstairs_step_wait_after_pitch_fall,/* Plan0 */
    downstairs_step_fast_raise_back,
    downstairs_step_stop_before_fall,
    downstairs_step_wait_fall_done,
    /* Plan B：PROCESS_FLOW_DOWNSTAIRS_PLAN=1 */
    downstairs_step_b_vy_rev_until_sudden,
    downstairs_step_b_wait_after_sudden_stop,
    downstairs_step_b_raise_hold_15s,
    downstairs_step_b_vy_rev_2s,
    downstairs_step_b_wait_after_clear_before_fall,
    downstairs_step_b_fall_hold_1s,
    /* Plan C：PROCESS_FLOW_DOWNSTAIRS_PLAN=2 */
    downstairs_step_c_vy_fwd,
    downstairs_step_c_vy_rev_first,
    downstairs_step_c_raise_hold,
    downstairs_step_c_vy_rev_second,
    downstairs_step_c_wait_before_fall,
    downstairs_step_c_fall_hold
} DownstairsStep;

typedef enum
{
    get_kfs_step_idle = 0,
    get_kfs_step_spin_front_to_p2,
    get_kfs_step_chassis_forward,
    get_kfs_step_spin_front_to_p1,
    get_kfs_step_wait_after_close_s1,
    get_kfs_step_main_lift_to_p1,
    get_kfs_step_wait_front_p2_done,
    get_kfs_step_done
} GetKfsStep;

typedef enum
{
    put_kfs_step_idle = 0,
    put_kfs_step_pre_position,  /* step1: main_lift->P4, first round also rotates three_kfs */
    put_kfs_step_wait_pre,      /* wait step1 done(1st 1s/subseq 0.5s)->close sucker+kfs_above->P3 */
    put_kfs_step_wait_above,    /* wait 2s->kfs_above->P1 + pre-rotate three_kfs for next */
    put_kfs_step_done
} PutKfsStep;

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
    volatile uint32_t put_kfs_round;
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
extern volatile ProcessDownstairsPlanBTune g_process_downstairs_plan_b_tune;
extern volatile ProcessDownstairsPlanCTune g_process_downstairs_plan_c_tune;
extern volatile ProcessGetKfsTune g_process_get_kfs_tune;
extern PutKfsStep put_kfs_step;
extern volatile ProcessPutKfsTune g_process_put_kfs_tune;

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
void Process_UpSlope(void);
uint8_t Process_UpSlope_IsBusy(void);
void Process_UpSlope_Reset(void);
void Process_Flow_ResetAll(void);
void Process_Flow_DebugSnapshot(void);

#endif