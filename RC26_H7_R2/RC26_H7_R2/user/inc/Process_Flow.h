#ifndef __PROCESS_FLOW_H__
#define __PROCESS_FLOW_H__

#include <stdint.h>

/* 半自动流程对底盘指令的按轴覆盖控制 */
#define PROCESS_FLOW_CHASSIS_OVERRIDE_VX  (1U << 0)
#define PROCESS_FLOW_CHASSIS_OVERRIDE_VY  (1U << 1)
#define PROCESS_FLOW_CHASSIS_OVERRIDE_VW  (1U << 2)

typedef struct
{
    uint8_t axis_mask;
    float vx;
    float vy;
    float vw;
} ProcessFlowChassisOverride;

typedef enum
{
    upstairs_step_idle = 0,
    upstairs_step_wait_raise_done,
    upstairs_step_forward_on_raised,
    upstairs_step_wait_before_fall,
    upstairs_step_wait_fall_done
} UpstairsStep;

typedef enum
{
    downstairs_step_idle = 0,
    downstairs_step_fast_raise_back,
    downstairs_step_stop_before_fall,
    downstairs_step_wait_fall_done
} DownstairsStep;

/* 调试：流程步骤追踪（用于防优化观察） */
typedef struct
{
    volatile uint32_t enable;       /* 0=关；非0=开 */
    volatile uint32_t seq;          /* 每次写入+1，便于看有没有刷新 */
    volatile uint32_t now_tick;     /* osKernelGetTickCount() */

    /* 上/下台阶步骤 */
    volatile uint32_t upstairs_step;
    volatile uint32_t downstairs_step;

    /* 关键判定量快照（避免断点时变量被优化/合并） */
    volatile uint32_t lift_has_stopped;
    volatile uint32_t r2_lift_mode;
    volatile uint32_t lift_rise_fast;
    volatile uint32_t lift_fall_fast;
    volatile uint32_t lift_stop_mode;
    volatile uint32_t lift_running;

    /* 底盘覆盖输出快照 */
    volatile uint32_t axis_mask;
    volatile float vx;
    volatile float vy;
    volatile float vw;
} ProcessFlowDebug;

extern UpstairsStep upstairs_step;
extern DownstairsStep downstairs_step;
extern ProcessFlowChassisOverride process_flow_chassis_override;
extern volatile ProcessFlowDebug process_flow_debug;

void Process_UpStairs(void);
void Process_DownStairs(void);
void Process_GetKFS(void);
void Process_PutKFS(void);
void Process_Flow_ResetAll(void);
void Process_Flow_DebugSnapshot(void);

#endif
