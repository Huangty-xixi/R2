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
    upstairs_step_wait_fall_done
} UpstairsStep;

typedef enum
{
    downstairs_step_idle = 0,
    downstairs_step_fast_raise_back,
    downstairs_step_stop_before_fall,
    downstairs_step_wait_fall_done
} DownstairsStep;

extern UpstairsStep upstairs_step;
extern DownstairsStep downstairs_step;
extern ProcessFlowChassisOverride process_flow_chassis_override;

void Process_UpStairs(void);
void Process_DownStairs(void);
void Process_GetKFS(void);
void Process_PutKFS(void);

#endif
