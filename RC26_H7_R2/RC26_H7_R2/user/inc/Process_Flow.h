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

extern ProcessFlowChassisOverride process_flow_chassis_override;

void Process_UpStairs(void);
void Process_DownStairs(void);
void Process_GetKFS(void);
void Process_PutKFS(void);

#endif
