#ifndef APP_FLOW_DISPATCH_H
#define APP_FLOW_DISPATCH_H

#include <stdint.h>
#include "R1_R2_connect.h"

void AppFlowDispatch_Init(void);
void AppFlowDispatch_Run(void);

/** R1Link 解码成功后写入二区 pending 任务 */
void AppFlowDispatch_OnR1WireMission(const r1_r2_mission_t *mission);

#endif /* APP_FLOW_DISPATCH_H */
