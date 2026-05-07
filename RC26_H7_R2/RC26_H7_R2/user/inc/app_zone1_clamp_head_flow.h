#ifndef APP_ZONE1_CLAMP_HEAD_FLOW_H
#define APP_ZONE1_CLAMP_HEAD_FLOW_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t enable;   //使能标志
    volatile uint32_t seq;      //序列号    
    volatile uint32_t now_ms;   //当前时间
    volatile uint32_t state;    //状态
    volatile uint32_t busy;     //忙碌标志
    volatile uint32_t done;     //完成标志
    volatile uint32_t failed;   //失败标志
    volatile float cmd_vy;      //命令vy
    volatile float cmd_vw;      //命令vw
    volatile float meas_chassis_rpm_abs; //测量底盘rpm绝对值
    volatile float target_x_m; //目标x坐标
    volatile float target_y_m; //目标y坐标
} AppZone1ClampHeadFlowDebug;

void AppZone1ClampHeadFlow_Init(void);                                                            
void AppZone1ClampHeadFlow_Start(void);                         
void AppZone1ClampHeadFlow_Run(void);        
void AppZone1ClampHeadFlow_Reset(void);
void AppZone1ClampHeadFlow_NotifyDockOk(void);
uint8_t AppZone1ClampHeadFlow_IsBusy(void);
uint8_t AppZone1ClampHeadFlow_IsDone(void);
uint8_t AppZone1ClampHeadFlow_IsFailed(void);   

extern volatile AppZone1ClampHeadFlowDebug g_app_zone1_clamp_head_flow_debug;

#endif /* APP_ZONE1_CLAMP_HEAD_FLOW_H */
