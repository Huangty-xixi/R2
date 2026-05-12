#ifndef APP_ZONE1_CLAMP_HEAD_FLOW_H
#define APP_ZONE1_CLAMP_HEAD_FLOW_H

#include <stdint.h>

typedef struct
{
    float forward_target_x_m;       //前进x坐标
    float forward_target_y_m;       //前进y坐标

    float backoff_dist_m;           //后退距离
    float back_slow_dist_m;         //后退慢速距离

    float shift_right_cmd;          //右移命令
    float back_slow_cmd;            //后退慢速命令                      

    float limit_meas_rpm_thr;       //限制测量rpm阈值
    float limit_cmd_thr;            //限制命令阈值
    uint32_t limit_debounce_ms;     //限制debounce时间
    uint32_t limit_timeout_ms;      //限制超时时间

    uint32_t clamp_timeout_ms;      //夹爪超时时间
    uint32_t dock_timeout_ms;       //对接超时时间
    uint32_t action_timeout_ms;     //动作超时时间

    /**
     * 导航子状态（到点/后退导航）允许的里程计最大龄期（ms）。
     * 0 表示仅使用 rc_odom_is_valid()，不做龄期判据。
     */
    uint32_t nav_odom_max_age_ms;
} AppZone1ClampHeadFlowConfig;

/**
 * @brief 1区流程单步调试控制块（仅用于调试观测/手动放行）
 *
 * 用法：
 * - 置 @c enable=1 打开单步模式；
 * - 每次希望流程“推进一步（完成一个状态并跳转到下一状态后暂停）”，将 @c allow 置 1；
 * - 每次发生状态跳转时，内部会自动将 @c allow 清 0。
 */
typedef struct
{
    volatile uint32_t enable;
    volatile uint32_t allow;
    volatile uint32_t last_from_state;
    volatile uint32_t last_to_state;
    volatile uint32_t last_transition_ms;
} AppZone1ClampHeadFlowStepCtrl;

typedef struct
{
    volatile uint32_t enable;   //使能标志
    volatile uint32_t seq;      //序列号    
    volatile uint32_t now_ms;   //当前时间
    volatile uint32_t state;    //状态
    volatile uint32_t busy;     //忙碌标志
    volatile uint32_t done;     //完成标志
    volatile uint32_t failed;   //失败标志
    volatile uint32_t step_enable; //单步模式开关（镜像）
    volatile uint32_t step_allow;  //单步放行标志（镜像）
    volatile float cmd_vy;      //命令vy
    volatile float cmd_vw;      //命令vw
    volatile float meas_chassis_rpm_abs; //测量底盘rpm绝对值
    volatile float target_x_m; //目标x坐标
    volatile float target_y_m; //目标y坐标      
    volatile float target_z_m; //目标z坐标
} AppZone1ClampHeadFlowDebug;

void AppZone1ClampHeadFlow_Init(void);                                                            
void AppZone1ClampHeadFlow_Start(void);                         
void AppZone1ClampHeadFlow_Run(void);        
void AppZone1ClampHeadFlow_Reset(void);
void AppZone1ClampHeadFlow_NotifyDockOk(void);
uint8_t AppZone1ClampHeadFlow_IsBusy(void);
uint8_t AppZone1ClampHeadFlow_IsDone(void);
uint8_t AppZone1ClampHeadFlow_IsFailed(void);   

/**
 * @brief 获取 1 区流程配置快照（只读拷贝）。
 * @param out 输出指针
 * @return 1=成功；0=空指针
 */
uint8_t AppZone1ClampHeadFlow_GetConfig(AppZone1ClampHeadFlowConfig *out);

/**
 * @brief 设置 1 区流程配置（带范围校验）。
 * @param cfg 输入配置
 * @return 1=成功；0=参数非法或空指针
 */
uint8_t AppZone1ClampHeadFlow_SetConfig(const AppZone1ClampHeadFlowConfig *cfg);

/**
 * @brief 设置 1 区入口导航目标（米）。
 * @return 1=成功；0=参数非法
 */
uint8_t AppZone1ClampHeadFlow_SetForwardTarget(float x_m, float y_m);

extern volatile AppZone1ClampHeadFlowConfig g_app_zone1_clamp_head_flow_cfg;
extern volatile AppZone1ClampHeadFlowStepCtrl g_app_zone1_clamp_head_flow_step;
extern volatile AppZone1ClampHeadFlowDebug g_app_zone1_clamp_head_flow_debug;

#endif /* APP_ZONE1_CLAMP_HEAD_FLOW_H */
