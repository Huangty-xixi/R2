#ifndef __CHASSIS_HEADING_HOLD_H__
#define __CHASSIS_HEADING_HOLD_H__

#include <stdint.h>

/* 航向保持参数（可在线调） */
typedef struct
{
    volatile float kp;                  /* 比例增益：角度误差纠偏力度 */
    volatile float ki;                  /* 积分增益：消除长期静差 */
    volatile float kd;                  /* 微分增益：抑制摆动（配合角速度） */
    volatile float i_limit;             /* 积分项限幅，防积分饱和 */
    volatile float out_limit;           /* 总输出限幅（叠加到Vx_in的最大修正） */

    float yaw_ref_deg;                 /* 参考航向角（deg） */
    float i_term;                       /* 当前积分项累计值 */
    float last_yaw_deg;                 /* 上一拍航向角（deg） */
    float yaw_rate_lpf;                 /* 滤波后的角速度（deg/s） */
    volatile float yaw_rate_lpf_alpha; /* 0~1, 越大越“跟随” */

    uint32_t last_tick_ms;              /* 上一拍时间戳（ms） */
    uint8_t yaw_inited;                 /* 初始化标志：0未锁参考，1已锁 ，车开始平移时会从0变成1，角度控制pid开始工作*/
} ChassisHeadingHold;

/* 供调试器/在线调参直接访问（定义在 chassis_heading_hold.c） */
extern volatile ChassisHeadingHold g_heading_hold;

/* 逐轴加速度限幅（速度斜坡）：y以 a_max 约束跟随 target */
typedef struct
{
    volatile float a_max;      /* 最大变化率（单位：目标量/秒） */
    float y;                   /* 当前输出 */
    uint32_t last_tick_ms;     /* 上一拍时间戳（ms） */
    uint8_t yaw_inited;        /* 初始化标志：0未锁参考，1已锁 ，车开始平移时会从0变成1，角度控制pid开始工作*/
} ChassisAxisLimiter;

/* 逐轴限幅参数（定义在 chassis_heading_hold.c） */
extern volatile ChassisAxisLimiter g_vy_limiter;
extern volatile ChassisAxisLimiter g_vw_limiter;
extern volatile ChassisAxisLimiter g_vx_limiter;

void ChassisAxisLimiter_Reset(ChassisAxisLimiter *lim, float y0);
float ChassisAxisLimiter_Update(ChassisAxisLimiter *lim, float target);

/* 平移锁角保持的输入死区（可在线调，定义在 chassis_heading_hold.c） */
extern volatile float g_heading_hold_trans_deadband;
extern volatile float g_heading_hold_rot_deadband;
extern volatile uint32_t g_heading_hold_release_delay_ms; /* 摇杆回中后延时退出阈值（ms） */

// void ChassisHeadingHold_Init(ChassisHeadingHold *hh,
//                              float kp, float ki, float kd,
//                              float i_limit, float out_limit,
//                              float yaw_rate_lpf_alpha);

/* 不保持航向/需要重置参考时调用 */
void ChassisHeadingHold_ResetRef(ChassisHeadingHold *hh, float yaw_deg);

/* 平移时角度保持（不使用运动方向解算）：
 * - vy_cmd/vw_cmd：平移输入（同 Chassis.param.Vy_in / Vw_in 单位）
 * - vx_cmd：旋转输入（同 Chassis.param.Vx_in 单位），用于判断“旋转介入则退出保持”
 * - yaw_body_deg：当前机身航向角（已包含安装偏角，例如 yaw + 9°）
 * 返回值：需要叠加到旋转通道的修正量
 */
float ChassisHeadingHold_TranslationHoldStep(ChassisHeadingHold *hh,
                                            float yaw_body_deg,
                                            float vx_cmd,
                                            float vy_cmd,
                                            float vw_cmd);

/* 平面解耦（前后<->左右）：
 * - 在 chassis_heading_hold.c 内部使用四轮 speed_rpm 反解得到的“估计速度反馈”做慢速trim
 * - 入口放在此头文件，底盘每周期调用一次即可
 * - vy_cmd/vw_cmd 为输入输出（就地修改）
 */
void ChassisDecouple_Apply(float vx_cmd, float *vy_cmd, float *vw_cmd);

#endif
