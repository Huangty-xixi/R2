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

/** 平移锁角保持：输入门限与摇杆回中后延时退出（可在线调） */
typedef struct
{
    volatile float trans_deadband;
    volatile float rot_deadband;
    volatile uint32_t release_delay_ms;
} ChassisHeadingHoldGate;

extern volatile ChassisHeadingHoldGate g_heading_hold_gate;

/** 平面 Vy/Vw 解耦 + 慢自适应 trim（可在线调） */
typedef struct
{
    volatile float k_yw_base;
    volatile float k_wy_base;
    volatile float k_yw_trim;
    volatile float k_wy_trim;
    volatile float trim_limit;
    volatile float k_total_limit;
    volatile float gamma_yw;
    volatile float gamma_wy;
    volatile float lpf_alpha;
    volatile float cmd_deadband;
    volatile float meas_min_rpm;
    volatile float yaw_rate_max_dps;
} ChassisDecoupleTune;

extern volatile ChassisDecoupleTune g_decouple_tune;

/** 起步/停车瞬态补偿（可在线调） */
typedef struct
{
    volatile float move_deadband;
    volatile float step_trigger;
    volatile uint32_t window_ms;
    volatile float yaw_damp_gain;
    volatile float vw_ff_gain;
    volatile float vy_ff_gain;
    volatile float amp_max;
    volatile float out_limit;
} ChassisTransientTune;

extern volatile ChassisTransientTune g_transient_tune;

/** 里程计漂移补偿（可在线调）：
 *  - 目标是让“纯前后/纯左右”指令下的横向漂移自动收敛
 *  - 仅用于平移精度修正，不替代导航位置环
 */
typedef struct
{
    volatile uint8_t enable;
    volatile float cmd_deadband;
    volatile float rot_deadband;
    volatile float kp_cross;
    volatile float ki_cross;
    volatile float i_limit;
    volatile float out_limit;
    volatile float max_dt_s;
} ChassisOdomDriftTune;

extern volatile ChassisOdomDriftTune g_odom_drift_tune;

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

/* 起步/停车瞬态补偿（用于抑制惯量扰动导致的瞬态偏航/侧偏）：
 * - 在平移命令突变的短时窗口内，输出额外Vx补偿
 * - 包含角速度阻尼项 + 方向相关前馈项
 */
float ChassisTransientComp_Update(float vx_cmd, float vy_cmd, float vw_cmd);

/** 里程计漂移补偿：
 *  - 根据里程计位姿差分估计车体系速度，抑制平移时的串轴漂移
 *  - @p vy_corr / @p vw_corr 为输出增量（调用方叠加到原命令）
 */
void ChassisOdomDriftComp_Update(float yaw_body_deg,
                                 float vx_cmd,
                                 float vy_cmd,
                                 float vw_cmd,
                                 float *vy_corr,
                                 float *vw_corr);

#endif
