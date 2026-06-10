#ifndef YAW_HEADING_CTRL_H
#define YAW_HEADING_CTRL_H

#include <stdint.h>

#include "app_zone2.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_speed;
    float dead_zone_deg;
    /** 误差在死区内且角速度低于门限后，持续该时间才停控 (ms) */
    uint32_t arrival_dwell_ms;
    /** 允许判到位的最大 |gyr_z| (°/s)，抑制 ODOM 滞后时过早停控 */
    float arrival_rate_thr_dps;
    /** |error| 低于该值时按误差比例限速，0=关闭 */
    float slow_zone_deg;
    /** 四向档位滞回半宽 (°)，减小 45°/135° 边界抖档 */
    float cardinal_hyst_deg;
    float gyro_lpf_alpha;
    float ki_active_thr_deg;
    float kp_outer;
    float kp_inner;
    float ki_inner;
    float i_inner_limit;
    float max_rate_dps;
} YawHeadingCtrlConfig;

typedef enum
{
    yaw_heading_cmd_none = 0,
    /** 四向环：切到下一档（前→左→后→右→前） */
    yaw_heading_cmd_turn_left_90,
    /** 四向环：切到上一档（前→右→后→左→前） */
    yaw_heading_cmd_turn_right_90,
    /** 四向环：切到对面档 */
    yaw_heading_cmd_turn_180,
} YawHeadingCmd;

/** Keil Watch：航向控制实时快照（YawHeadingCtrl_Run 每周期刷新） */
typedef struct
{
    uint8_t enable;
    uint8_t heading_idx;   /* 0=前 1=左 2=后 3=右 */
    float norm_yaw_deg;
    float target_yaw_deg;
    float error_deg;
    float gyr_z_dps;
    float spd_cmd;
    uint32_t dead_zone_dwell_ms;
} yaw_heading_dbg_t;

/**
 * @brief 初始化：将当前 IMU 航向设为零点、目标为 0°，并清除底盘 override。
 */
void YawHeadingCtrl_Init(void);

/**
 * @brief 读取当前航向控制参数（拷贝到输出结构）。
 * @param out 输出缓冲区
 * @return 1=成功，0=失败（如 out 为空）
 */
uint8_t YawHeadingCtrl_GetConfig(YawHeadingCtrlConfig *out);

/**
 * @brief 写入航向控制参数（带合法性校验）。
 * @param cfg 配置指针
 * @return 1=成功，0=参数无效未写入
 */
uint8_t YawHeadingCtrl_SetConfig(const YawHeadingCtrlConfig *cfg);

/**
 * @brief 提交四向换档命令（目标恒为 0/±90/180° 之一，非物理相对转角）。
 * @param cmd 命令枚举
 * @return 1=已接受，0=未初始化或命令非法
 */
uint8_t YawHeadingCtrl_PostCommand(YawHeadingCmd cmd);

/**
 * @brief 周期运行：处理待执行命令，PI 跟踪目标航向并通过底盘 override 输出旋转速度。
 */
void YawHeadingCtrl_Run(void);

/**
 * @brief 按赛场「前后左右」设绝对四向目标（与 app_zone2 语义一致）；周期 PD 见 YawHeadingCtrl_Run。
 *
 * 航向约定：0=前，90=左，180=后，-90=右（归一化系，相对 Init 时 yaw_zero）。
 * SKIP：停止跟踪并清除底盘 override。
 */
void YawHeadingCtrl_RunFieldDir(app_zone2_field_dir_t dir);

/**
 * @brief 查询是否仍在跟踪目标航向或队列中仍有待处理命令。
 * @return 1=忙，0=空闲
 */
uint8_t YawHeadingCtrl_IsBusy(void);

/** 并发导航段切换前清零稳态计时（新段 begin 时调用） */
void YawHeadingCtrl_ParallelLegSettleReset(void);

/**
 * @brief 导航到点且航向停控后，|gyr| 低于门限并持续一小段时间。
 * @return 1=可切下一段，0=仍在稳态等待
 */
uint8_t YawHeadingCtrl_ParallelLegSettled(void);

extern volatile YawHeadingCtrlConfig g_yaw_heading_ctrl_cfg;
extern volatile yaw_heading_dbg_t g_yaw_heading_dbg;

#endif /* YAW_HEADING_CTRL_H */
