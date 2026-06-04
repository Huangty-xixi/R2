#ifndef YAW_HEADING_CTRL_H
#define YAW_HEADING_CTRL_H

#include <stdint.h>

#include "app_zone2.h"

typedef struct
{
    float kp;
    float kd;
    float max_speed;
    float dead_zone_deg;
} YawHeadingCtrlConfig;

typedef enum
{
    yaw_heading_cmd_none = 0,
    yaw_heading_cmd_turn_left_90,                                    // 左转 90°
    yaw_heading_cmd_turn_right_90,                                   // 右转 90°
    yaw_heading_cmd_turn_180,                                        // 掉头 180°
} YawHeadingCmd;

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
 * @brief 提交离散转向命令：左转 90° / 右转 90° / 掉头 180°。
 * @param cmd 命令枚举
 * @return 1=已接受，0=未初始化或命令非法
 */
uint8_t YawHeadingCtrl_PostCommand(YawHeadingCmd cmd);

/**
 * @brief 周期运行：处理待执行命令，PD 跟踪目标航向并通过底盘 override 输出旋转速度。
 */
void YawHeadingCtrl_Run(void);

/**
 * @brief 按赛场「前后左右」设目标场向（与 app_zone2 梅林格语义一致）；不写电机，周期 PD 见 YawHeadingCtrl_Run。
 *
 * 场地 map：+x 向右、+y 向上；梅林邻格方向见 app_zone2.c field_dir_between_mf_cells。
 * 航向约定：0 = 朝场地「前」（app_zone2 的 FRONT = +y），180 = 朝「后」（-y），
 * LEFT=红区-x、yaw +90°；RIGHT=+x、yaw -90°；邻格见 app_zone2.c field_dir_between。
 * 再换算到与 Init 时 yaw_zero 一致的归一化目标。
 * SKIP：停止跟踪并清除底盘 override。
 *
 * @note 与 odom 类似：此处只改「目标」，manual_chassis_function 内每周期 YawHeadingCtrl_Run。
 * @note IMU yaw 需与上述世界轴一致；若仅上电相对零点，请先在对齐场地轴的姿态下 Init。
 */
void YawHeadingCtrl_RunFieldDir(app_zone2_field_dir_t dir);

/**
 * @brief 查询是否仍在跟踪目标航向或队列中仍有待处理命令。
 * @return 1=忙，0=空闲
 */
uint8_t YawHeadingCtrl_IsBusy(void);

extern volatile YawHeadingCtrlConfig g_yaw_heading_ctrl_cfg;

#endif /* YAW_HEADING_CTRL_H */
