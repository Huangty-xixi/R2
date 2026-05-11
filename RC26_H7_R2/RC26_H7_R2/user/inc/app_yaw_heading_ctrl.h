#ifndef APP_YAW_HEADING_CTRL_H
#define APP_YAW_HEADING_CTRL_H

#include <stdint.h>

typedef struct
{
    float kp;
    float kd;
    float max_speed;
    float dead_zone_deg;
} AppYawHeadingCtrlConfig;

typedef enum
{
    app_yaw_heading_cmd_none = 0,
    app_yaw_heading_cmd_turn_left_90,                                    // 左转 90 度
    app_yaw_heading_cmd_turn_right_90,                                   // 右转 90 度
    app_yaw_heading_cmd_turn_180,                                        // 转 180 度
} AppYawHeadingCmd;

/**
 * @brief 航向控制模块初始化（上电零点设为固定 0 度）。
 */
void AppYawHeadingCtrl_Init(void);

/**
 * @brief 获取航向控制参数快照（只读拷贝）。
 * @param out 输出指针
 * @return 1=成功，0=空指针
 */
uint8_t AppYawHeadingCtrl_GetConfig(AppYawHeadingCtrlConfig *out);

/**
 * @brief 设置航向控制参数（带范围校验）。
 * @param cfg 输入配置
 * @return 1=成功，0=参数非法或空指针
 */
uint8_t AppYawHeadingCtrl_SetConfig(const AppYawHeadingCtrlConfig *cfg);

/**
 * @brief 提交航向命令（左 90 / 右 90 / 180）。
 * @param cmd 命令枚举
 * @return 1=接受命令，0=参数非法或模块未初始化
 */
uint8_t AppYawHeadingCtrl_PostCommand(AppYawHeadingCmd cmd);

/**
 * @brief 航向控制循环函数（放在周期任务中反复调用）。
 */
void AppYawHeadingCtrl_Run(void);

/**
 * @brief 查询航向控制器是否正在执行转向。
 * @return 1=忙碌（正在转向），0=空闲
 */
uint8_t AppYawHeadingCtrl_IsBusy(void);

extern volatile AppYawHeadingCtrlConfig g_app_yaw_heading_ctrl_cfg;

#endif /* APP_YAW_HEADING_CTRL_H */
