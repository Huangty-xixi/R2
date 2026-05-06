#ifndef APP_YAW_HEADING_CTRL_H
#define APP_YAW_HEADING_CTRL_H

#include <stdint.h>

typedef enum
{
    app_yaw_heading_cmd_none = 0,
    app_yaw_heading_cmd_turn_left_90,
    app_yaw_heading_cmd_turn_right_90,
    app_yaw_heading_cmd_turn_180
} AppYawHeadingCmd;

/**
 * @brief 航向控制模块初始化（上电零点设为固定0°）。
 */
void AppYawHeadingCtrl_Init(void);

/**
 * @brief 提交航向命令（左90/右90/180）。
 * @param cmd 命令枚举
 * @return 1=接受命令；0=参数非法或模块未初始化
 */
uint8_t AppYawHeadingCtrl_PostCommand(AppYawHeadingCmd cmd);

/**
 * @brief 航向控制循环函数（放在周期任务中反复调用）。
 */
void AppYawHeadingCtrl_Run(void);

#endif /* APP_YAW_HEADING_CTRL_H */
