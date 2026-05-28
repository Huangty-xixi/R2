/**
 * @file app_zone3.h
 * @brief 三区 R1 指令业务：导航点1~4、PutKFS、UpStairs、STOP 回点1
 * @note USART3 wire id 1~5；放三层仅 USART1 wire id=1
 */
#ifndef APP_ZONE3_H
#define APP_ZONE3_H

#include <stdint.h>

typedef enum
{
    APP_Z3_CMD_NONE = 0,

    APP_Z3_CMD_PUT_KFS_P2, // 放2层左
    APP_Z3_CMD_PUT_KFS_P3, // 放2层中
    APP_Z3_CMD_PUT_KFS_P4, // 放2层右

    APP_Z3_CMD_STOP_ACTION, // 停止动作
    APP_Z3_CMD_UP_R1, // 上R1
    APP_Z3_CMD_PUT_KFS_ON_R1, // 放3层
} app_zone3_cmd_id_t;

typedef struct
{
    app_zone3_cmd_id_t id;  // 指令ID
    uint8_t seq;  // 序列号，没有就填0
    uint8_t raw_cmd;  // 原始指令
} app_zone3_r1_cmd_t;

typedef struct
{
    float p1_x_m; // 导航点1 x坐标
    float p1_y_m; // 导航点1 y坐标
    float p2_x_m; // 导航点2 x坐标
    float p2_y_m; // 导航点2 y坐标
    float p3_x_m; // 导航点3 x坐标
    float p3_y_m; // 导航点3 y坐标
    float p4_x_m; // 导航点4 x坐标
    float p4_y_m; // 导航点4 y坐标
    uint32_t up_r1_delay_ms; // 上R1延迟时间
    uint32_t nav_timeout_ms; // 导航超时时间
    uint32_t action_timeout_ms; // 动作超时时间
} AppZone3Config;

void AppZone3_Init(void); // 初始化
void AppZone3_Start(void); // 进入三区：先导航到点1，再等待R1命令
void AppZone3_Reset(void); // 重置
void AppZone3_Run(void); // 运行

/** 解析层/中断上下文：只入队，不执行动作 */
void AppZone3_PostR1Cmd(const app_zone3_r1_cmd_t *cmd);

uint8_t AppZone3_IsActive(void); // 是否活动
uint8_t AppZone3_IsDone(void); // 是否完成
uint8_t AppZone3_IsFailed(void); // 是否失败
uint8_t AppZone3_IsOnR1(void); // 是否在R1

/** 放KFS完成判据预留，当前为空判据 */
uint8_t AppZone3_PutKFS_IsBusy(void);

extern volatile AppZone3Config g_app_zone3_cfg; // 配置

#endif /* APP_ZONE3_H */
