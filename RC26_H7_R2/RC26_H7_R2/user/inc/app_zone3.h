/**
 * @file app_zone3.h
 * @brief 三区 R1 指令业务：导航点1~4、PutKFS、UpStairs、STOP 回点1
 *
 * === 业务调用链 ===
 * IR帧接收(R1->R2):
 *   EE..FF(USART1): r1_zone3_parse_from_link_z3_cmd -> PostR1Cmd
 *   EE..FF(USART10): STOP->r1_zone3_parse_from_usart10_stop -> PostR1Cmd
 *   EE..FF(USART10): GET_KFS->r1_zone3_parse_from_link_z3_cmd -> PostR1Cmd
 *   55..AA(USART10): r1_zone3_parse_from_link_z3_put -> PostR1Cmd
 *
 * EE..FF 5B: wire 1~3 放P2/P3/P4（put_sub 00直放/01左偏/02右偏），4=STOP 5=上R1 6/7=取kfs；放3层走55..AA
 */
#ifndef APP_ZONE3_H
#define APP_ZONE3_H

#include <stdint.h>

#include "r1_link_z3_cmd.h"

typedef enum
{
    APP_Z3_CMD_NONE = 0,

    APP_Z3_CMD_PUT_KFS_P2, // 放2层左
    APP_Z3_CMD_PUT_KFS_P3, // 放2层中
    APP_Z3_CMD_PUT_KFS_P4, // 放2层右

    APP_Z3_CMD_STOP_ACTION, // 停止动作
    APP_Z3_CMD_UP_R1, // 上R1
    APP_Z3_CMD_PUT_KFS_ON_R1,         // 放3层(仅R1在位有效)
    APP_Z3_CMD_GET_KFS_G1,   // 取第一个地面KFS(导航到G1+转场前+p2)
    APP_Z3_CMD_GET_KFS_G2,   // 取第二个地面KFS(导航到G2+转场前+p2)
} app_zone3_cmd_id_t;
typedef struct
{
    app_zone3_cmd_id_t id;  // 指令ID
    uint8_t seq;  // 序列号，没有就填0
    uint8_t raw_cmd;  // wire cmd_id
    uint8_t put_sub;  // wire put_sub：00直放 01左偏 02右偏；其它命令00
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
    float g1_x_m; // 取kfs位置1 x
    float g1_y_m; // 取kfs位置1 y
    float g2_x_m; // 取kfs位置2 x
    float g2_y_m; // 取kfs位置2 y
    uint32_t up_r1_delay_ms; // 上R1前等待时间
    uint32_t nav_timeout_ms; // 导航超时时间
    uint32_t action_timeout_ms; // 动作超时时间
} AppZone3Config;

void AppZone3_Init(void); // 初始化
void AppZone3_Start(void); // 启动三区流程：先去点1，等待R1命令
void AppZone3_Reset(void); // 复位
void AppZone3_Run(void); // 运行

/** 链路层/中断内调用，锁存R1指令到中断安全缓冲 */
void AppZone3_PostR1Cmd(const app_zone3_r1_cmd_t *cmd);

uint8_t AppZone3_IsActive(void); // 是否激活
uint8_t AppZone3_IsDone(void); // 是否完成
uint8_t AppZone3_IsFailed(void); // 是否失败
uint8_t AppZone3_IsOnR1(void); // 是否在R1上

/** 放KFS动作是否忙（供上层判断是否可切区） */
uint8_t AppZone3_PutKFS_IsBusy(void);

extern volatile AppZone3Config g_app_zone3_cfg; // 配置

#endif /* APP_ZONE3_H */
