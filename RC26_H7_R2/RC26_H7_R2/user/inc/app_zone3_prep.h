/**
 * @file app_zone3_prep.h
 * @brief 三区技能赛预备阶段：等R1上坡→自己上坡→取KFS2→取KFS3→导航到出口→交棒三区
 */
#ifndef APP_ZONE3_PREP_H
#define APP_ZONE3_PREP_H

#include <stdint.h>

typedef enum {
    app_zone3_prep_state_idle = 0,
    app_zone3_prep_state_wait_r1_upslope,   /* 等待R1先上坡(APP_Z3_PREP_WAIT_R1_MS) */
    app_zone3_prep_state_upslope,           /* 上坡(Process_UpSlope) */
    app_zone3_prep_state_nav_to_p5,         /* 新:上坡后导航到P5 */
    app_zone3_prep_state_nav_to_p1,         /* 新:从P5导航到P1 */
    app_zone3_prep_state_done,              /* 完成：交棒AppZone3_Start */
    app_zone3_prep_state_failed,            /* 失败 */
} app_zone3_prep_state_t;

void AppZone3Prep_Init(void);
void AppZone3Prep_Start(void);
void AppZone3Prep_Reset(void);
void AppZone3Prep_Run(void);

uint8_t AppZone3Prep_IsActive(void);
uint8_t AppZone3Prep_IsDone(void);
uint8_t AppZone3Prep_IsFailed(void);

#endif /* APP_ZONE3_PREP_H */
