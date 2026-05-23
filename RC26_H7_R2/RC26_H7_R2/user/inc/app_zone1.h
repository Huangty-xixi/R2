#ifndef APP_ZONE1_H
#define APP_ZONE1_H

#include <stdint.h>

typedef struct
{
    /* 1 turn_left_90 */
    uint32_t action_timeout_ms;

    /* 2 back_slow_to_limit */
    float back_slow_cmd;

    /* 2~3~6 限位检测（慢退/右移/慢进共用） */
    float limit_meas_rpm_thr;
    float limit_cmd_thr;
    uint32_t limit_debounce_ms;
    uint32_t limit_timeout_ms;

    /* 3 shift_right_monitor */
    float shift_right_slow_cmd;    /* 蓝区：右移 vz；红区运行时取反（左移） */
    float shift_right_vy_comp_cmd; /* 横移时 vy 补偿，红蓝区同值不镜像 */

    /* 4 shift_right_clamp_wait */
    uint32_t clamp_timeout_ms;
    uint32_t clamp_upright_hold_dwell_ms;

    /* 5 forward2_advance */
    float forward2_advance_vy_cmd; /* 夹后延时前进 Vy，+前-后 */
    float forward2_advance_vw_cmd; /* 夹后延时左移 Vw，蓝区负/红区运行时取反 */
    uint32_t forward2_advance_ms;  /* 夹后前进+左移保持时间 ms */

    /* 6 turn_180（仅航向，配置见 action_timeout_ms） */

    /* 7 forward_slow_to_limit */
    float forward_slow_cmd;

    /* 8 wait_r1_release */
    uint32_t r1_wait_timeout_ms;

    /* 9 nav_to_step_start */
    float step_start_target_x_m;
    float step_start_target_y_m;
    uint32_t nav_odom_max_age_ms;
} AppZone1Config;

void AppZone1_Init(void);
void AppZone1_Start(void);
void AppZone1_Run(void);
void AppZone1_Reset(void);

void AppZone1_NotifyR1Release(void);

uint8_t AppZone1_IsBusy(void);
uint8_t AppZone1_IsDone(void);
uint8_t AppZone1_IsFailed(void);

uint8_t AppZone1_GetConfig(AppZone1Config *out);
uint8_t AppZone1_SetConfig(const AppZone1Config *cfg);
uint8_t AppZone1_SetForward2Advance(float vy_cmd, float vw_cmd, uint32_t advance_ms);

extern volatile AppZone1Config g_app_zone1_cfg;

#endif /* APP_ZONE1_H */
