#ifndef APP_ZONE1_H
#define APP_ZONE1_H

#include <stdint.h>

typedef struct
{
    float forward_target_x_m;
    float forward_target_y_m;
    float forward2_target_x_m;
    float forward2_target_y_m;
    float step_start_target_x_m;
    float step_start_target_y_m;

    float shift_right_slow_cmd;   /* 蓝区：右移 vz；红区运行时取反（左移） */
    float shift_right_vy_comp_cmd; /* 横移时 vy 补偿，红蓝区同值不镜像 */
    float back_slow_cmd;
    float forward_slow_cmd;
    float limit_meas_rpm_thr;
    float limit_cmd_thr;
    uint32_t limit_debounce_ms;
    uint32_t limit_timeout_ms;
    uint32_t clamp_timeout_ms;
    uint32_t clamp_upright_hold_dwell_ms;
    uint32_t r1_wait_timeout_ms;
    uint32_t action_timeout_ms;
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
uint8_t AppZone1_SetForwardTarget(float x_m, float y_m);
uint8_t AppZone1_SetForward2Target(float x_m, float y_m);

extern volatile AppZone1Config g_app_zone1_cfg;

#endif /* APP_ZONE1_H */
