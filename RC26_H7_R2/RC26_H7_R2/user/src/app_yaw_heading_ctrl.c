#include "app_yaw_heading_ctrl.h"

#include "Process_Flow.h"
#include "Sensor_Task.h"

#include <math.h>

#define APP_YAW_HEADING_KP                 (3.0f)                //比例增益
#define APP_YAW_HEADING_KD                 (0.20f)               //微分增益
#define APP_YAW_HEADING_MAX_SPEED          (80.0f)               //最大速度
#define APP_YAW_HEADING_DEAD_ZONE_DEG      (1.5f)               //死区角度

#define APP_YAW_HEADING_IDX_MAX            (4U)                //航向索引最大值

typedef struct
{
    uint8_t inited;                                             //初始化标志
    uint8_t enable;                                             //使能标志
    uint8_t heading_idx;                                        //航向索引
    float yaw_zero_deg;                                          //零点航向角
    float target_yaw_deg;                                         //目标航向角
    float error_deg;                                             //误差角度     
    AppYawHeadingCmd pending_cmd;                                 //待执行命令
} AppYawHeadingCtrlCtx;

static AppYawHeadingCtrlCtx g_app_yaw_heading_ctx;                 //航向控制上下文

static float app_yaw_heading_wrap_deg(float deg)                 //角度wrap
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static float app_yaw_heading_clampf(float x, float min_v, float max_v)    //clamp
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}
//  
static float app_yaw_heading_get_norm_yaw_deg(void)                  //获取归一化航向角
{
    const float raw_yaw_deg = g_sensor_task_data.imu.yaw_deg;
    return app_yaw_heading_wrap_deg(raw_yaw_deg - g_app_yaw_heading_ctx.yaw_zero_deg);
}

static void app_yaw_heading_apply_vx_only(float vx_cmd)             //应用vx命令
{
    process_flow_chassis_override.axis_mask =
        (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VX |
                  PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                  PROCESS_FLOW_CHASSIS_OVERRIDE_VW);
    process_flow_chassis_override.vx = vx_cmd;
    process_flow_chassis_override.vy = 0.0f;
    process_flow_chassis_override.vw = 0.0f;
}

static uint8_t app_yaw_heading_is_cmd_valid(AppYawHeadingCmd cmd)    //命令有效性检查
{
    return (uint8_t)((cmd == app_yaw_heading_cmd_turn_left_90) ||
                     (cmd == app_yaw_heading_cmd_turn_right_90) ||
                     (cmd == app_yaw_heading_cmd_turn_180));
}

static void app_yaw_heading_prepare_target_by_command(AppYawHeadingCmd cmd)   //根据命令准备目标航向角
{
    static const float heading_table_deg[APP_YAW_HEADING_IDX_MAX] = {0.0f, 90.0f, 180.0f, -90.0f};

    if (cmd == app_yaw_heading_cmd_turn_left_90)
    {
        g_app_yaw_heading_ctx.heading_idx = (uint8_t)((g_app_yaw_heading_ctx.heading_idx + 1U) % APP_YAW_HEADING_IDX_MAX);
    }
    else if (cmd == app_yaw_heading_cmd_turn_right_90)
    {
        g_app_yaw_heading_ctx.heading_idx =
            (uint8_t)((g_app_yaw_heading_ctx.heading_idx + (APP_YAW_HEADING_IDX_MAX - 1U)) % APP_YAW_HEADING_IDX_MAX);
    }
    else
    {
        g_app_yaw_heading_ctx.heading_idx = (uint8_t)((g_app_yaw_heading_ctx.heading_idx + 2U) % APP_YAW_HEADING_IDX_MAX);
    }

    g_app_yaw_heading_ctx.target_yaw_deg = heading_table_deg[g_app_yaw_heading_ctx.heading_idx];
    g_app_yaw_heading_ctx.enable = 1U;
}

void AppYawHeadingCtrl_Init(void)                                    //初始化
{
    g_app_yaw_heading_ctx.inited = 1U;
    g_app_yaw_heading_ctx.enable = 0U;
    g_app_yaw_heading_ctx.heading_idx = 0U;
    g_app_yaw_heading_ctx.yaw_zero_deg = g_sensor_task_data.imu.yaw_deg;
    g_app_yaw_heading_ctx.target_yaw_deg = 0.0f;
    g_app_yaw_heading_ctx.error_deg = 0.0f;
    g_app_yaw_heading_ctx.pending_cmd = app_yaw_heading_cmd_none;

    Process_Flow_ClearChassisOverride();
}

uint8_t AppYawHeadingCtrl_PostCommand(AppYawHeadingCmd cmd)          //提交命令
{
    if ((g_app_yaw_heading_ctx.inited == 0U) || (app_yaw_heading_is_cmd_valid(cmd) == 0U))
    {
        return 0U;
    }

    g_app_yaw_heading_ctx.pending_cmd = cmd;
    return 1U;
}

void AppYawHeadingCtrl_Run(void)                                    //运行          
{
    float norm_yaw_deg;
    float gyr_z_dps;
    float spd_cmd;

    if (g_app_yaw_heading_ctx.inited == 0U)
    {
        return;
    }

    if (g_app_yaw_heading_ctx.pending_cmd != app_yaw_heading_cmd_none)
    {
        app_yaw_heading_prepare_target_by_command(g_app_yaw_heading_ctx.pending_cmd);
        g_app_yaw_heading_ctx.pending_cmd = app_yaw_heading_cmd_none;
    }

    if (g_app_yaw_heading_ctx.enable == 0U)
    {
        return;
    }

    norm_yaw_deg = app_yaw_heading_get_norm_yaw_deg();
    g_app_yaw_heading_ctx.error_deg = app_yaw_heading_wrap_deg(g_app_yaw_heading_ctx.target_yaw_deg - norm_yaw_deg);

    if (fabsf(g_app_yaw_heading_ctx.error_deg) < APP_YAW_HEADING_DEAD_ZONE_DEG)
    {
        g_app_yaw_heading_ctx.enable = 0U;
        Process_Flow_ClearChassisOverride();
        return;
    }

    gyr_z_dps = g_sensor_task_data.imu.gyr_z_dps;
    spd_cmd = APP_YAW_HEADING_KP * g_app_yaw_heading_ctx.error_deg - APP_YAW_HEADING_KD * gyr_z_dps;
    spd_cmd = app_yaw_heading_clampf(spd_cmd, -APP_YAW_HEADING_MAX_SPEED, APP_YAW_HEADING_MAX_SPEED);

    app_yaw_heading_apply_vx_only(spd_cmd);
}
