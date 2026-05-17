#include "app_yaw_heading_ctrl.h"

#include "Process_Flow.h"
#include "Sensor_Task.h"
#include "upper_pc_protocol.h"

#include <math.h>

// 航向索引最大值
#define APP_YAW_HEADING_IDX_MAX            (4U)
/* Wrap 边界归一容差：避免 -180/180 数值抖动导致误差突跳 */
#define APP_YAW_HEADING_WRAP_EPS_DEG       (1e-3f)

typedef struct
{
    uint8_t inited;                                             // 初始化标志
    uint8_t enable;                                             // 使能标志
    uint8_t heading_idx;                                        // 航向索引
    float yaw_zero_deg;                                          // 零点航向角
    float target_yaw_deg;                                         // 目标航向角
    float error_deg;                                             // 误差角
    AppYawHeadingCmd pending_cmd;                                 // 待执行命令
} AppYawHeadingCtrlCtx;

static volatile AppYawHeadingCtrlCtx g_app_yaw_heading_ctx;                 // 航向控制上下文

volatile AppYawHeadingCtrlConfig g_app_yaw_heading_ctrl_cfg = {
    .kp = 3.0f,
    .kd = 0.20f,
    .max_speed = 20.0f,
    .dead_zone_deg = 1.5f,
};

static uint8_t app_yaw_heading_cfg_is_valid(const AppYawHeadingCtrlConfig *cfg)
{
    if (cfg == 0)
    {
        return 0U;
    }
    if (!isfinite(cfg->kp) || cfg->kp < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->kd) || cfg->kd < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->max_speed) || cfg->max_speed <= 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->dead_zone_deg) || cfg->dead_zone_deg < 0.0f || cfg->dead_zone_deg > 30.0f)
    {
        return 0U;
    }
    return 1U;
}

static float app_yaw_heading_wrap_deg(float deg)                 // 角度 wrap to (-180,180]
{
    deg = fmodf(deg, 360.0f);
    if (deg <= -180.0f)
    {
        deg += 360.0f;
    }
    else if (deg > 180.0f)
    {
        deg -= 360.0f;
    }

    /* 将 -180 统一映射到 +180，避免边界来回翻转 */
    if (fabsf(deg + 180.0f) <= APP_YAW_HEADING_WRAP_EPS_DEG)
    {
        deg = 180.0f;
    }
    return deg;
}

static float app_yaw_heading_clampf(float x, float min_v, float max_v)    // clamp
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float app_yaw_heading_get_raw_yaw_deg(void)                     // 最新航向（USB ODOM 优先）
{
    if (rc_odom_is_valid() != 0U)
    {
        return rc_get_latest_odom()->yaw;
    }
    return g_sensor_task_data.imu.yaw_deg;
}

static float app_yaw_heading_get_norm_yaw_deg(void)                  // 获取归一化航向角
{
    const float raw_yaw_deg = app_yaw_heading_get_raw_yaw_deg();
    return app_yaw_heading_wrap_deg(raw_yaw_deg - g_app_yaw_heading_ctx.yaw_zero_deg);
}

static void app_yaw_heading_apply_vx_only(float vx_cmd)             // 应用 vx 指令（旋转通道）
{
    process_flow_chassis_override.axis_mask =
        (uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VX |
                  PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                  PROCESS_FLOW_CHASSIS_OVERRIDE_VW);
    process_flow_chassis_override.priority = PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH;
    process_flow_chassis_override.vx = vx_cmd;
    process_flow_chassis_override.vy = 0.0f;
    process_flow_chassis_override.vw = 0.0f;
}

static uint8_t app_yaw_heading_is_cmd_valid(AppYawHeadingCmd cmd)    // 命令有效性检查
{
    return (uint8_t)((cmd == app_yaw_heading_cmd_turn_left_90) ||
                     (cmd == app_yaw_heading_cmd_turn_right_90) ||
                     (cmd == app_yaw_heading_cmd_turn_180));
}

/* 按当前归一化 yaw 重建离散象限索引，避免索引靠历史累加发生漂移失配 */
static uint8_t app_yaw_heading_idx_from_norm_yaw(float norm_yaw_deg)
{
    const float y = app_yaw_heading_wrap_deg(norm_yaw_deg);

    if ((y >= 45.0f) && (y < 135.0f))
    {
        return 1U; /* 90 deg */
    }
    if ((y <= -45.0f) && (y > -135.0f))
    {
        return 3U; /* -90 deg */
    }
    if ((y >= 135.0f) || (y <= -135.0f))
    {
        return 2U; /* 180 deg */
    }
    return 0U; /* 0 deg */
}

static void app_yaw_heading_prepare_target_by_command(AppYawHeadingCmd cmd)   // 根据命令准备目标航向角
{
    static const float heading_table_deg[APP_YAW_HEADING_IDX_MAX] = {0.0f, 90.0f, 180.0f, -90.0f};
    uint8_t cur_idx = app_yaw_heading_idx_from_norm_yaw(app_yaw_heading_get_norm_yaw_deg());

    g_app_yaw_heading_ctx.heading_idx = cur_idx;
    if (cmd == app_yaw_heading_cmd_turn_left_90)
    {
        g_app_yaw_heading_ctx.heading_idx = (uint8_t)((cur_idx + 1U) % APP_YAW_HEADING_IDX_MAX);
    }
    else if (cmd == app_yaw_heading_cmd_turn_right_90)
    {
        g_app_yaw_heading_ctx.heading_idx =
            (uint8_t)((cur_idx + 3U) % APP_YAW_HEADING_IDX_MAX);
    }
    else
    {
        g_app_yaw_heading_ctx.heading_idx = (uint8_t)((cur_idx + 2U) % APP_YAW_HEADING_IDX_MAX);
    }

    g_app_yaw_heading_ctx.target_yaw_deg = heading_table_deg[g_app_yaw_heading_ctx.heading_idx];
    g_app_yaw_heading_ctx.enable = 1U;
}

void AppYawHeadingCtrl_Init(void)                                    // 初始化
{
    g_app_yaw_heading_ctx.inited = 1U;
    g_app_yaw_heading_ctx.enable = 0U;
    g_app_yaw_heading_ctx.heading_idx = 0U;
    g_app_yaw_heading_ctx.yaw_zero_deg = app_yaw_heading_get_raw_yaw_deg();
    g_app_yaw_heading_ctx.target_yaw_deg = 0.0f;
    g_app_yaw_heading_ctx.error_deg = 0.0f;
    g_app_yaw_heading_ctx.pending_cmd = app_yaw_heading_cmd_none;

    Process_Flow_ClearChassisOverride();
}

uint8_t AppYawHeadingCtrl_GetConfig(AppYawHeadingCtrlConfig *out)
{
    if (out == 0)
    {
        return 0U;
    }
    *out = g_app_yaw_heading_ctrl_cfg;
    return 1U;
}

uint8_t AppYawHeadingCtrl_SetConfig(const AppYawHeadingCtrlConfig *cfg)
{
    if (app_yaw_heading_cfg_is_valid(cfg) == 0U)
    {
        return 0U;
    }
    g_app_yaw_heading_ctrl_cfg = *cfg;
    return 1U;
}

uint8_t AppYawHeadingCtrl_PostCommand(AppYawHeadingCmd cmd)          // 提交命令
{
    if ((g_app_yaw_heading_ctx.inited == 0U) || (app_yaw_heading_is_cmd_valid(cmd) == 0U))
    {
        return 0U;
    }

    g_app_yaw_heading_ctx.pending_cmd = cmd;
    return 1U;
}

/*
 * 与 app_zone2 场地轴一致：红区 map +x 右、+y 上；红蓝场向语义相同（LEFT=+x、RIGHT=-x）。
 * FRONT=+y、BACK=-y；航向：FRONT 0°、BACK 180°（前为 0、逆时针为正）。
 * 红区 map：+x → yaw -90°（FIELD_LEFT），-x → yaw +90°（FIELD_RIGHT）。
 * field_dir_between_mf_cells：dy>0 FRONT，dy<0 BACK，dx>0 LEFT(+x)，dx<0 RIGHT(-x)。
 */
static float app_yaw_heading_field_dir_to_world_heading_deg(app_zone2_field_dir_t dir)
{
    switch (dir)
    {
        case APP_ZONE2_FIELD_FRONT:
            return 0.0f;
        case APP_ZONE2_FIELD_BACK:
            return 180.0f;
        case APP_ZONE2_FIELD_LEFT:
            return -90.0f;
        case APP_ZONE2_FIELD_RIGHT:
            return 90.0f;
        default:
            return 0.0f;
    }
}

void AppYawHeadingCtrl_RunFieldDir(app_zone2_field_dir_t dir)
{
    float world_heading_deg;

    if (g_app_yaw_heading_ctx.inited == 0U)
    {
        return;
    }

    g_app_yaw_heading_ctx.pending_cmd = app_yaw_heading_cmd_none;

    if (dir == APP_ZONE2_FIELD_FACE_SKIP)
    {
        g_app_yaw_heading_ctx.enable = 0U;
        Process_Flow_ClearChassisOverride();
        return;
    }

    world_heading_deg = app_yaw_heading_field_dir_to_world_heading_deg(dir);
    g_app_yaw_heading_ctx.target_yaw_deg =
        app_yaw_heading_wrap_deg(world_heading_deg - g_app_yaw_heading_ctx.yaw_zero_deg);
    g_app_yaw_heading_ctx.enable = 1U;
    /* 周期 PD 在 chassis.c manual_chassis_function 与 odom_nav_goto_run 一并调用 */
}

void AppYawHeadingCtrl_Run(void)                                    // 运行
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

    if (fabsf(g_app_yaw_heading_ctx.error_deg) < g_app_yaw_heading_ctrl_cfg.dead_zone_deg)
    {
        g_app_yaw_heading_ctx.enable = 0U;
        Process_Flow_ClearChassisOverride();
        return;
    }

    gyr_z_dps = g_sensor_task_data.imu.gyr_z_dps;
    spd_cmd = g_app_yaw_heading_ctrl_cfg.kp * g_app_yaw_heading_ctx.error_deg - g_app_yaw_heading_ctrl_cfg.kd * gyr_z_dps;
    spd_cmd = app_yaw_heading_clampf(spd_cmd, -g_app_yaw_heading_ctrl_cfg.max_speed, g_app_yaw_heading_ctrl_cfg.max_speed);

    app_yaw_heading_apply_vx_only(-spd_cmd); // 应用 vx 指令（旋转通道）
}

uint8_t AppYawHeadingCtrl_IsBusy(void)
{
    if (g_app_yaw_heading_ctx.inited == 0U)
    {
        return 0U;
    }

    return (uint8_t)((g_app_yaw_heading_ctx.enable != 0U) ||
                     (g_app_yaw_heading_ctx.pending_cmd != app_yaw_heading_cmd_none));
}
