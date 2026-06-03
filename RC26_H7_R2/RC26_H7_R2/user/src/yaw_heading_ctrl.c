#include "yaw_heading_ctrl.h"

#include "Process_Flow.h"
#include "Sensor_Task.h"
#include "common.h"
#include "upper_pc_protocol.h"

#include <math.h>

#define YAW_HEADING_IDX_MAX            (4U)
#define YAW_HEADING_WRAP_EPS_DEG       (1e-3f)

typedef struct
{
    uint8_t inited;
    uint8_t enable;
    uint8_t heading_idx;
    float yaw_zero_deg;
    float target_yaw_deg;
    float error_deg;
    YawHeadingCmd pending_cmd;
} YawHeadingCtrlCtx;

static volatile YawHeadingCtrlCtx g_yaw_heading_ctx;

volatile YawHeadingCtrlConfig g_yaw_heading_ctrl_cfg = {
    .kp = 3.0f,
    .kd = 0.20f,
    .max_speed = 20.0f,
    .dead_zone_deg = 1.5f,
};

static uint8_t yaw_heading_cfg_is_valid(const YawHeadingCtrlConfig *cfg)
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

static float yaw_heading_wrap_deg(float deg)
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

    if (fabsf(deg + 180.0f) <= YAW_HEADING_WRAP_EPS_DEG)
    {
        deg = 180.0f;
    }
    return deg;
}

static float yaw_heading_clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float yaw_heading_get_raw_yaw_deg(void)
{
    if (rc_odom_is_valid() != 0U)
    {
        return rc_get_latest_odom()->yaw;
    }
    return g_sensor_task_data.imu.yaw_deg;
}

static float yaw_heading_get_norm_yaw_deg(void)
{
    const float raw_yaw_deg = yaw_heading_get_raw_yaw_deg();
    return yaw_heading_wrap_deg(raw_yaw_deg - g_yaw_heading_ctx.yaw_zero_deg);
}

static void yaw_heading_apply_vx_only(float vx_cmd)
{
    Process_Flow_SetChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX,
                                        PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH,
                                        vx_cmd, 0.0f, 0.0f);
}

static uint8_t yaw_heading_is_cmd_valid(YawHeadingCmd cmd)
{
    return (uint8_t)((cmd == yaw_heading_cmd_turn_left_90) ||
                     (cmd == yaw_heading_cmd_turn_right_90) ||
                     (cmd == yaw_heading_cmd_turn_180));
}

static uint8_t yaw_heading_idx_from_norm_yaw(float norm_yaw_deg)
{
    const float y = yaw_heading_wrap_deg(norm_yaw_deg);

    if ((y >= 45.0f) && (y < 135.0f))
    {
        return 1U;
    }
    if ((y <= -45.0f) && (y > -135.0f))
    {
        return 3U;
    }
    if ((y >= 135.0f) || (y <= -135.0f))
    {
        return 2U;
    }
    return 0U;
}

static void yaw_heading_prepare_target_by_command(YawHeadingCmd cmd)
{
    static const float heading_table_deg[YAW_HEADING_IDX_MAX] = {0.0f, 90.0f, 180.0f, -90.0f};
    uint8_t cur_idx = yaw_heading_idx_from_norm_yaw(yaw_heading_get_norm_yaw_deg());

    g_yaw_heading_ctx.heading_idx = cur_idx;
    if (cmd == yaw_heading_cmd_turn_left_90)
    {
        g_yaw_heading_ctx.heading_idx = (uint8_t)((cur_idx + 1U) % YAW_HEADING_IDX_MAX);
    }
    else if (cmd == yaw_heading_cmd_turn_right_90)
    {
        g_yaw_heading_ctx.heading_idx =
            (uint8_t)((cur_idx + 3U) % YAW_HEADING_IDX_MAX);
    }
    else
    {
        g_yaw_heading_ctx.heading_idx = (uint8_t)((cur_idx + 2U) % YAW_HEADING_IDX_MAX);
    }

    g_yaw_heading_ctx.target_yaw_deg = heading_table_deg[g_yaw_heading_ctx.heading_idx];
    g_yaw_heading_ctx.enable = 1U;
}

void YawHeadingCtrl_Init(void)
{
    g_yaw_heading_ctx.inited = 1U;
    g_yaw_heading_ctx.enable = 0U;
    g_yaw_heading_ctx.heading_idx = 0U;
    g_yaw_heading_ctx.yaw_zero_deg = yaw_heading_get_raw_yaw_deg();
    g_yaw_heading_ctx.target_yaw_deg = 0.0f;
    g_yaw_heading_ctx.error_deg = 0.0f;
    g_yaw_heading_ctx.pending_cmd = yaw_heading_cmd_none;

    Process_Flow_ClearChassisOverride();
}

uint8_t YawHeadingCtrl_GetConfig(YawHeadingCtrlConfig *out)
{
    if (out == 0)
    {
        return 0U;
    }
    *out = g_yaw_heading_ctrl_cfg;
    return 1U;
}

uint8_t YawHeadingCtrl_SetConfig(const YawHeadingCtrlConfig *cfg)
{
    if (yaw_heading_cfg_is_valid(cfg) == 0U)
    {
        return 0U;
    }
    g_yaw_heading_ctrl_cfg = *cfg;
    return 1U;
}

uint8_t YawHeadingCtrl_PostCommand(YawHeadingCmd cmd)
{
    if ((g_yaw_heading_ctx.inited == 0U) || (yaw_heading_is_cmd_valid(cmd) == 0U))
    {
        return 0U;
    }

    g_yaw_heading_ctx.pending_cmd = cmd;
    return 1U;
}

static float yaw_heading_field_dir_to_world_heading_deg(app_zone2_field_dir_t dir)
{
    switch (dir)
    {
        case APP_ZONE2_FIELD_FRONT:
            return 0.0f;
        case APP_ZONE2_FIELD_BACK:
            return 180.0f;
        case APP_ZONE2_FIELD_LEFT:
            return 90.0f;
        case APP_ZONE2_FIELD_RIGHT:
            return -90.0f;
        default:
            return 0.0f;
    }
}

void YawHeadingCtrl_RunFieldDir(app_zone2_field_dir_t dir)
{
    float world_heading_deg;

    if (g_yaw_heading_ctx.inited == 0U)
    {
        return;
    }

    g_yaw_heading_ctx.pending_cmd = yaw_heading_cmd_none;

    if (dir == APP_ZONE2_FIELD_FACE_SKIP)
    {
        g_yaw_heading_ctx.enable = 0U;
        Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
        return;
    }

    world_heading_deg = yaw_heading_field_dir_to_world_heading_deg(dir);
    g_yaw_heading_ctx.target_yaw_deg =
        yaw_heading_wrap_deg(world_heading_deg - g_yaw_heading_ctx.yaw_zero_deg);
    g_yaw_heading_ctx.enable = 1U;
}

void YawHeadingCtrl_Run(void)
{
    float norm_yaw_deg;
    float gyr_z_dps;
    float spd_cmd;
    uint8_t active;

    if (g_yaw_heading_ctx.inited == 0U)
    {
        return;
    }

    if (g_yaw_heading_ctx.pending_cmd != yaw_heading_cmd_none)
    {
        yaw_heading_prepare_target_by_command(g_yaw_heading_ctx.pending_cmd);
        g_yaw_heading_ctx.pending_cmd = yaw_heading_cmd_none;
    }

    /* 提前读取当前状态，供调试发送使用 */
    norm_yaw_deg = yaw_heading_get_norm_yaw_deg();
    gyr_z_dps    = g_sensor_task_data.imu.gyr_z_dps;
    active       = g_yaw_heading_ctx.enable;

    if (active != 0U)
    {
        g_yaw_heading_ctx.error_deg =
            yaw_heading_wrap_deg(g_yaw_heading_ctx.target_yaw_deg - norm_yaw_deg);

        if (fabsf(g_yaw_heading_ctx.error_deg) < g_yaw_heading_ctrl_cfg.dead_zone_deg)
        {
            g_yaw_heading_ctx.enable = 0U;
            Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
            active = 0U;
        }
    }

    if (active != 0U)
    {
        spd_cmd = g_yaw_heading_ctrl_cfg.kp * g_yaw_heading_ctx.error_deg
                  - g_yaw_heading_ctrl_cfg.kd * gyr_z_dps;
        spd_cmd = yaw_heading_clampf(spd_cmd, -g_yaw_heading_ctrl_cfg.max_speed,
                                     g_yaw_heading_ctrl_cfg.max_speed);
        yaw_heading_apply_vx_only(-spd_cmd);
    }
    else
    {
        spd_cmd = 0.0f;
    }

    /* 常发调试数据到上位机 (50Hz)，空闲时发零值表示在线 */
    {
        static uint32_t last_dbg_ms = 0U;
        uint32_t now_ms = common_now_ms();
        if (now_ms - last_dbg_ms >= 20U)
        {
            last_dbg_ms = now_ms;
            rc_debug_heading_hold_t dbg;
            dbg.yaw_ref_deg  = g_yaw_heading_ctx.target_yaw_deg;
            dbg.yaw_deg      = norm_yaw_deg;
            dbg.err_deg      = (active != 0U) ? g_yaw_heading_ctx.error_deg : 0.0f;
            dbg.i_term       = 0.0f;
            dbg.output       = spd_cmd;
            dbg.yaw_rate_dps = gyr_z_dps;
            rc_send_debug_heading_hold(&dbg);
        }
    }
}

uint8_t YawHeadingCtrl_IsBusy(void)
{
    if (g_yaw_heading_ctx.inited == 0U)
    {
        return 0U;
    }

    return (uint8_t)((g_yaw_heading_ctx.enable != 0U) ||
                     (g_yaw_heading_ctx.pending_cmd != yaw_heading_cmd_none));
}
