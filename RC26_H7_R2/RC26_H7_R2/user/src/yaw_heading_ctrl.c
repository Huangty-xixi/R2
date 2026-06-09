#include "yaw_heading_ctrl.h"
#include "cmsis_os.h"

#include "Process_Flow.h"
#include "Sensor_Task.h"
#include "common.h"
#include "upper_pc_protocol.h"

#include <math.h>

#define YAW_HEADING_IDX_MAX             (4U)
#define YAW_HEADING_WRAP_EPS_DEG        (1e-3f)
#define YAW_HEADING_SLOW_ZONE_MIN_RATIO (0.35f)
#define YAW_HEADING_CARDINAL_BOUND_DEG  (45.0f)
#define YAW_HEADING_CARDINAL_BACK_DEG   (135.0f)

typedef struct
{
    uint8_t inited;
    uint8_t enable;
    uint8_t heading_idx;
    float yaw_zero_deg;
    float target_yaw_deg;
    float error_deg;
    YawHeadingCmd pending_cmd;
    uint32_t dead_zone_entry_tick;
    float error_integral;
} YawHeadingCtrlCtx;

static const float s_yaw_heading_cardinal_deg[YAW_HEADING_IDX_MAX] = {
    0.0f, 90.0f, 180.0f, -90.0f,
};

static volatile YawHeadingCtrlCtx g_yaw_heading_ctx;

volatile YawHeadingCtrlConfig g_yaw_heading_ctrl_cfg = {
    .kp = 4.8f,
    .ki = 0.0f,
    .kd = 0.42f,
    .max_speed = 40.0f,
    .dead_zone_deg = 1.0f,
    .arrival_dwell_ms = 120U,
    .arrival_rate_thr_dps = 6.0f,
    .slow_zone_deg = 22.0f,
    .cardinal_hyst_deg = 20.0f,
};

volatile yaw_heading_dbg_t g_yaw_heading_dbg;

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
    if (!isfinite(cfg->ki) || cfg->ki < 0.0f)
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
    if (cfg->arrival_dwell_ms == 0U)
    {
        return 0U;
    }
    if (!isfinite(cfg->arrival_rate_thr_dps) || cfg->arrival_rate_thr_dps < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->slow_zone_deg) || cfg->slow_zone_deg < 0.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->cardinal_hyst_deg) || cfg->cardinal_hyst_deg < 0.0f ||
        cfg->cardinal_hyst_deg > 20.0f)
    {
        return 0U;
    }
    return 1U;
}

static float yaw_heading_eff_max_speed(float error_deg_abs)
{
    const float vmax = g_yaw_heading_ctrl_cfg.max_speed;
    const float slow_zone = g_yaw_heading_ctrl_cfg.slow_zone_deg;

    if (slow_zone <= 1e-3f)
    {
        return vmax;
    }

    {
        const float ratio = fmaxf(YAW_HEADING_SLOW_ZONE_MIN_RATIO,
                                  fminf(1.0f, error_deg_abs / slow_zone));
        return vmax * ratio;
    }
}

static uint8_t yaw_heading_arrival_ready(float error_deg, float gyr_z_dps, uint32_t now_tick)
{
    const float dead_zone = g_yaw_heading_ctrl_cfg.dead_zone_deg;
    const float rate_thr = g_yaw_heading_ctrl_cfg.arrival_rate_thr_dps;
    const uint32_t dwell_ms = g_yaw_heading_ctrl_cfg.arrival_dwell_ms;

    if (fabsf(error_deg) >= dead_zone)
    {
        return 0U;
    }
    if (fabsf(gyr_z_dps) > rate_thr)
    {
        g_yaw_heading_ctx.dead_zone_entry_tick = 0U;
        return 0U;
    }
    if (g_yaw_heading_ctx.dead_zone_entry_tick == 0U)
    {
        g_yaw_heading_ctx.dead_zone_entry_tick = now_tick;
        return 0U;
    }
    if ((now_tick - g_yaw_heading_ctx.dead_zone_entry_tick) < dwell_ms)
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

static uint8_t yaw_heading_idx_from_norm_yaw_plain(float norm_yaw_deg)
{
    const float y = yaw_heading_wrap_deg(norm_yaw_deg);

    if ((y >= YAW_HEADING_CARDINAL_BOUND_DEG) && (y < YAW_HEADING_CARDINAL_BACK_DEG))
    {
        return 1U;
    }
    if ((y <= -YAW_HEADING_CARDINAL_BOUND_DEG) && (y > -YAW_HEADING_CARDINAL_BACK_DEG))
    {
        return 3U;
    }
    if ((y >= YAW_HEADING_CARDINAL_BACK_DEG) || (y <= -YAW_HEADING_CARDINAL_BACK_DEG))
    {
        return 2U;
    }
    return 0U;
}

static uint8_t yaw_heading_idx_from_cardinal_target(float target_yaw_deg)
{
    const float t = yaw_heading_wrap_deg(target_yaw_deg);

    if (fabsf(t - 90.0f) < 1.0f)
    {
        return 1U;
    }
    if (fabsf(t + 90.0f) < 1.0f)
    {
        return 3U;
    }
    if (fabsf(fabsf(t) - 180.0f) < 1.0f)
    {
        return 2U;
    }
    return 0U;
}

static void yaw_heading_cardinal_lock_update(float norm_yaw_deg)
{
    const float h = g_yaw_heading_ctrl_cfg.cardinal_hyst_deg;
    const float y = yaw_heading_wrap_deg(norm_yaw_deg);
    uint8_t idx = g_yaw_heading_ctx.heading_idx;

    if (idx >= YAW_HEADING_IDX_MAX)
    {
        idx = yaw_heading_idx_from_norm_yaw_plain(y);
        g_yaw_heading_ctx.heading_idx = idx;
        return;
    }

    switch (idx)
    {
        case 0U:
            if (y > (YAW_HEADING_CARDINAL_BOUND_DEG + h))
            {
                idx = 1U;
            }
            else if (y < (-YAW_HEADING_CARDINAL_BOUND_DEG - h))
            {
                idx = 3U;
            }
            break;

        case 1U:
            if (y < (YAW_HEADING_CARDINAL_BOUND_DEG - h))
            {
                idx = 0U;
            }
            else if (y > (YAW_HEADING_CARDINAL_BACK_DEG + h))
            {
                idx = 2U;
            }
            break;

        case 2U:
            if (y >= 0.0f)
            {
                if (y < (YAW_HEADING_CARDINAL_BACK_DEG - h))
                {
                    idx = 1U;
                }
            }
            else if (y > (-YAW_HEADING_CARDINAL_BACK_DEG + h))
            {
                idx = 3U;
            }
            break;

        case 3U:
            if (y > (-YAW_HEADING_CARDINAL_BOUND_DEG + h))
            {
                idx = 0U;
            }
            else if (y < (-YAW_HEADING_CARDINAL_BACK_DEG - h))
            {
                idx = 2U;
            }
            break;

        default:
            idx = yaw_heading_idx_from_norm_yaw_plain(y);
            break;
    }

    g_yaw_heading_ctx.heading_idx = idx;
}

static void yaw_heading_set_cardinal_target_idx(uint8_t idx)
{
    if (idx >= YAW_HEADING_IDX_MAX)
    {
        idx = 0U;
    }
    g_yaw_heading_ctx.heading_idx = idx;
    g_yaw_heading_ctx.target_yaw_deg = s_yaw_heading_cardinal_deg[idx];
}

static void yaw_heading_prepare_target_by_command(YawHeadingCmd cmd)
{
    uint8_t idx;

    yaw_heading_cardinal_lock_update(yaw_heading_get_norm_yaw_deg());
    idx = g_yaw_heading_ctx.heading_idx;

    if (cmd == yaw_heading_cmd_turn_left_90)
    {
        idx = (uint8_t)((idx + 1U) % YAW_HEADING_IDX_MAX);
    }
    else if (cmd == yaw_heading_cmd_turn_right_90)
    {
        idx = (uint8_t)((idx + 3U) % YAW_HEADING_IDX_MAX);
    }
    else
    {
        idx = (uint8_t)((idx + 2U) % YAW_HEADING_IDX_MAX);
    }

    yaw_heading_set_cardinal_target_idx(idx);
    g_yaw_heading_ctx.enable = 1U;
}

static void yaw_heading_dbg_refresh(float norm_yaw_deg, float gyr_z_dps, float spd_cmd, uint8_t active)
{
    uint32_t dwell_ms = 0U;
    uint32_t now_tick = osKernelGetTickCount();

    if ((active != 0U) && (g_yaw_heading_ctx.dead_zone_entry_tick != 0U) &&
        (now_tick >= g_yaw_heading_ctx.dead_zone_entry_tick))
    {
        dwell_ms = now_tick - g_yaw_heading_ctx.dead_zone_entry_tick;
    }

    g_yaw_heading_dbg.enable = g_yaw_heading_ctx.enable;
    g_yaw_heading_dbg.heading_idx = g_yaw_heading_ctx.heading_idx;
    g_yaw_heading_dbg.norm_yaw_deg = norm_yaw_deg;
    g_yaw_heading_dbg.target_yaw_deg = g_yaw_heading_ctx.target_yaw_deg;
    g_yaw_heading_dbg.error_deg = g_yaw_heading_ctx.error_deg;
    g_yaw_heading_dbg.gyr_z_dps = gyr_z_dps;
    g_yaw_heading_dbg.spd_cmd = spd_cmd;
    g_yaw_heading_dbg.dead_zone_dwell_ms = dwell_ms;
}

void YawHeadingCtrl_Init(void)
{
    const float norm_yaw_deg = yaw_heading_get_norm_yaw_deg();

    g_yaw_heading_ctx.inited = 1U;
    g_yaw_heading_ctx.enable = 0U;
    g_yaw_heading_ctx.heading_idx = yaw_heading_idx_from_norm_yaw_plain(norm_yaw_deg);
    g_yaw_heading_ctx.yaw_zero_deg = yaw_heading_get_raw_yaw_deg();
    g_yaw_heading_ctx.target_yaw_deg = 0.0f;
    g_yaw_heading_ctx.error_deg = 0.0f;
    g_yaw_heading_ctx.pending_cmd = yaw_heading_cmd_none;
    g_yaw_heading_ctx.dead_zone_entry_tick = 0U;
    g_yaw_heading_ctx.error_integral = 0.0f;

    Process_Flow_ClearChassisOverride();
    yaw_heading_dbg_refresh(norm_yaw_deg, 0.0f, 0.0f, 0U);
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
    float target_yaw_deg;

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
    target_yaw_deg = yaw_heading_wrap_deg(world_heading_deg - g_yaw_heading_ctx.yaw_zero_deg);
    g_yaw_heading_ctx.target_yaw_deg = target_yaw_deg;
    g_yaw_heading_ctx.heading_idx = yaw_heading_idx_from_cardinal_target(target_yaw_deg);
    g_yaw_heading_ctx.dead_zone_entry_tick = 0U;
    g_yaw_heading_ctx.error_integral = 0.0f;
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
        g_yaw_heading_ctx.dead_zone_entry_tick = 0U;
        g_yaw_heading_ctx.error_integral = 0.0f;
    }

    norm_yaw_deg = yaw_heading_get_norm_yaw_deg();
    gyr_z_dps    = g_sensor_task_data.imu.gyr_z_dps;
    active       = g_yaw_heading_ctx.enable;

    if (active == 0U)
    {
        yaw_heading_cardinal_lock_update(norm_yaw_deg);
    }

    g_yaw_heading_ctx.error_deg =
        yaw_heading_wrap_deg(g_yaw_heading_ctx.target_yaw_deg - norm_yaw_deg);

    if (active != 0U)
    {
        const uint32_t now_tick = osKernelGetTickCount();

        if (yaw_heading_arrival_ready(g_yaw_heading_ctx.error_deg, gyr_z_dps, now_tick) != 0U)
        {
            g_yaw_heading_ctx.heading_idx =
                yaw_heading_idx_from_cardinal_target(g_yaw_heading_ctx.target_yaw_deg);
            g_yaw_heading_ctx.enable = 0U;
            Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
            active = 0U;
            g_yaw_heading_ctx.dead_zone_entry_tick = 0U;
            g_yaw_heading_ctx.error_integral = 0.0f;
        }
        else if (fabsf(g_yaw_heading_ctx.error_deg) >= g_yaw_heading_ctrl_cfg.dead_zone_deg)
        {
            g_yaw_heading_ctx.dead_zone_entry_tick = 0U;
        }
    }

    if (active != 0U)
    {
        const float eff_max = yaw_heading_eff_max_speed(fabsf(g_yaw_heading_ctx.error_deg));

        g_yaw_heading_ctx.error_integral += g_yaw_heading_ctx.error_deg;
        {
            float max_int = eff_max / fmaxf(g_yaw_heading_ctrl_cfg.ki, 1e-6f);
            g_yaw_heading_ctx.error_integral = yaw_heading_clampf(g_yaw_heading_ctx.error_integral, -max_int, max_int);
        }
        spd_cmd = g_yaw_heading_ctrl_cfg.kp * g_yaw_heading_ctx.error_deg
                  + g_yaw_heading_ctrl_cfg.ki * g_yaw_heading_ctx.error_integral
                  - g_yaw_heading_ctrl_cfg.kd * gyr_z_dps;
        spd_cmd = yaw_heading_clampf(spd_cmd, -eff_max, eff_max);
        yaw_heading_apply_vx_only(-spd_cmd);
    }
    else
    {
        spd_cmd = 0.0f;
    }

    yaw_heading_dbg_refresh(norm_yaw_deg, gyr_z_dps, spd_cmd, active);

    {
        static uint32_t last_dbg_ms = 0U;
        uint32_t now_ms = common_now_ms();
        if (now_ms - last_dbg_ms >= 20U)
        {
            last_dbg_ms = now_ms;
            rc_debug_heading_hold_t dbg;
            dbg.yaw_ref_deg  = g_yaw_heading_ctx.target_yaw_deg;
            dbg.yaw_deg      = norm_yaw_deg;
            dbg.err_deg      = g_yaw_heading_ctx.error_deg;
            dbg.i_term       = g_yaw_heading_ctx.error_integral;
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
