#include "yaw_heading_ctrl.h"
#include "cmsis_os.h"

#include "Process_Flow.h"
#include "Sensor_Task.h"
#include "common.h"
#include "upper_pc_protocol.h"

#include <math.h>

#define YAW_HEADING_IDX_MAX             (4U)     // 航向索引最大值
#define YAW_HEADING_WRAP_EPS_DEG        (1e-3f) // 航向误差最小值
#define YAW_HEADING_CARDINAL_BOUND_DEG  (45.0f) // 航向索引边界值
#define YAW_HEADING_CARDINAL_BACK_DEG   (135.0f) // 航向索引反向边界值
#define YAW_HEADING_SLOW_ZONE_MIN_RATIO (0.15f) // 慢速区最低速比例

typedef struct
{
    uint8_t inited;             // 初始化标志
    uint8_t enable;             // 使能标志
    uint8_t heading_idx;        // 航向索引
    float yaw_zero_deg;         // 航向零点
    float target_yaw_deg;       // 目标航向
    float error_deg;            // 误差角度
    YawHeadingCmd pending_cmd;  // 待执行命令
    uint32_t dead_zone_entry_tick; // 死区进入时间戳
    float error_integral;       // 误差积分
    float gyro_rate_lpf;        // 陀螺仪滤波
    float rate_integral;        // 速度积分
    uint32_t last_run_tick;    // 上次运行时间戳
    float last_error_deg;       // 上一周期误差（过零清积分）
} YawHeadingCtrlCtx;

static const float s_yaw_heading_cardinal_deg[YAW_HEADING_IDX_MAX] = {
    0.0f, 90.0f, 180.0f, -90.0f,
}; // 航向索引对应的航向值

static volatile YawHeadingCtrlCtx g_yaw_heading_ctx;

volatile YawHeadingCtrlConfig g_yaw_heading_ctrl_cfg = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.85f,

    .max_speed = 45.0f,
    .max_rate_dps = 85.0f,
    .dead_zone_deg = 3.5f,
    .arrival_dwell_ms = 120U,
    .arrival_rate_thr_dps = 6.0f,

    .slow_zone_deg = 30.0f,
    .cardinal_hyst_deg = 20.0f,

    .gyro_lpf_alpha = 0.40f,
    .ki_active_thr_deg = 0.0f,

    .kp_outer = 2.6f,
    .kp_inner = 3.0f,
    .ki_inner = 0.0f,
    .i_inner_limit = 10.0f,
}; // 航向控制配置
volatile yaw_heading_dbg_t g_yaw_heading_dbg; // 航向控制调试信息

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
    if (!isfinite(cfg->gyro_lpf_alpha) || cfg->gyro_lpf_alpha < 0.0f || cfg->gyro_lpf_alpha > 1.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->ki_active_thr_deg) || cfg->ki_active_thr_deg < 0.0f || cfg->ki_active_thr_deg > 90.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->kp_outer) || cfg->kp_outer < 0.0f || cfg->kp_outer > 50.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->kp_inner) || cfg->kp_inner < 0.0f || cfg->kp_inner > 100.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->ki_inner) || cfg->ki_inner < 0.0f || cfg->ki_inner > 50.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->i_inner_limit) || cfg->i_inner_limit < 0.0f || cfg->i_inner_limit > 100.0f)
    {
        return 0U;
    }
    if (!isfinite(cfg->max_rate_dps) || cfg->max_rate_dps <= 0.0f || cfg->max_rate_dps > 500.0f)
    {
        return 0U;
    }
    return 1U;
}

/** |error| 进入 slow_zone 后按 smoothstep 缩小允许 vx / 目标角速度，抑制冲过头 */
static float yaw_heading_eff_limit(float error_deg_abs, float vmax)
{
    const float slow_zone = g_yaw_heading_ctrl_cfg.slow_zone_deg;

    if (slow_zone <= 1e-3f || error_deg_abs >= slow_zone)
    {
        return vmax;
    }

    {
        const float x = error_deg_abs / slow_zone;
        const float ratio = x * x * (3.0f - 2.0f * x);
        return vmax * fmaxf(YAW_HEADING_SLOW_ZONE_MIN_RATIO, ratio);
    }
}

static uint8_t yaw_heading_arrival_ready(float error_deg, float gyr_z_dps, uint32_t now_tick)
{
    const float dead_zone = g_yaw_heading_ctrl_cfg.dead_zone_deg; // 死区角度
    const float rate_thr = g_yaw_heading_ctrl_cfg.arrival_rate_thr_dps; // 到位角速度阈值
    const uint32_t dwell_ms = g_yaw_heading_ctrl_cfg.arrival_dwell_ms; // 到位驻留时间

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

static float yaw_heading_wrap_deg(float deg) // 航向wrap
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

static float yaw_heading_clampf(float x, float min_v, float max_v) // 限幅
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float yaw_heading_get_raw_yaw_deg(void) // 获取原始航向
{
    if (rc_odom_is_valid() != 0U)
    {
        return rc_get_latest_odom()->yaw;
    }
    return g_sensor_task_data.imu.yaw_deg;
}

static float yaw_heading_get_norm_yaw_deg(void) // 获取归一化航向
{
    const float raw_yaw_deg = yaw_heading_get_raw_yaw_deg();
    return yaw_heading_wrap_deg(raw_yaw_deg - g_yaw_heading_ctx.yaw_zero_deg);
}

static void yaw_heading_apply_vx_only(float vx_cmd) // 应用vx
{
    Process_Flow_SetChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX,
                                        PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH,
                                        vx_cmd, 0.0f, 0.0f);
}

static uint8_t yaw_heading_is_cmd_valid(YawHeadingCmd cmd) // 命令有效性检查
{
    return (uint8_t)((cmd == yaw_heading_cmd_turn_left_90) ||
                     (cmd == yaw_heading_cmd_turn_right_90) ||
                     (cmd == yaw_heading_cmd_turn_180));
}

static uint8_t yaw_heading_idx_from_norm_yaw_plain(float norm_yaw_deg) // 从归一化航向获取航向索引
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

static uint8_t yaw_heading_idx_from_cardinal_target(float target_yaw_deg) // 从目标航向获取航向索引
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

static void yaw_heading_cardinal_lock_update(float norm_yaw_deg) // 四向锁更新
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

    g_yaw_heading_ctx.heading_idx = idx;    // 更新航向索引
}

static void yaw_heading_set_cardinal_target_idx(uint8_t idx) // 设置四向目标航向索引
{
    if (idx >= YAW_HEADING_IDX_MAX)
    {
        idx = 0U;
    }
    g_yaw_heading_ctx.heading_idx = idx;
    g_yaw_heading_ctx.target_yaw_deg = s_yaw_heading_cardinal_deg[idx];
}

static void yaw_heading_prepare_target_by_command(YawHeadingCmd cmd) // 准备目标航向
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
    g_yaw_heading_ctx.rate_integral = 0.0f;
    g_yaw_heading_ctx.last_run_tick = 0U;
    g_yaw_heading_ctx.last_error_deg = 0.0f;
    g_yaw_heading_ctx.enable = 1U;
}

static void yaw_heading_dbg_refresh(float norm_yaw_deg, float gyr_z_dps, float spd_cmd, uint8_t active) // 调试信息刷新
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

void YawHeadingCtrl_Init(void) // 初始化
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
    g_yaw_heading_ctx.gyro_rate_lpf = 0.0f;
    g_yaw_heading_ctx.rate_integral = 0.0f;
    g_yaw_heading_ctx.last_run_tick = 0U;
    g_yaw_heading_ctx.last_error_deg = 0.0f;

    Process_Flow_ClearChassisOverride();
    yaw_heading_dbg_refresh(norm_yaw_deg, 0.0f, 0.0f, 0U);
}

uint8_t YawHeadingCtrl_GetConfig(YawHeadingCtrlConfig *out) // 获取配置
{
    if (out == 0)
    {
        return 0U;
    }
    *out = g_yaw_heading_ctrl_cfg;
    return 1U;
}

uint8_t YawHeadingCtrl_SetConfig(const YawHeadingCtrlConfig *cfg) // 设置配置
{
    if (yaw_heading_cfg_is_valid(cfg) == 0U)
    {
        return 0U;
    }
    g_yaw_heading_ctrl_cfg = *cfg;
    return 1U;
}

uint8_t YawHeadingCtrl_PostCommand(YawHeadingCmd cmd) // 提交命令
{
    if ((g_yaw_heading_ctx.inited == 0U) || (yaw_heading_is_cmd_valid(cmd) == 0U))
    {
        return 0U;
    }

    g_yaw_heading_ctx.pending_cmd = cmd;
    return 1U;
}

static float yaw_heading_field_dir_to_world_heading_deg(app_zone2_field_dir_t dir) // 场地方向到世界航向
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

void YawHeadingCtrl_RunFieldDir(app_zone2_field_dir_t dir) // 运行场地方向
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
    g_yaw_heading_ctx.rate_integral = 0.0f;
    g_yaw_heading_ctx.last_run_tick = 0U;
    g_yaw_heading_ctx.last_error_deg = 0.0f;
    g_yaw_heading_ctx.enable = 1U;
}

void YawHeadingCtrl_Run(void) // 运行
{
    float norm_yaw_deg; // 归一化航向
    float gyr_z_dps; // 陀螺仪角速度
    float spd_cmd; // 速度命令
    uint8_t active; // 使能标志

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
        g_yaw_heading_ctx.last_error_deg = 0.0f;
    }

    norm_yaw_deg = yaw_heading_get_norm_yaw_deg();
    gyr_z_dps    = g_sensor_task_data.imu.gyr_z_dps;
    active       = g_yaw_heading_ctx.enable;

    if (active == 0U)
    {
        yaw_heading_cardinal_lock_update(norm_yaw_deg);
    }

    /* error = target - meas，与 ChassisHeadingHold 一致，wrap 后取最短路径 */
    g_yaw_heading_ctx.error_deg =
        yaw_heading_wrap_deg(g_yaw_heading_ctx.target_yaw_deg - norm_yaw_deg);

    /* ---- gyro LPF: always runs (filtered value used by arrival + PID) ---- */
    {
        const float err_abs = fabsf(g_yaw_heading_ctx.error_deg);
        float alpha = g_yaw_heading_ctrl_cfg.gyro_lpf_alpha;
        if (active != 0U)
        {
            if (err_abs > 10.0f)
            {
                alpha *= 2.0f;
            }
            else if (err_abs < 3.0f)
            {
                alpha *= 0.5f;
            }
        }
        else
        {
            alpha *= 0.5f;
        }
        alpha = yaw_heading_clampf(alpha, 0.02f, 0.5f);
        g_yaw_heading_ctx.gyro_rate_lpf =
            alpha * gyr_z_dps + (1.0f - alpha) * g_yaw_heading_ctx.gyro_rate_lpf;
    }
    const float gyro_lpf = g_yaw_heading_ctx.gyro_rate_lpf;

    /* ---- arrival detection (uses filtered gyro, not raw) ---- */
    if (active != 0U)
    {
        const uint32_t now_tick = osKernelGetTickCount();

        if (yaw_heading_arrival_ready(g_yaw_heading_ctx.error_deg, gyro_lpf, now_tick) != 0U)
        {
            g_yaw_heading_ctx.heading_idx =
                yaw_heading_idx_from_cardinal_target(g_yaw_heading_ctx.target_yaw_deg);
            g_yaw_heading_ctx.enable = 0U;
            Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VX);
            active = 0U;
            g_yaw_heading_ctx.dead_zone_entry_tick = 0U;
            g_yaw_heading_ctx.error_integral = 0.0f;
            g_yaw_heading_ctx.rate_integral = 0.0f;
            g_yaw_heading_ctx.last_run_tick = 0U;
            g_yaw_heading_ctx.last_error_deg = 0.0f;
        }
        else if (fabsf(g_yaw_heading_ctx.error_deg) >= g_yaw_heading_ctrl_cfg.dead_zone_deg)
        {
            g_yaw_heading_ctx.dead_zone_entry_tick = 0U;
        }
    }

    /* ---- PID block ---- */
    if (active != 0U)
    {
        float dt = 0.005f;
        {
            const uint32_t run_tick = osKernelGetTickCount();
            if (g_yaw_heading_ctx.last_run_tick != 0U)
            {
                dt = (float)(run_tick - g_yaw_heading_ctx.last_run_tick) * 0.001f;
                if (dt <= 0.0f || dt > 0.1f) { dt = 0.005f; }
            }
            g_yaw_heading_ctx.last_run_tick = run_tick;
        }

        const float err = g_yaw_heading_ctx.error_deg;
        const float err_abs = fabsf(err);
        const float dead_zone = g_yaw_heading_ctrl_cfg.dead_zone_deg;

        /* hard stop: within dead zone + low rate -> output 0, leak integral */
        if (err_abs < dead_zone && fabsf(gyro_lpf) < g_yaw_heading_ctrl_cfg.arrival_rate_thr_dps)
        {
            spd_cmd = 0.0f;
            g_yaw_heading_ctx.rate_integral *= 0.5f;
            yaw_heading_apply_vx_only(0.0f);
        }
        else
        {
            float vx_cmd;
            const float eff_max_vx = yaw_heading_eff_limit(err_abs, g_yaw_heading_ctrl_cfg.max_speed);
            const float eff_max_rate = yaw_heading_eff_limit(err_abs, g_yaw_heading_ctrl_cfg.max_rate_dps);
            const float slow_zone = g_yaw_heading_ctrl_cfg.slow_zone_deg;

            /* 误差过零：清内环积分，避免 ±60 反向饱和 */
            if ((g_yaw_heading_ctx.last_error_deg * err < 0.0f) &&
                (fabsf(g_yaw_heading_ctx.last_error_deg) > 0.5f))
            {
                g_yaw_heading_ctx.rate_integral = 0.0f;
            }
            g_yaw_heading_ctx.last_error_deg = err;

            if (err_abs < slow_zone)
            {
                /* 近区：角度 PD（与 ChassisHeadingHold 同形），不再跑串级内环 */
                g_yaw_heading_ctx.rate_integral *= 0.85f;
                spd_cmd = g_yaw_heading_ctrl_cfg.kp_outer * err
                        - g_yaw_heading_ctrl_cfg.kd * gyro_lpf;
                spd_cmd = yaw_heading_clampf(spd_cmd, -eff_max_vx, eff_max_vx);
            }
            else
            {
                float target_rate = g_yaw_heading_ctrl_cfg.kp_outer * err;
                float spd_raw;

                target_rate = yaw_heading_clampf(target_rate, -eff_max_rate, eff_max_rate);

                {
                    const float rate_err = target_rate - gyro_lpf;

                    g_yaw_heading_ctx.rate_integral += rate_err * dt;
                    g_yaw_heading_ctx.rate_integral = yaw_heading_clampf(
                        g_yaw_heading_ctx.rate_integral,
                        -g_yaw_heading_ctrl_cfg.i_inner_limit,
                         g_yaw_heading_ctrl_cfg.i_inner_limit);

                    spd_raw = g_yaw_heading_ctrl_cfg.kp_inner * rate_err
                            + g_yaw_heading_ctrl_cfg.ki_inner * g_yaw_heading_ctx.rate_integral;
                    spd_cmd = yaw_heading_clampf(spd_raw, -eff_max_vx, eff_max_vx);

                    if (fabsf(spd_raw) > eff_max_vx + 1e-3f)
                    {
                        g_yaw_heading_ctx.rate_integral *= 0.7f;
                    }
                }
            }

            vx_cmd = -spd_cmd;
            yaw_heading_apply_vx_only(vx_cmd);
        }
    }
    else
    {
        spd_cmd = 0.0f;
    }
    {
        const float vx_applied = (active != 0U) ? -spd_cmd : 0.0f;
        yaw_heading_dbg_refresh(norm_yaw_deg, gyr_z_dps, vx_applied, active);
    }

    {
        static uint32_t last_dbg_ms = 0U;
        uint32_t now_ms = common_now_ms();
        if (now_ms - last_dbg_ms >= 20U)
        {
            last_dbg_ms = now_ms;
            rc_debug_heading_hold_t dbg;
            const float vx_applied = (active != 0U) ? -spd_cmd : 0.0f;
            dbg.yaw_ref_deg  = g_yaw_heading_ctx.target_yaw_deg;
            dbg.yaw_deg      = norm_yaw_deg;
            dbg.err_deg      = g_yaw_heading_ctx.error_deg;
            dbg.i_term       = g_yaw_heading_ctx.rate_integral;
            dbg.output       = vx_applied;
            dbg.yaw_rate_dps = gyro_lpf;
            rc_send_debug_heading_hold(&dbg);
        }
    }
}

uint8_t YawHeadingCtrl_IsBusy(void) // 是否繁忙
{
    if (g_yaw_heading_ctx.inited == 0U)
    {
        return 0U;
    }

    return (uint8_t)((g_yaw_heading_ctx.enable != 0U) ||
                     (g_yaw_heading_ctx.pending_cmd != yaw_heading_cmd_none));
}
