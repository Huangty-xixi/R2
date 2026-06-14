#include "camera_correct.h"
#include "Process_Flow.h"
#include "cmsis_os.h"
#include <math.h>
#include "upper_pc_protocol.h"

volatile CameraCorrectCfg g_camera_correct_cfg = {
    .kp = 0.5f,
    .ki = 0.02f,
    .kd = 0.05f,
    .i_max = 0.3f,
    .out_max = 0.5f,
    .dead_zone = 0.02f,       /* 2cm死区 */
    .stable_need = 3U,         /* 连续3帧 */
    .timeout_ms = 3000U,       /* 3秒总超时 */
    .data_timeout_ms = 200U,   /* 200ms无新帧视为invalid */
};

static float s_last_error;
static float s_integral;
static uint8_t s_stable_count;
static uint32_t s_start_ms;
static uint32_t s_last_frame_ms;
static float s_last_vw;

void CameraCorrect_Reset(void)
{
    s_last_error = 0.0f;
    s_integral = 0.0f;
    s_stable_count = 0U;
    s_start_ms = osKernelGetTickCount();
    s_last_frame_ms = s_start_ms;
    s_last_vw = 0.0f;
}

float CameraCorrect_Update(float error_m)
{
    uint32_t now_ms;
    float dt;
    float p_term, i_term, d_term;
    float vw;

    now_ms = osKernelGetTickCount();
    s_last_frame_ms = now_ms;  /* 每次调用更新，用于IsTimeout判数据超时 */

    /* 死区检查: 连续稳定帧计数 */
    if (fabsf(error_m) < g_camera_correct_cfg.dead_zone)
    {
        s_stable_count++;
    }
    else
    {
        s_stable_count = 0U;
    }

    /* 用固定dt=33ms，摄像头30fps */
    dt = 0.033f;

    /* PID计算 */
    p_term = g_camera_correct_cfg.kp * error_m;

    s_integral += g_camera_correct_cfg.ki * error_m * dt;
    if (s_integral > g_camera_correct_cfg.i_max)
        s_integral = g_camera_correct_cfg.i_max;
    if (s_integral < -g_camera_correct_cfg.i_max)
        s_integral = -g_camera_correct_cfg.i_max;
    i_term = s_integral;

    d_term = g_camera_correct_cfg.kd * (error_m - s_last_error) / dt;
    s_last_error = error_m;

    vw = p_term + i_term + d_term;
    if (vw > g_camera_correct_cfg.out_max)
        vw = g_camera_correct_cfg.out_max;
    if (vw < -g_camera_correct_cfg.out_max)
        vw = -g_camera_correct_cfg.out_max;

    s_last_vw = vw;

    /* 写Vw到底盘，HIGH优先级 */
    Process_Flow_SetChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VW,
                                        PROCESS_FLOW_OVERRIDE_PRIORITY_HIGH,
                                        0.0f, 0.0f, vw);

    return vw;
}

uint8_t CameraCorrect_IsDone(void)
{
    return (uint8_t)(s_stable_count >= g_camera_correct_cfg.stable_need);
}

uint8_t CameraCorrect_IsTimeout(void)
{
    uint32_t now_ms;
    uint32_t age_ms;

    now_ms = osKernelGetTickCount();

    /* 总超时 */
    if ((now_ms - s_start_ms) >= g_camera_correct_cfg.timeout_ms)
        return 1U;

    /* 数据超时: 200ms以上无新帧 */
    age_ms = (uint32_t)(now_ms - s_last_frame_ms);
    if (age_ms >= g_camera_correct_cfg.data_timeout_ms)
        return 1U;

    return 0U;
}

static uint8_t s_dbg_inited = 0U;

void CameraCorrect_DebugRun(void)
{
    if (s_dbg_inited == 0U)
    {
        CameraCorrect_Reset();
        s_dbg_inited = 1U;
    }

    rc_send_raw_byte(0xCC);

    if (rc_get_kfs_lateral_fresh() != 0U)
    {
        (void)CameraCorrect_Update(rc_get_kfs_lateral_err_m());
    }

    if (CameraCorrect_IsTimeout() != 0U)
    {
        CameraCorrect_DebugExit();
    }
}

void CameraCorrect_DebugExit(void)
{
    rc_send_raw_byte(0xBB);
    Process_Flow_ClearChassisOverrideAxes(PROCESS_FLOW_CHASSIS_OVERRIDE_VW);
    CameraCorrect_Reset();
    s_dbg_inited = 0U;
}

float camera_kfs_to_lateral_error(float x, float y, float z)
{
    (void)x; (void)y; (void)z;
    return 0.0f;
}
