#include "chassis_heading_hold.h"
#include "main.h"
#include "Sensor_Task.h"
#include "dji_motor.h"

/* 航向保持参数（可在线调） */
volatile ChassisHeadingHold g_heading_hold =
{
    .kp = 3.0f,                  /* 比例增益：角度误差纠偏力度 */
    .ki = 0.1f,                  /* 积分增益：消除长期静差 */
    .kd = 0.3f,                 /* 微分增益：抑制摆动（配合角速度） */
    .i_limit = 10.0f,           /* 积分项限幅，防积分饱和 */
    .out_limit = 10.0f,         /* 总输出限幅（叠加到Vx_in的最大修正） */
    .yaw_ref_deg = 0.0f,         /* 参考航向角（deg） */
    .i_term = 0.0f,              /* 当前积分项累计值 */
    .last_yaw_deg = 0.0f,        /* 上一拍航向角（deg） */
    .yaw_rate_lpf = 0.0f,        /* 滤波后的角速度（deg/s） */
    .yaw_rate_lpf_alpha = 0.05f, /* 角速度一阶低通系数(0~1) */
    .last_tick_ms = 0U,          /* 上一拍时间戳（ms） */
    .yaw_inited = 0U                 /* 初始化标志：0未锁参考，1已锁 ，车开始平移时会从0变成1，角度控制pid开始工作*/
};

/* 逐轴加速度限幅参数（可在线调） */
volatile ChassisAxisLimiter g_vy_limiter = { .a_max = 3000.0f, .y = 0.0f, .last_tick_ms = 0U, .yaw_inited = 0U }; /* 前后轴最大变化率 */
volatile ChassisAxisLimiter g_vw_limiter = { .a_max = 1800.0f, .y = 0.0f, .last_tick_ms = 0U, .yaw_inited = 0U }; /* 左右轴最大变化率 */
volatile ChassisAxisLimiter g_vx_limiter = { .a_max = 2500.0f, .y = 0.0f, .last_tick_ms = 0U, .yaw_inited = 0U }; /* 旋转轴最大变化率 */

/* 平移锁角保持输入死区（可在线调） */
volatile float g_heading_hold_trans_deadband = 1.0f;
volatile float g_heading_hold_rot_deadband = 1.0f;
volatile uint32_t g_heading_hold_release_delay_ms = 3000U;

/* =========================
 * 平面 2×2 解耦 + 慢自适应trim（可在线调）
 * 目标：减少 Vy<->Vw 双向串扰（机构不对称/负载变化）
 * 说明：反馈来自四轮 speed_rpm 的“粗反解”，只用于慢速修正，不依赖绝对标定
 * ========================= */
volatile float g_decouple_k_yw_base = 0.0f;      /* Vy += k_yw * Vw *///解耦系数，用于补偿左右轮速度差异
volatile float g_decouple_k_wy_base = 0.0f;      /* Vw += k_wy * Vy *///解耦系数，用于补偿前后轮速度差异
volatile float g_decouple_k_yw_trim = 0.0f;//解耦系数，用于补偿左右轮速度差异
volatile float g_decouple_k_wy_trim = 0.0f;//解耦系数，用于补偿前后轮速度差异
volatile float g_decouple_trim_limit = 0.0f;    /* trim 限幅 */
volatile float g_decouple_k_total_limit = 10.0f; /* 总系数限幅 */
volatile float g_decouple_gamma_yw = 0.0003f;    /* 学习速率（越小越稳），用于补偿左右轮速度差异 */
volatile float g_decouple_gamma_wy = 0.0003f;//学习速率，用于补偿前后轮速度差异
volatile float g_decouple_lpf_alpha = 0.05f;     /* 反馈低通 */
volatile float g_decouple_cmd_deadband = 2.0f;   /* “纯单轴命令”判定死区（按你Vy/Vw命令量级调） */
volatile float g_decouple_meas_min_rpm = 10.0f;  /* 反馈过小不学习（抑制噪声） */
volatile float g_decouple_yaw_rate_max_dps = 50.0f; /* 姿态变化大不学习 */

static float g_decouple_vy_meas_lpf = 0.0f;//左右轮速度反馈低通滤波
static float g_decouple_vw_meas_lpf = 0.0f;//前后轮速度反馈低通滤波
static uint32_t g_decouple_last_tick_ms = 0U;//上一拍时间戳
static uint16_t g_decouple_persist_yw = 0U;//左右轮速度反馈低通滤波
static uint16_t g_decouple_persist_wy = 0U;//前后轮速度反馈低通滤波
static const uint16_t g_decouple_persist_need = 10U;//左右轮速度反馈低通滤波

/* 调试观测：直接镜像当前读取到的四个底盘轮速 */
volatile int16_t g_chassis_rpm_dbg_1 = 0;
volatile int16_t g_chassis_rpm_dbg_2 = 0;
volatile int16_t g_chassis_rpm_dbg_3 = 0;
volatile int16_t g_chassis_rpm_dbg_4 = 0;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float absf(float v) { return (v >= 0.0f) ? v : -v; }

/* 由四轮 rpm 反解得到“车体前后/左右”估计量（单位：rpm，比例常数未知但对慢trim足够） */
static void chassis_decode_vy_vw_from_wheel_rpm(float *vy_rpm, float *vw_rpm)
{
    /* 与 chassis.c 的混控一致：
     * w1=Vx+Vy+Vw; w2=Vx-Vy+Vw; w3=Vx+Vy-Vw; w4=Vx-Vy-Vw
     * => Vy=(w1-w2+w3-w4)/4; Vw=(w1+w2-w3-w4)/4
     */
    const int16_t rpm1 = chassis_motor1.speed_rpm;
    const int16_t rpm2 = chassis_motor2.speed_rpm;
    const int16_t rpm3 = chassis_motor3.speed_rpm;
    const int16_t rpm4 = chassis_motor4.speed_rpm;
    const float w1 = (float)rpm1;
    const float w2 = (float)rpm2;
    const float w3 = (float)rpm3;
    const float w4 = (float)rpm4;

    g_chassis_rpm_dbg_1 = rpm1;
    g_chassis_rpm_dbg_2 = rpm2;
    g_chassis_rpm_dbg_3 = rpm3;
    g_chassis_rpm_dbg_4 = rpm4;
//空指针保护，计算前后左右轮速度
    if (vy_rpm) *vy_rpm = (w1 - w2 + w3 - w4) * 0.25f;
    if (vw_rpm) *vw_rpm = (w1 + w2 - w3 - w4) * 0.25f;
}

void ChassisDecouple_Apply(float vx_cmd, float *vy_cmd, float *vw_cmd)
{
    float vy_rpm = 0.0f;
    float vw_rpm = 0.0f;
    float dt = 0.01f;
    uint32_t now = HAL_GetTick();
//空指针保护
    if (vy_cmd == 0 || vw_cmd == 0) return;
//如果上一拍时间戳为0，则设置上一拍时间戳
    if (g_decouple_last_tick_ms == 0U)
    {
        g_decouple_last_tick_ms = now;
    }
    else//如果上一拍时间戳不为0，则计算时间差
    {
        dt = (float)(now - g_decouple_last_tick_ms) / 1000.0f;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;//时间差不能为0
        g_decouple_last_tick_ms = now;//更新上一拍时间戳
    }
//反馈估计 + 低通
    chassis_decode_vy_vw_from_wheel_rpm(&vy_rpm, &vw_rpm);
    g_decouple_vy_meas_lpf = g_decouple_lpf_alpha * vy_rpm + (1.0f - g_decouple_lpf_alpha) * g_decouple_vy_meas_lpf;//左右轮速度反馈低通滤波
    g_decouple_vw_meas_lpf = g_decouple_lpf_alpha * vw_rpm + (1.0f - g_decouple_lpf_alpha) * g_decouple_vw_meas_lpf;//前后轮速度反馈低通滤波
//合成系数
    float k_yw = clampf(g_decouple_k_yw_base + g_decouple_k_yw_trim, -g_decouple_k_total_limit, g_decouple_k_total_limit);
    float k_wy = clampf(g_decouple_k_wy_base + g_decouple_k_wy_trim, -g_decouple_k_total_limit, g_decouple_k_total_limit);//合成系数
//先做静态解耦补偿（就地修改命令）
    {
        const float vy_in = *vy_cmd;
        const float vw_in = *vw_cmd;
        *vy_cmd = vy_in + k_yw * vw_in;
        *vw_cmd = vw_in + k_wy * vy_in;
    }

    /* ================= 慢自适应 trim（强门控）================= */
    const uint8_t no_rot_cmd = (absf(vx_cmd) < g_heading_hold_rot_deadband) ? 1U : 0U;//判断是否旋转
    const uint8_t yaw_stable = (absf(g_imu_gyr_z_dps) < g_decouple_yaw_rate_max_dps) ? 1U : 0U;//判断是否稳定
//用“原始命令”判断是否纯单轴（避免解耦后串扰影响判定）
    const float vy_raw_abs = absf(*vy_cmd);//左右轮速度绝对值
    const float vw_raw_abs = absf(*vw_cmd);//前后轮速度绝对值
//判断是否纯单轴
    const uint8_t pure_vw_cmd = (vw_raw_abs > g_decouple_cmd_deadband && vy_raw_abs < g_decouple_cmd_deadband) ? 1U : 0U;//判断是否纯单轴
    const uint8_t pure_vy_cmd = (vy_raw_abs > g_decouple_cmd_deadband && vw_raw_abs < g_decouple_cmd_deadband) ? 1U : 0U;//判断是否纯单轴

    //当不旋转且稳定且纯单轴且左右轮速度反馈低通滤波大于最小rpm时，学习k_yw
    if (no_rot_cmd && yaw_stable && pure_vw_cmd && absf(g_decouple_vy_meas_lpf) > g_decouple_meas_min_rpm)
    {
        //如果左右轮速度反馈低通滤波小于0xFFFFU，则增加1
        if (g_decouple_persist_yw < 0xFFFFU) g_decouple_persist_yw++;
        //如果左右轮速度反馈低通滤波大于等于10，则学习k_yw
        if (g_decouple_persist_yw >= g_decouple_persist_need)
        {
            g_decouple_k_yw_trim += g_decouple_gamma_yw * g_decouple_vy_meas_lpf * (*vw_cmd) * dt;//学习k_yw
            g_decouple_k_yw_trim = clampf(g_decouple_k_yw_trim, -g_decouple_trim_limit, g_decouple_trim_limit);//k_yw限幅
        }
    }
    else//如果左右轮速度反馈低通滤波大于等于10，则重置左右轮速度反馈低通滤波
    {
        g_decouple_persist_yw = 0U;//重置左右轮速度反馈低通滤波
    }
    //学 k_wy：前后为主时，希望左右反馈≈0
    if (no_rot_cmd && yaw_stable && pure_vy_cmd && absf(g_decouple_vw_meas_lpf) > g_decouple_meas_min_rpm)
    {
        if (g_decouple_persist_wy < 0xFFFFU) g_decouple_persist_wy++;
        if (g_decouple_persist_wy >= g_decouple_persist_need)
        {
            g_decouple_k_wy_trim += g_decouple_gamma_wy * g_decouple_vw_meas_lpf * (*vy_cmd) * dt;
            g_decouple_k_wy_trim = clampf(g_decouple_k_wy_trim, -g_decouple_trim_limit, g_decouple_trim_limit);
        }
    }
    else
    {
        g_decouple_persist_wy = 0U;
    }
}

//wrap to (-180, 180]
static float wrap_deg(float d)
{
    while (d > 180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

//reset the limiter  加速度限幅没用上
void ChassisAxisLimiter_Reset(ChassisAxisLimiter *lim, float y0)
{
    if (lim == 0) return;
    lim->y = y0;
    lim->last_tick_ms = HAL_GetTick();
    lim->yaw_inited = 1U;
}

//update the limiter  加速度限幅没用上
float ChassisAxisLimiter_Update(ChassisAxisLimiter *lim, float target)
{
    uint32_t now = 0U;
    float dt = 0.0f;
    float max_step = 0.0f;
    float diff = 0.0f;

    if (lim == 0) return target;

    now = HAL_GetTick();
    dt = (float)(now - lim->last_tick_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f;
    }
    lim->last_tick_ms = now;

    if (lim->yaw_inited == 0U)
    {
        lim->y = target;
        lim->yaw_inited = 1U;
        return lim->y;
    }

    if (lim->a_max <= 0.0f)
    {
        lim->y = target;
        return lim->y;
    }

    max_step = lim->a_max * dt;
    diff = target - lim->y;
    diff = clampf(diff, -max_step, max_step);
    lim->y += diff;
    return lim->y;
}


//reset the reference of the heading hold
void ChassisHeadingHold_ResetRef(ChassisHeadingHold *hh, float yaw_deg)
{
    if (hh == 0) return;

    hh->yaw_ref_deg = yaw_deg;
    hh->i_term = 0.0f;
    hh->last_yaw_deg = yaw_deg;
    hh->yaw_rate_lpf = 0.0f;
    hh->last_tick_ms = HAL_GetTick();
    hh->yaw_inited = 1U;
}

//update the heading hold (internal PID output)
static float ChassisHeadingHold_Update(ChassisHeadingHold *hh, float yaw_deg)
{
    uint32_t now = 0U;
    float dt = 0.0f;
    float err = 0.0f;
    float yaw_rate = 0.0f;
    float d_term = 0.0f;
    float out = 0.0f;
    //空指针保护
    if (hh == 0) return 0.0f;

    now = HAL_GetTick();
    //计算时间差
    dt = (float)(now - hh->last_tick_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f; /* 兜底，避免初次/暂停后dt异常，时间差不能为0 */
    }
    hh->last_tick_ms = now;
    //如果未初始化，则初始化，角度控制pid开始工作
    if (hh->yaw_inited == 0U)
    {
        ChassisHeadingHold_ResetRef(hh, yaw_deg);
        return 0.0f;
    }
    //计算角度误差
    /* error = ref - meas */
    err = wrap_deg(hh->yaw_ref_deg - yaw_deg);


    /* D项使用陀螺仪Z轴角速度（deg/s），并做一阶滤波 */
    //获取陀螺仪Z轴角速度作为微分项
    yaw_rate = g_imu_gyr_z_dps;
    hh->last_yaw_deg = yaw_deg; /* 保留用于观测/调试 */
    //一阶滤波
    hh->yaw_rate_lpf = hh->yaw_rate_lpf_alpha * yaw_rate + (1.0f - hh->yaw_rate_lpf_alpha) * hh->yaw_rate_lpf;
    //计算积分项
    hh->i_term += hh->ki * err * dt;
    hh->i_term = clampf(hh->i_term, -hh->i_limit, hh->i_limit);

    /* D项用“测得角速度”近似：d/dt(err)= -yaw_rate */
    d_term = hh->kd * (-hh->yaw_rate_lpf);

    /* 底盘旋转正方向与航向误差定义相反，输出整体取反 */
    out = -(hh->kp * err + hh->i_term + d_term);
    out = clampf(out, -hh->out_limit, hh->out_limit);

    return out;
}

//translation hold step
float ChassisHeadingHold_TranslationHoldStep(ChassisHeadingHold *hh,
                                            float yaw_body_deg,
                                            float vx_cmd,
                                            float vy_cmd,
                                            float vw_cmd)
{
    static uint8_t trans_moving_last = 0U;//平移状态标志，0表示未平移，1表示平移
    static uint8_t release_timing = 0U;//延时标志，0表示未延时，1表示延时，用作车辆平移后延时退出保持，抑制停车漂移
    static uint32_t release_start_ms = 0U;//延时开始时间戳
    float trans_abs_sum = 0.0f;//平移输入绝对值之和
    uint8_t trans_moving_now = 0U;//平移状态标志，0表示未平移，1表示平移
    float rot_cmd_abs = 0.0f;//旋转输入绝对值
    uint32_t now_ms = HAL_GetTick();//当前时间戳

    //空指针保护
    if (hh == 0) return 0.0f;

    //计算平移输入绝对值之和
    trans_abs_sum = ((vy_cmd >= 0.0f) ? vy_cmd : -vy_cmd)
                  + ((vw_cmd >= 0.0f) ? vw_cmd : -vw_cmd);
    //判断是否平移
    trans_moving_now = (trans_abs_sum > g_heading_hold_trans_deadband) ? 1U : 0U;

    rot_cmd_abs = (vx_cmd >= 0.0f) ? vx_cmd : -vx_cmd;

    //判断是否旋转
    /* 若有旋转输入，立即退出保持（避免与人为旋转叠加对抗） */
    if (rot_cmd_abs > g_heading_hold_rot_deadband)
    {
        hh->yaw_inited = 0U;
        release_timing = 0U;
        trans_moving_last = trans_moving_now;
        return 0.0f;
    }

    /* 仅在“平移且无旋转输入”时启用保持 */
    if (trans_moving_now != 0U)
    {
        /* 平移刚开始时锁定参考角为“开始平移前的机身角度” */
        if (trans_moving_last == 0U || hh->yaw_inited == 0U)
        {
            ChassisHeadingHold_ResetRef(hh, yaw_body_deg);
        }
        //重置延时标志
        release_timing = 0U;
        //更新平移状态标志
        trans_moving_last = trans_moving_now;
        //更新角度控制pid输出
        return ChassisHeadingHold_Update(hh, yaw_body_deg);
    }

    /* 未平移或停车时：先延时保持，到达ms阈值后再退出 */
    if (hh->yaw_inited != 0U)
    {
        //如果延时标志为0，则设置延时标志，并记录延时开始时间戳
        if (release_timing == 0U)
        {
            release_timing = 1U;
            release_start_ms = now_ms;
        }

        //如果延时时间小于阈值，则继续保持
        if ((uint32_t)(now_ms - release_start_ms) < g_heading_hold_release_delay_ms)
        {
            trans_moving_last = trans_moving_now;
            return ChassisHeadingHold_Update(hh, yaw_body_deg);
        }

        //如果延时时间大于阈值，则退出保持、重置延时标志、更新平移状态标志
        //角度控制pid不工作
        hh->yaw_inited = 0U; 
        //重置延时标志
        release_timing = 0U;
        //更新平移状态标志
        trans_moving_last = trans_moving_now;
        //返回0，不进行角度控制
        return 0.0f;
    }
    //角度控制pid不工作
    hh->yaw_inited = 0U; 
    //重置延时标志
    release_timing = 0U; 
    //更新平移状态标志
    trans_moving_last = trans_moving_now;
    //返回0，不进行角度控制
    return 0.0f;
}

