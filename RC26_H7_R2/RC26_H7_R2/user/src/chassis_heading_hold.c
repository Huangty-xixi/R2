#include "chassis_heading_hold.h"
#include "main.h"
#include "Sensor_Task.h"

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
volatile uint32_t g_heading_hold_release_delay_ms = 2000U;

//amplitude limiting function
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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

