#include "chassis_heading_hold.h"
#include "main.h"
#include "Sensor_Task.h"
#include "dji_motor.h"
#include "upper_pc_protocol.h"
#include "common.h"
#include <math.h>

/* 航向保持参数（可在线调） */
volatile ChassisHeadingHold g_heading_hold =
{
    .kp = 2.8f,                  /* 比例增益：角度误差纠偏力度 */
    .ki = 0.15f,                  /* 积分增益：消除长期静差 */
    .kd = 0.7f,                 /* 微分增益：抑制摆动（配合角速度） */
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

/* 逐轴加速度限幅参数（可在线调）（让车不那么快加速和减速） */
/**
  * @brief 前后轴加速度限幅参数
  * @param g_vy_limiter 前后轴加速度限幅参数
  */
volatile ChassisAxisLimiter g_vy_limiter = { 
    .a_max = 3000.0f, //前后轴最大变化率
    .y = 0.0f, //当前输出
    .last_tick_ms = 0U, //上一拍时间戳
    .yaw_inited = 0U //初始化标志
};
/**
  * @brief 左右轴加速度限幅参数
  * @param g_vw_limiter 左右轴加速度限幅参数
  */
volatile ChassisAxisLimiter g_vw_limiter = { 
    .a_max = 1800.0f, //左右轴最大变化率
    .y = 0.0f, //当前输出
    .last_tick_ms = 0U, //上一拍时间戳
    .yaw_inited = 0U //初始化标志
};
/**
  * @brief 旋转轴加速度限幅参数
  * @param g_vx_limiter 旋转轴加速度限幅参数
  */
volatile ChassisAxisLimiter g_vx_limiter = { 
    .a_max = 2500.0f, //旋转轴最大变化率
    .y = 0.0f, //当前输出
    .last_tick_ms = 0U, //上一拍时间戳
    .yaw_inited = 0U //初始化标志
};

/* 平移锁角保持：输入门限与摇杆回中后延时退出（可在线调） */
volatile ChassisHeadingHoldGate g_heading_hold_gate = {
    .trans_deadband = 1.0f,//平移死区
    .rot_deadband = 0.4f,//旋转死区
    .release_delay_ms = 3000U,//延时退出
};

/**
  * @brief 平面 Vy/Vw 解耦 + 慢自适应 trim参数
  * @param g_decouple_tune 平面 Vy/Vw 解耦 + 慢自适应 trim参数
  */
volatile ChassisDecoupleTune g_decouple_tune = {
    .k_yw_base = 0.0f,//y轴比例增益
    .k_wy_base = 0.0f,//w轴比例增益
    .k_yw_trim = 0.0f,//y轴积分交叉
    .k_wy_trim = 0.0f,//w轴积分交叉
    .trim_limit = 0.0f,//限幅
    .k_total_limit = 10.0f,//总限幅
    .gamma_yw = 0.0003f,//y轴学习率
    .gamma_wy = 0.0003f,//w轴学习率
    .lpf_alpha = 0.05f,//低通滤波系数
    .cmd_deadband = 2.0f,//命令死区
    .meas_min_rpm = 10.0f,//最小rpm
    .yaw_rate_max_dps = 50.0f,//最大角速度
};

/**
  * @brief 起步/停车瞬态补偿参数
  * @param g_transient_tune 起步/停车瞬态补偿参数
  */
volatile ChassisTransientTune g_transient_tune = {
    .move_deadband = 2.0f,//平移死区
    .step_trigger = 20.0f,//步触发
    .window_ms = 220U,//窗口时间
    .yaw_damp_gain = 0.05f,//角速度阻尼增益
    .vw_ff_gain = 2.0f,//w轴前馈增益
    .vy_ff_gain = 1.5f,//y轴前馈增益
    .amp_max = 3.0f,//幅值最大值
    .out_limit = 12.0f,//输出限幅
};

/**
  * @brief 里程计漂移补偿参数
  * @param g_odom_drift_tune 里程计漂移补偿参数
  */
volatile ChassisOdomDriftTune g_odom_drift_tune = {
    .enable = 1U,//启用标志
    .cmd_deadband = 5.0f,//命令死区
    .rot_deadband = 0.4f,//旋转死区
    .kp_cross = 0.08f,//y轴积分交叉比例增益
    .ki_cross = 0.02f,//w轴积分交叉积分增益
    .i_limit = 30.0f,//积分限幅
    .out_limit = 8.0f,//输出限幅
    .max_dt_s = 0.1f,//最大时间差
};


/**
  * @brief 里程计漂移补偿状态
  * @param g_odom_drift_state 里程计漂移补偿状态
  */
typedef struct
{
    uint8_t inited;//初始化标志
    float last_x_m;//上一拍x轴位置
    float last_y_m;//上一拍y轴位置
    uint32_t last_ms;//上一拍时间戳
    float i_vy_cross;//y轴积分交叉
    float i_vw_cross;//w轴积分交叉
} ChassisOdomDriftState;

static float g_decouple_vy_meas_lpf = 0.0f;//左右轮速度反馈低通滤波
static float g_decouple_vw_meas_lpf = 0.0f;//前后轮速度反馈低通滤波
static uint32_t g_decouple_last_tick_ms = 0U;//上一拍时间戳
static uint16_t g_decouple_persist_yw = 0U;//左右轮速度反馈低通滤波
static uint16_t g_decouple_persist_wy = 0U;//前后轮速度反馈低通滤波
static const uint16_t g_decouple_persist_need = 10U;//左右轮速度反馈低通滤波

static ChassisOdomDriftState g_odom_drift_st = {0U, 0.0f, 0.0f, 0U, 0.0f, 0.0f};

/**
  * @brief 由四轮 rpm 反解得到“车体前后/左右”估计量（单位：rpm，比例常数未知但对慢trim足够）
  * @param vy_rpm 前后轮速度
  * @param vw_rpm 左右轮速度
  */
static void chassis_decode_vy_vw_from_wheel_rpm(float *vy_rpm, float *vw_rpm)
{
    /* 与 chassis.c 的混控一致：
     * w1=Vx+Vy+Vw; w2=Vx-Vy+Vw; w3=Vx+Vy-Vw; w4=Vx-Vy-Vw
     * => Vy=(w1-w2+w3-w4)/4; Vw=(w1+w2-w3-w4)/4
     */
    const float w1 = (float)chassis_motor1.speed_rpm;
    const float w2 = (float)chassis_motor2.speed_rpm;
    const float w3 = (float)chassis_motor3.speed_rpm;
    const float w4 = (float)chassis_motor4.speed_rpm;
//空指针保护，计算前后左右轮速度
    if (vy_rpm) *vy_rpm = (w1 - w2 + w3 - w4) * 0.25f;
    if (vw_rpm) *vw_rpm = (w1 + w2 - w3 - w4) * 0.25f;
}

/**
  * @brief 平面解耦（前后<->左右）：
  * @param vx_cmd 旋转速度命令
  * @param vy_cmd 前后速度命令
  * @param vw_cmd 左右速度命令
  */
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
    g_decouple_vy_meas_lpf = g_decouple_tune.lpf_alpha * vy_rpm + (1.0f - g_decouple_tune.lpf_alpha) * g_decouple_vy_meas_lpf;//左右轮速度反馈低通滤波
    g_decouple_vw_meas_lpf = g_decouple_tune.lpf_alpha * vw_rpm + (1.0f - g_decouple_tune.lpf_alpha) * g_decouple_vw_meas_lpf;//前后轮速度反馈低通滤波
//合成系数
    float k_yw = clampf(g_decouple_tune.k_yw_base + g_decouple_tune.k_yw_trim, -g_decouple_tune.k_total_limit, g_decouple_tune.k_total_limit);
    float k_wy = clampf(g_decouple_tune.k_wy_base + g_decouple_tune.k_wy_trim, -g_decouple_tune.k_total_limit, g_decouple_tune.k_total_limit);//合成系数
//先做静态解耦补偿（就地修改命令）
    {
        const float vy_in = *vy_cmd;
        const float vw_in = *vw_cmd;
        *vy_cmd = vy_in + k_yw * vw_in;
        *vw_cmd = vw_in + k_wy * vy_in;
    }

    /* ================= 慢自适应 trim（强门控）================= */
    const uint8_t no_rot_cmd = (fabsf(vx_cmd) < g_heading_hold_gate.rot_deadband) ? 1U : 0U;//判断是否旋转
    const uint8_t yaw_stable = (fabsf(g_sensor_task_data.imu.gyr_z_dps) < g_decouple_tune.yaw_rate_max_dps) ? 1U : 0U;//判断是否稳定
//用“原始命令”判断是否纯单轴（避免解耦后串扰影响判定）
    const float vy_raw_abs = fabsf(*vy_cmd);//左右轮速度绝对值
    const float vw_raw_abs = fabsf(*vw_cmd);//前后轮速度绝对值
//判断是否纯单轴
    const uint8_t pure_vw_cmd = (vw_raw_abs > g_decouple_tune.cmd_deadband && vy_raw_abs < g_decouple_tune.cmd_deadband) ? 1U : 0U;//判断是否纯单轴
    const uint8_t pure_vy_cmd = (vy_raw_abs > g_decouple_tune.cmd_deadband && vw_raw_abs < g_decouple_tune.cmd_deadband) ? 1U : 0U;//判断是否纯单轴

    //当不旋转且稳定且纯单轴且左右轮速度反馈低通滤波大于最小rpm时，学习k_yw
    if (no_rot_cmd && yaw_stable && pure_vw_cmd && fabsf(g_decouple_vy_meas_lpf) > g_decouple_tune.meas_min_rpm)
    {
        //如果左右轮速度反馈低通滤波小于0xFFFFU，则增加1
        if (g_decouple_persist_yw < 0xFFFFU) g_decouple_persist_yw++;
        //如果左右轮速度反馈低通滤波大于等于10，则学习k_yw
        if (g_decouple_persist_yw >= g_decouple_persist_need)
        {
            g_decouple_tune.k_yw_trim += g_decouple_tune.gamma_yw * g_decouple_vy_meas_lpf * (*vw_cmd) * dt;//学习k_yw
            g_decouple_tune.k_yw_trim = clampf(g_decouple_tune.k_yw_trim, -g_decouple_tune.trim_limit, g_decouple_tune.trim_limit);//k_yw限幅
        }
    }
    else//如果左右轮速度反馈低通滤波大于等于10，则重置左右轮速度反馈低通滤波
    {
        g_decouple_persist_yw = 0U;//重置左右轮速度反馈低通滤波
    }
    //学 k_wy：前后为主时，希望左右反馈≈0
    if (no_rot_cmd && yaw_stable && pure_vy_cmd && fabsf(g_decouple_vw_meas_lpf) > g_decouple_tune.meas_min_rpm)
    {
        if (g_decouple_persist_wy < 0xFFFFU) g_decouple_persist_wy++;
        if (g_decouple_persist_wy >= g_decouple_persist_need)
        {
            g_decouple_tune.k_wy_trim += g_decouple_tune.gamma_wy * g_decouple_vw_meas_lpf * (*vy_cmd) * dt;
            g_decouple_tune.k_wy_trim = clampf(g_decouple_tune.k_wy_trim, -g_decouple_tune.trim_limit, g_decouple_tune.trim_limit);
        }
    }
    else
    {
        g_decouple_persist_wy = 0U;
    }
}

/**
  * @brief 起步/停车瞬态补偿
  * @param vx_cmd 旋转速度命令
  * @param vy_cmd 前后速度命令
  * @param vw_cmd 左右速度命令
  * @return 需要叠加到旋转通道的修正量
  */
float ChassisTransientComp_Update(float vx_cmd, float vy_cmd, float vw_cmd)
{
    static uint8_t moving_last = 0U;
    static float vy_last = 0.0f;
    static float vw_last = 0.0f;
    static uint32_t window_start_ms = 0U;
    static float ff_vw_hold = 0.0f;//左右平移事件触发的Vx前馈（幅值锁存）
    static float ff_vy_hold = 0.0f;//前后平移事件触发的Vx前馈（幅值锁存）

    uint32_t now_ms = HAL_GetTick();
    float move_abs_sum = fabsf(vy_cmd) + fabsf(vw_cmd);
    uint8_t moving_now = (move_abs_sum > g_transient_tune.move_deadband) ? 1U : 0U;
    float d_vy = vy_cmd - vy_last;
    float d_vw = vw_cmd - vw_last;
    float delta_abs_sum = fabsf(d_vy) + fabsf(d_vw);
    uint8_t start_event = (moving_last == 0U && moving_now != 0U && delta_abs_sum > g_transient_tune.step_trigger) ? 1U : 0U;
    uint8_t stop_event = (moving_last != 0U && moving_now == 0U && (fabsf(vy_last) + fabsf(vw_last)) > g_transient_tune.move_deadband) ? 1U : 0U;
    float out = 0.0f;

    /* 有人为旋转输入时不叠加瞬态补偿，避免打架 */
    if (fabsf(vx_cmd) > g_heading_hold_gate.rot_deadband)
    {
        moving_last = moving_now;
        vy_last = vy_cmd;
        vw_last = vw_cmd;
        return 0.0f;
    }

    /* 在起步/停车突变时开启短时补偿窗口，并锁定方向 */
    if (start_event != 0U || stop_event != 0U)
    {
        float amp_norm = 1.0f;
        window_start_ms = now_ms;
        if (g_transient_tune.step_trigger > 0.0f)
        {
            amp_norm = delta_abs_sum / g_transient_tune.step_trigger;
        }
        amp_norm = clampf(amp_norm, 0.0f, g_transient_tune.amp_max);
        ff_vw_hold = ((d_vw > 0.0f) ? 1.0f : ((d_vw < 0.0f) ? -1.0f : 0.0f)) * amp_norm;
        ff_vy_hold = ((d_vy > 0.0f) ? 1.0f : ((d_vy < 0.0f) ? -1.0f : 0.0f)) * amp_norm;
    }

    if ((uint32_t)(now_ms - window_start_ms) < g_transient_tune.window_ms)
    {
        float env = 1.0f;
        if (g_transient_tune.window_ms > 0U)
        {
            env = 1.0f - ((float)(now_ms - window_start_ms) / (float)g_transient_tune.window_ms);
            env = clampf(env, 0.0f, 1.0f);
        }
        /* 角速度阻尼：优先压住短时摆动 */
        out += -g_transient_tune.yaw_damp_gain * g_sensor_task_data.imu.gyr_z_dps;

        /* 幅值相关前馈：跳变越大，补偿越强，并随窗口衰减 */
        out += g_transient_tune.vw_ff_gain * ff_vw_hold * env;
        out += g_transient_tune.vy_ff_gain * ff_vy_hold * env;
    }

    moving_last = moving_now;
    vy_last = vy_cmd;
    vw_last = vw_cmd;
    return clampf(out, -g_transient_tune.out_limit, g_transient_tune.out_limit);
}

/**
  * @brief 里程计漂移补偿
  * @param yaw_body_deg 车身航向角
  * @param vx_cmd 旋转速度命令
  * @param vy_cmd 前后速度命令
  * @param vw_cmd 左右速度命令
  * @param vy_corr 前后速度补偿
  * @param vw_corr 左右速度补偿
  */
void ChassisOdomDriftComp_Update(float yaw_body_deg,
                                 float vx_cmd,
                                 float vy_cmd,
                                 float vw_cmd,
                                 float *vy_corr,
                                 float *vw_corr)
{
    const rc_odom_t *odom = NULL;
    uint32_t now_ms = HAL_GetTick();
    float dt_s = 0.0f;
    float dx = 0.0f;
    float dy = 0.0f;
    float vy_meas = 0.0f;
    float vw_meas = 0.0f;
    float yaw_rad = yaw_body_deg * (M_PI_F / 180.0f);
    uint8_t pure_vy = 0U;
    uint8_t pure_vw = 0U;
    float vy_abs = fabsf(vy_cmd);
    float vw_abs = fabsf(vw_cmd);

    if (vy_corr == 0 || vw_corr == 0)
    {
        return;
    }
    *vy_corr = 0.0f;
    *vw_corr = 0.0f;

    if (g_odom_drift_tune.enable == 0U)
    {
        g_odom_drift_st.i_vy_cross = 0.0f;
        g_odom_drift_st.i_vw_cross = 0.0f;
        return;
    }
    if (fabsf(vx_cmd) > g_odom_drift_tune.rot_deadband)
    {
        g_odom_drift_st.i_vy_cross = 0.0f;
        g_odom_drift_st.i_vw_cross = 0.0f;
        return;
    }
    if (rc_odom_is_valid() == 0U)
    {
        g_odom_drift_st.inited = 0U;
        g_odom_drift_st.i_vy_cross = 0.0f;
        g_odom_drift_st.i_vw_cross = 0.0f;
        return;
    }

    odom = rc_get_latest_odom();
    if (g_odom_drift_st.inited == 0U)
    {
        g_odom_drift_st.inited = 1U;
        g_odom_drift_st.last_x_m = odom->x;
        g_odom_drift_st.last_y_m = odom->y;
        g_odom_drift_st.last_ms = now_ms;
        return;
    }

    dt_s = (float)(now_ms - g_odom_drift_st.last_ms) * 0.001f;
    if (dt_s <= 1e-4f || dt_s > g_odom_drift_tune.max_dt_s)
    {
        g_odom_drift_st.last_x_m = odom->x;
        g_odom_drift_st.last_y_m = odom->y;
        g_odom_drift_st.last_ms = now_ms;
        return;
    }

    dx = odom->x - g_odom_drift_st.last_x_m;
    dy = odom->y - g_odom_drift_st.last_y_m;
    g_odom_drift_st.last_x_m = odom->x;
    g_odom_drift_st.last_y_m = odom->y;
    g_odom_drift_st.last_ms = now_ms;

    /* 世界系位移差分 -> 车体系速度（与底盘轴定义一致） */
    vy_meas = (cosf(yaw_rad) * dy + sinf(yaw_rad) * dx) / dt_s;
    vw_meas = (-sinf(yaw_rad) * dy + cosf(yaw_rad) * dx) / dt_s;

    pure_vy = (vy_abs > g_odom_drift_tune.cmd_deadband && vw_abs < g_odom_drift_tune.cmd_deadband) ? 1U : 0U;
    pure_vw = (vw_abs > g_odom_drift_tune.cmd_deadband && vy_abs < g_odom_drift_tune.cmd_deadband) ? 1U : 0U;

    if (pure_vy != 0U)
    {
        const float err_cross = vw_meas;
        g_odom_drift_st.i_vw_cross += err_cross * dt_s;
        g_odom_drift_st.i_vw_cross = clampf(g_odom_drift_st.i_vw_cross, -g_odom_drift_tune.i_limit, g_odom_drift_tune.i_limit);
        *vw_corr = -(g_odom_drift_tune.kp_cross * err_cross + g_odom_drift_tune.ki_cross * g_odom_drift_st.i_vw_cross);
        *vw_corr = clampf(*vw_corr, -g_odom_drift_tune.out_limit, g_odom_drift_tune.out_limit);
        g_odom_drift_st.i_vy_cross = 0.0f;
    }
    else if (pure_vw != 0U)
    {
        const float err_cross = vy_meas;
        g_odom_drift_st.i_vy_cross += err_cross * dt_s;
        g_odom_drift_st.i_vy_cross = clampf(g_odom_drift_st.i_vy_cross, -g_odom_drift_tune.i_limit, g_odom_drift_tune.i_limit);
        *vy_corr = -(g_odom_drift_tune.kp_cross * err_cross + g_odom_drift_tune.ki_cross * g_odom_drift_st.i_vy_cross);
        *vy_corr = clampf(*vy_corr, -g_odom_drift_tune.out_limit, g_odom_drift_tune.out_limit);
        g_odom_drift_st.i_vw_cross = 0.0f;
    }
    else
    {
        g_odom_drift_st.i_vy_cross = 0.0f;
        g_odom_drift_st.i_vw_cross = 0.0f;
    }
}

//wrap to (-180, 180]
//reset the limiter  加速度限幅没用上
/**
  * @brief 重置加速度限幅
  * @param lim 加速度限幅
  * @param y0 初始值
  */
void ChassisAxisLimiter_Reset(ChassisAxisLimiter *lim, float y0)
{
    if (lim == 0) return;
    lim->y = y0;
    lim->last_tick_ms = HAL_GetTick();
    lim->yaw_inited = 1U;
}

//update the limiter  加速度限幅没用上
/**
  * @brief 更新加速度限幅
  * @param lim 加速度限幅
  * @param target 目标值
  * @return 更新后的值
  */
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
/**
  * @brief 重置航向保持参考
  * @param hh 航向保持
  * @param yaw_deg 航向角
  */
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
/**
  * @brief 更新航向保持（内部PID输出）
  * @param hh 航向保持
  * @param yaw_deg 航向角
  * @return 需要叠加到旋转通道的修正量
  */
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
    err = wrap_deg_180(hh->yaw_ref_deg - yaw_deg);


    /* D项使用陀螺仪Z轴角速度（deg/s），并做一阶滤波 */
    //获取陀螺仪Z轴角速度作为微分项
    yaw_rate = g_sensor_task_data.imu.gyr_z_dps;
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
    trans_moving_now = (trans_abs_sum > g_heading_hold_gate.trans_deadband) ? 1U : 0U;

    rot_cmd_abs = (vx_cmd >= 0.0f) ? vx_cmd : -vx_cmd;

    //判断是否旋转
    /* 若有旋转输入，立即退出保持（避免与人为旋转叠加对抗） */
    if (rot_cmd_abs > g_heading_hold_gate.rot_deadband)
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
        if ((uint32_t)(now_ms - release_start_ms) < g_heading_hold_gate.release_delay_ms)
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

