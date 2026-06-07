#include "weapon.h"
#include "remote_control.h"
#include "Motion_Task.h"
#include "clamp_head_ctrl.h"
#include "main.h"
#include "tim.h"
#include "chassis.h"
#include "dji_motor.h"
#include <math.h>
#include <stdlib.h>



DJI_MotorModule weapon_clamp_motor;
float weapon_clamp_motor_pos_pid_param[PID_PARAMETER_NUM] = {0.8f, 0.0f, 0.05f, 1.0f, 500.0f, 8192.0f};
float weapon_clamp_motor_spd_pid_param[PID_PARAMETER_NUM] = {5.0f, 0.4f, 0.2f, 1.0f, 500.0f, 10000.0f};
volatile weapon_tune_t g_weapon_tune = {
    .clamp = {
        .close_pos_ticks = 1200.0f,
        .pos_tol_ticks = 50.0f,
        .arrival_rpm_thr = 10,
        .arrival_confirm_cnt = 3U,
        .move_timeout_ms = 3000U,
    },
    .servo = {
        .pwm_mid = 1285U,
        .pwm_upright = 2150U,
    },
};
volatile weapon_clamp_motor_dbg_t g_weapon_clamp_motor_dbg;

static weapon_clamp_pos_fsm_t s_weapon_clamp_pos_fsm = WEAPON_CLAMP_POS_AT_ZERO;
static float s_weapon_clamp_pos_cmd;
static uint8_t s_weapon_clamp_arrival_cnt;
static uint8_t s_weapon_clamp_force_zero_output;
static uint32_t s_weapon_clamp_move_start_ms;

// 舵机状态
uint8_t servo_state = 0;    // 舵机状态
uint8_t clamp_state = 0;     // 夹爪状态
uint8_t sucker1_state = 0;     // 吸盘1状态
uint8_t sucker2_state = 0;     // 吸盘2状态
uint8_t sucker3_state = 0;     // 吸盘3状态
uint8_t sucker4_state = 0;     // 吸盘4状态

// 舵机锁定
uint8_t ch5_lock = 0;

/* master_weapon_action_bits */
#define MASTER_WEAPON_SERVO_BIT   (1U << 0)
#define MASTER_WEAPON_CLAMP_BIT   (1U << 1)
#define MASTER_WEAPON_SUCKER1_BIT (1U << 2)
#define MASTER_WEAPON_SUCKER2_BIT (1U << 3)
#define MASTER_WEAPON_SUCKER3_BIT (1U << 4)
#define MASTER_WEAPON_SUCKER4_BIT (1U << 5)

/**
 * @brief 吸盘1+2 打开，吸盘3+4 关闭，GPIO 使用 sucker*_use 设置
 * @param open1 吸盘1打开(1)/关闭(0)
 * @param open2 吸盘2打开(1)/关闭(0)
 */
void pump1_two_suckers_linkage_nominal_open(uint8_t open1, uint8_t open2)
{
    (void)open1;
    (void)open2;
}

/**
 * @brief 吸盘3+4 打开，吸盘1+2 关闭，GPIO 使用 sucker*_use 设置
 * @param open3 吸盘3打开(1)/关闭(0)
 * @param open4 吸盘4打开(1)/关闭(0)
 */
void pump2_two_suckers_linkage_nominal_open(uint8_t open3, uint8_t open4)
{
    (void)open3;
    (void)open4;
}

void weapon_init(void)
{
    Weapon_ClampMotor_Init();
}

void Weapon_ClampMotor_Init(void)
{
    DJImotor_Create(&weapon_clamp_motor,
                    WEAPON_CLAMP_MOTOR_CMD_ID,
                    WEAPON_CLAMP_MOTOR_FEEDBACK_ID,
                    &hfdcan2,
                    DJI_2006,
                    POSITION,
                    PID_POSITION,
                    weapon_clamp_motor_pos_pid_param);
    PID_Init(&weapon_clamp_motor.pid_spd, PID_POSITION, weapon_clamp_motor_spd_pid_param);
    Weapon_ClampMotor_Reset();
}

static void weapon_clamp_pid_clear(void)
{
    if (weapon_clamp_motor.pid_pos.PID_Calc_Clear != 0)
    {
        weapon_clamp_motor.pid_pos.PID_Calc_Clear(&weapon_clamp_motor.pid_pos);
    }
    if (weapon_clamp_motor.pid_spd.PID_Calc_Clear != 0)
    {
        weapon_clamp_motor.pid_spd.PID_Calc_Clear(&weapon_clamp_motor.pid_spd);
    }
}

/** 上电/复位：当前编码器作为 total_angle=0 的零点 */
static void weapon_clamp_zero_calibrate(void)
{
    weapon_clamp_motor.offset_angle = weapon_clamp_motor.angle;
    weapon_clamp_motor.round_cnt = 0;
    weapon_clamp_motor.total_angle = 0;
}

static void weapon_clamp_dbg_sync_motion(void)
{
    switch (s_weapon_clamp_pos_fsm)
    {
    case WEAPON_CLAMP_POS_GO_ZERO:
        g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_OPENING;
        break;
    case WEAPON_CLAMP_POS_GO_TARGET:
        g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_CLOSING;
        break;
    case WEAPON_CLAMP_POS_AT_ZERO:
        g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_AT_OPEN;
        break;
    case WEAPON_CLAMP_POS_AT_TARGET:
        g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_AT_CLOSE;
        break;
    case WEAPON_CLAMP_POS_IDLE:
    default:
        break;
    }

    g_weapon_clamp_motor_dbg.pos_fsm = s_weapon_clamp_pos_fsm;
    g_weapon_clamp_motor_dbg.at_open_limit =
        (uint8_t)(s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_AT_ZERO);
    g_weapon_clamp_motor_dbg.at_close_limit =
        (uint8_t)(s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_AT_TARGET);
    g_weapon_clamp_motor_dbg.is_busy =
        (uint8_t)((s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_GO_ZERO) ||
                  (s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_GO_TARGET));
}

static void weapon_clamp_begin_move(weapon_clamp_pos_fsm_t next_fsm)
{
    s_weapon_clamp_pos_fsm = next_fsm;
    s_weapon_clamp_arrival_cnt = 0U;
    s_weapon_clamp_force_zero_output = 0U;
    s_weapon_clamp_move_start_ms = HAL_GetTick();
    g_weapon_clamp_motor_dbg.move_fault = 0U;
    weapon_clamp_pid_clear();
    weapon_clamp_dbg_sync_motion();
}

void Weapon_ClampMotor_Reset(void)
{
    weapon_clamp_zero_calibrate();
    weapon_clamp_pid_clear();

    s_weapon_clamp_pos_fsm = WEAPON_CLAMP_POS_AT_ZERO;
    s_weapon_clamp_pos_cmd = 0.0f;
    s_weapon_clamp_arrival_cnt = 0U;
    s_weapon_clamp_force_zero_output = 1U;
    s_weapon_clamp_move_start_ms = HAL_GetTick();

    g_weapon_clamp_motor_dbg.target = WEAPON_CLAMP_TARGET_OPEN;
    g_weapon_clamp_motor_dbg.pos_cmd = 0.0f;
    g_weapon_clamp_motor_dbg.total_angle = (float)weapon_clamp_motor.total_angle;
    g_weapon_clamp_motor_dbg.pos_err = 0.0f;
    g_weapon_clamp_motor_dbg.pid_input = 0.0f;
    g_weapon_clamp_motor_dbg.pid_output = 0.0f;
    g_weapon_clamp_motor_dbg.speed_rpm = 0;
    g_weapon_clamp_motor_dbg.speed_rpm_abs = 0;
    g_weapon_clamp_motor_dbg.arrival_cnt = 0U;
    g_weapon_clamp_motor_dbg.move_fault = 0U;
    g_weapon_clamp_motor_dbg.step_tick = HAL_GetTick();
    g_weapon_clamp_motor_dbg.move_start_ms = s_weapon_clamp_move_start_ms;
    g_weapon_clamp_motor_dbg.move_elapsed_ms = 0U;
    weapon_clamp_motor.pid_spd.Output = 0.0f;
    weapon_clamp_dbg_sync_motion();
}

void Weapon_ClampMotor_GoToZero(void)
{
    if (s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_AT_ZERO)
    {
        s_weapon_clamp_pos_cmd = 0.0f;
        s_weapon_clamp_force_zero_output = 1U;
        weapon_clamp_dbg_sync_motion();
        return;
    }
    weapon_clamp_begin_move(WEAPON_CLAMP_POS_GO_ZERO);
}

void Weapon_ClampMotor_GoToTarget(void)
{
    if (s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_AT_TARGET)
    {
        s_weapon_clamp_pos_cmd = g_weapon_tune.clamp.close_pos_ticks;
        s_weapon_clamp_force_zero_output = 1U;
        weapon_clamp_dbg_sync_motion();
        return;
    }
    weapon_clamp_begin_move(WEAPON_CLAMP_POS_GO_TARGET);
}

void Weapon_ClampMotor_SetTarget(uint8_t close)
{
    g_weapon_clamp_motor_dbg.target =
        (close != 0U) ? WEAPON_CLAMP_TARGET_CLOSE : WEAPON_CLAMP_TARGET_OPEN;

    if (close != 0U)
    {
        Weapon_ClampMotor_GoToTarget();
    }
    else
    {
        Weapon_ClampMotor_GoToZero();
    }
}

static uint8_t weapon_clamp_pos_arrived(float pos_err, int rpm_abs)
{
    const float tol = g_weapon_tune.clamp.pos_tol_ticks;

    if (!isfinite(pos_err) || !isfinite(tol) || (tol <= 0.0f))
    {
        s_weapon_clamp_arrival_cnt = 0U;
        return 0U;
    }
    if (fabsf(pos_err) > tol)
    {
        s_weapon_clamp_arrival_cnt = 0U;
        return 0U;
    }
    if (rpm_abs > (int)g_weapon_tune.clamp.arrival_rpm_thr)
    {
        s_weapon_clamp_arrival_cnt = 0U;
        return 0U;
    }
    if (g_weapon_tune.clamp.arrival_confirm_cnt == 0U)
    {
        return 1U;
    }
    s_weapon_clamp_arrival_cnt++;
    if (s_weapon_clamp_arrival_cnt >= g_weapon_tune.clamp.arrival_confirm_cnt)
    {
        return 1U;
    }
    return 0U;
}

static void weapon_clamp_pos_fsm_step(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const int rpm_abs = abs((int)weapon_clamp_motor.speed_rpm);
    float pos_err;
    uint32_t elapsed_ms;

    g_weapon_clamp_motor_dbg.speed_rpm = weapon_clamp_motor.speed_rpm;
    g_weapon_clamp_motor_dbg.speed_rpm_abs = (int16_t)rpm_abs;
    g_weapon_clamp_motor_dbg.total_angle = (float)weapon_clamp_motor.total_angle;
    g_weapon_clamp_motor_dbg.step_tick = now_ms;

    switch (s_weapon_clamp_pos_fsm)
    {
    case WEAPON_CLAMP_POS_GO_ZERO:
        s_weapon_clamp_pos_cmd = 0.0f;
        break;
    case WEAPON_CLAMP_POS_GO_TARGET:
        s_weapon_clamp_pos_cmd = g_weapon_tune.clamp.close_pos_ticks;
        break;
    case WEAPON_CLAMP_POS_AT_ZERO:
        s_weapon_clamp_pos_cmd = 0.0f;
        s_weapon_clamp_force_zero_output = 1U;
        g_weapon_clamp_motor_dbg.pos_cmd = s_weapon_clamp_pos_cmd;
        g_weapon_clamp_motor_dbg.pos_err =
            s_weapon_clamp_pos_cmd - (float)weapon_clamp_motor.total_angle;
        g_weapon_clamp_motor_dbg.pid_input = s_weapon_clamp_pos_cmd;
        weapon_clamp_dbg_sync_motion();
        return;
    case WEAPON_CLAMP_POS_AT_TARGET:
        s_weapon_clamp_pos_cmd = g_weapon_tune.clamp.close_pos_ticks;
        s_weapon_clamp_force_zero_output = 1U;
        g_weapon_clamp_motor_dbg.pos_cmd = s_weapon_clamp_pos_cmd;
        g_weapon_clamp_motor_dbg.pos_err =
            s_weapon_clamp_pos_cmd - (float)weapon_clamp_motor.total_angle;
        g_weapon_clamp_motor_dbg.pid_input = s_weapon_clamp_pos_cmd;
        weapon_clamp_dbg_sync_motion();
        return;
    case WEAPON_CLAMP_POS_IDLE:
    default:
        s_weapon_clamp_force_zero_output = 1U;
        s_weapon_clamp_pos_cmd = (float)weapon_clamp_motor.total_angle;
        g_weapon_clamp_motor_dbg.pos_cmd = s_weapon_clamp_pos_cmd;
        g_weapon_clamp_motor_dbg.pos_err = 0.0f;
        g_weapon_clamp_motor_dbg.pid_input = s_weapon_clamp_pos_cmd;
        weapon_clamp_dbg_sync_motion();
        return;
    }

    pos_err = s_weapon_clamp_pos_cmd - (float)weapon_clamp_motor.total_angle;
    g_weapon_clamp_motor_dbg.pos_cmd = s_weapon_clamp_pos_cmd;
    g_weapon_clamp_motor_dbg.pos_err = pos_err;
    g_weapon_clamp_motor_dbg.pid_input = s_weapon_clamp_pos_cmd;
    g_weapon_clamp_motor_dbg.arrival_cnt = s_weapon_clamp_arrival_cnt;

    elapsed_ms = now_ms - s_weapon_clamp_move_start_ms;
    g_weapon_clamp_motor_dbg.move_start_ms = s_weapon_clamp_move_start_ms;
    g_weapon_clamp_motor_dbg.move_elapsed_ms = elapsed_ms;

    if (g_weapon_tune.clamp.move_timeout_ms > 0U &&
        elapsed_ms > g_weapon_tune.clamp.move_timeout_ms)
    {
        g_weapon_clamp_motor_dbg.move_fault = 1U;
        s_weapon_clamp_pos_fsm = WEAPON_CLAMP_POS_IDLE;
        s_weapon_clamp_force_zero_output = 1U;
        weapon_clamp_motor.pid_spd.Output = 0.0f;
        weapon_clamp_pid_clear();
        weapon_clamp_dbg_sync_motion();
        return;
    }

    if (weapon_clamp_pos_arrived(pos_err, rpm_abs) != 0U)
    {
        s_weapon_clamp_pos_fsm =
            (s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_GO_ZERO) ?
            WEAPON_CLAMP_POS_AT_ZERO : WEAPON_CLAMP_POS_AT_TARGET;
        s_weapon_clamp_force_zero_output = 1U;
        s_weapon_clamp_arrival_cnt = 0U;
        weapon_clamp_motor.pid_spd.Output = 0.0f;
        weapon_clamp_pid_clear();
        weapon_clamp_dbg_sync_motion();
        return;
    }

    s_weapon_clamp_force_zero_output = 0U;
    weapon_clamp_dbg_sync_motion();
}

void Weapon_ClampMotor_RunStep(void)
{
    weapon_clamp_pos_fsm_step();
    if (s_weapon_clamp_force_zero_output != 0U)
    {
        weapon_clamp_motor.pid_spd.Output = 0.0f;
    }
    else
    {
        weapon_clamp_motor.PID_Calculate(&weapon_clamp_motor, s_weapon_clamp_pos_cmd);
    }
    g_weapon_clamp_motor_dbg.pid_output = weapon_clamp_motor.pid_spd.Output;
}

float Weapon_ClampMotor_GetCanOutput(void)
{
    return weapon_clamp_motor.pid_spd.Output;
}

uint8_t Weapon_ClampMotor_AtOpenLimit(void)
{
    return (uint8_t)(s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_AT_ZERO);
}

uint8_t Weapon_ClampMotor_AtCloseLimit(void)
{
    return (uint8_t)(s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_AT_TARGET);
}

uint8_t Weapon_ClampMotor_IsBusy(void)
{
    return (uint8_t)((s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_GO_ZERO) ||
                     (s_weapon_clamp_pos_fsm == WEAPON_CLAMP_POS_GO_TARGET));
}

uint8_t Weapon_ClampPath_IsActive(void)
{
    if (control_mode == remote_control)
    {
        return (remote_mode == weapon_mode) ? 1U : 0U;
    }
    if ((control_mode == full_auto_control) && (app_flow_mode == app_flow_zone1))
    {
        return 1U;
    }
    return 0U;
}

void Weapon_Can2_PublishGuideOnly(void)
{
    DJIset_motor_data(&hfdcan2, 0X200,
                      guide_motor1.pid_spd.Output,
                      guide_motor2.pid_spd.Output,
                      0.0f,
                      0.0f);
}

void Weapon_Can2_PublishWithClamp(void)
{
    Weapon_ClampMotor_RunStep();
    DJIset_motor_data(&hfdcan2, 0X200,
                      guide_motor1.pid_spd.Output,
                      guide_motor2.pid_spd.Output,
                      0.0f,
                      Weapon_ClampMotor_GetCanOutput());
}

static void weapon_master_drive_by_bits(uint8_t action_bits)
{
    /* bit0 舵机状态 */
    servo_state = ((action_bits & MASTER_WEAPON_SERVO_BIT) != 0U) ? 0U : 1U;

    /* bit1: 夹爪状态 */
    clamp_state = ((action_bits & MASTER_WEAPON_CLAMP_BIT) != 0U) ? 1U : 0U;

    servo_use();
    clamp_use();

    /* bit2~bit5: 吸盘1~4状态 */
    sucker1_state = ((action_bits & MASTER_WEAPON_SUCKER1_BIT) != 0U) ? 1U : 0U;
    sucker2_state = ((action_bits & MASTER_WEAPON_SUCKER2_BIT) != 0U) ? 1U : 0U;
    sucker3_state = ((action_bits & MASTER_WEAPON_SUCKER3_BIT) != 0U) ? 1U : 0U;
    sucker4_state = ((action_bits & MASTER_WEAPON_SUCKER4_BIT) != 0U) ? 1U : 0U;

    // 吸盘1和吸盘2联动
    pump1_two_suckers_linkage_nominal_open((uint8_t)(sucker1_state & 0x01U), (uint8_t)(sucker2_state & 0x01U));
    // 吸盘3和吸盘4联动
    pump2_two_suckers_linkage_nominal_open((uint8_t)(sucker3_state & 0x01U), (uint8_t)(sucker4_state & 0x01U));
}




/**
  * @brief 手动武器功能
  */
void manual_weapon_function(void)
{
    /* master 主武器控制（见 Motion_Task.h 手动武器功能） */
    // if (control_mode == master_control)
    // {
    //     weapon_master_drive_by_bits(master_weapon_action_bits);
    //     return;
    // }

    /* 远程控制 */
    if(control_mode == remote_control)
    {
        if (RCctrl.CH3 >=1500)
        {
        servo_use();
        }
        if (RCctrl.CH3<=500)
        {
        clamp_use();
        }
        if (RCctrl.CH2>=1500)
        {
        sucker1_use();
        }
        if (RCctrl.CH1<=500)
        {
        sucker2_use();
        }
        if (RCctrl.CH1>=1500)
        {
        sucker3_use();
        }
        if (RCctrl.CH2<=500)
        {
        sucker4_use();
        }
    }
    else if (control_mode == full_auto_control)
    {
        if (app_flow_mode == app_flow_zone1)
        {
            ClampHeadCtrl_Run();
        }
        else
        {
            servo_use();
        }
        sucker1_use();
        sucker2_use();
        sucker3_use();
        sucker4_use();
    }
    else
    {
        servo_state = 0;
        clamp_state = 0;
        sucker1_state = 0;
        sucker2_state = 0;
        sucker3_state = 0;
        sucker4_state = 0;
    }

    if (Weapon_ClampPath_IsActive() != 0U)
    {
        Weapon_Can2_PublishWithClamp();
    }

}



/**
* @brief 舵机使用
  */
void servo_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            servo_state ^= 1; // 舵机状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }   
    }
    if ((servo_state % 2U) == 0U)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)g_weapon_tune.servo.pwm_mid);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)g_weapon_tune.servo.pwm_upright);
    }
}

/**
  * @brief 夹爪使用
  */
void clamp_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            clamp_state ^= 1U;
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }

    Weapon_ClampMotor_SetTarget(clamp_state);
}

/**
  * @brief 吸盘1使用
  */
void sucker1_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            sucker1_state ^= 1; // 吸盘1状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, sucker1_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief 吸盘2使用
  */
void sucker2_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            sucker2_state ^= 1; // 吸盘2状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, sucker2_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

    /**
  * @brief 吸盘3使用
  */
void sucker3_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            sucker3_state ^= 1; // 吸盘3状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, sucker3_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief 吸盘4使用
  */
void sucker4_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            sucker4_state ^= 1; // 吸盘4状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, sucker4_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
