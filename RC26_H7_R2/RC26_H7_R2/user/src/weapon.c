#include "weapon.h"
#include "remote_control.h"
#include "Motion_Task.h"
#include "clamp_head_ctrl.h"
#include "main.h"
#include "tim.h"
#include "chassis.h"
#include "dji_motor.h"
#include <stdlib.h>



DJI_MotorModule weapon_clamp_motor;
float weapon_clamp_motor_pid_param[PID_PARAMETER_NUM] = {5.0f, 0.4f, 0.2f, 1, 500.0f, 10000.0f};
volatile weapon_tune_t g_weapon_tune = {
    .clamp = {
        .run_thr_rpm = 10,
        .stop_thr_rpm = 10,
        .stop_cnt_max = 1U,
        .cmd_open_pwm = -1000.0f,
        .cmd_close_pwm = 1000.0f,
        .hold_idle_pwm = 0.0f,
        .hold_open_pwm = 0.0f,
        .hold_close_pwm = 0.0f,
        .open_rounds   = 20.0f,
        .close_rounds  = 20.0f,
    },
    .servo = {
        .pwm_mid = 1300U,
        .pwm_upright = 2150U,
    },
};
volatile weapon_clamp_motor_dbg_t g_weapon_clamp_motor_dbg;

static float s_weapon_clamp_pid_input;
static uint8_t s_weapon_clamp_seen_move;
static uint8_t s_weapon_clamp_stop_cnt;

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
                    SPEED,
                    PID_POSITION,
                    weapon_clamp_motor_pid_param);
    Weapon_ClampMotor_Reset();
}

void Weapon_ClampMotor_Reset(void)
{
    s_weapon_clamp_pid_input = g_weapon_tune.clamp.hold_idle_pwm;
    s_weapon_clamp_seen_move = 0U;
    s_weapon_clamp_stop_cnt = 0U;
    g_weapon_clamp_motor_dbg.target = WEAPON_CLAMP_TARGET_OPEN;
    g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_AT_OPEN;
    g_weapon_clamp_motor_dbg.pid_input = g_weapon_tune.clamp.hold_idle_pwm;
    g_weapon_clamp_motor_dbg.cmd_pwm = 0.0f;
    g_weapon_clamp_motor_dbg.hold_pwm = g_weapon_tune.clamp.hold_open_pwm;
    g_weapon_clamp_motor_dbg.pid_output = 0.0f;
    g_weapon_clamp_motor_dbg.speed_rpm = 0;
    g_weapon_clamp_motor_dbg.speed_rpm_abs = 0;
    g_weapon_clamp_motor_dbg.seen_move = 0U;
    g_weapon_clamp_motor_dbg.stop_cnt = 0U;
    g_weapon_clamp_motor_dbg.at_open_limit = 1U;
    g_weapon_clamp_motor_dbg.at_close_limit = 0U;
    g_weapon_clamp_motor_dbg.is_busy = 0U;
    g_weapon_clamp_motor_dbg.step_tick = 0U;
    g_weapon_clamp_motor_dbg.round_start = 0;
    g_weapon_clamp_motor_dbg.round_cur = 0;
    g_weapon_clamp_motor_dbg.round_delta = 0;
    weapon_clamp_motor.pid_spd.Output = 0.0f;
}

void Weapon_ClampMotor_SetTarget(uint8_t close)
{
    weapon_clamp_target_t tgt = (close != 0U) ? WEAPON_CLAMP_TARGET_CLOSE : WEAPON_CLAMP_TARGET_OPEN;

    g_weapon_clamp_motor_dbg.target = tgt;

    if (tgt == WEAPON_CLAMP_TARGET_CLOSE)
    {
        if (g_weapon_clamp_motor_dbg.motion != WEAPON_CLAMP_MOTION_AT_CLOSE)
        {
            g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_CLOSING;
            s_weapon_clamp_seen_move = 0U;
            s_weapon_clamp_stop_cnt = 0U;
            g_weapon_clamp_motor_dbg.round_start = weapon_clamp_motor.round_cnt;
        }
    }
    else
    {
        if (g_weapon_clamp_motor_dbg.motion != WEAPON_CLAMP_MOTION_AT_OPEN)
        {
            g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_OPENING;
            s_weapon_clamp_seen_move = 0U;
            s_weapon_clamp_stop_cnt = 0U;
            g_weapon_clamp_motor_dbg.round_start = weapon_clamp_motor.round_cnt;
        }
    }
}

static float weapon_clamp_motor_hold_pwm_get(weapon_clamp_motion_t motion)
{
    if (motion == WEAPON_CLAMP_MOTION_AT_OPEN)
    {
        return g_weapon_tune.clamp.hold_open_pwm;
    }
    if (motion == WEAPON_CLAMP_MOTION_AT_CLOSE)
    {
        return g_weapon_tune.clamp.hold_close_pwm;
    }
    return g_weapon_tune.clamp.hold_idle_pwm;
}

static void weapon_clamp_motor_dbg_refresh_limits(void)
{
    weapon_clamp_motion_t motion = g_weapon_clamp_motor_dbg.motion;

    g_weapon_clamp_motor_dbg.at_open_limit =
        (uint8_t)(motion == WEAPON_CLAMP_MOTION_AT_OPEN);
    g_weapon_clamp_motor_dbg.at_close_limit =
        (uint8_t)(motion == WEAPON_CLAMP_MOTION_AT_CLOSE);
    g_weapon_clamp_motor_dbg.is_busy =
        (uint8_t)((motion == WEAPON_CLAMP_MOTION_OPENING) ||
                  (motion == WEAPON_CLAMP_MOTION_CLOSING));
}

static void weapon_clamp_motor_motion_step(void)
{
    int rpm_abs = abs((int)weapon_clamp_motor.speed_rpm);
    float hold_pwm;

    g_weapon_clamp_motor_dbg.speed_rpm = weapon_clamp_motor.speed_rpm;
    g_weapon_clamp_motor_dbg.speed_rpm_abs = (int16_t)rpm_abs;
    g_weapon_clamp_motor_dbg.seen_move = s_weapon_clamp_seen_move;
    g_weapon_clamp_motor_dbg.stop_cnt = s_weapon_clamp_stop_cnt;
    g_weapon_clamp_motor_dbg.step_tick = HAL_GetTick();

    switch (g_weapon_clamp_motor_dbg.motion)
    {
    case WEAPON_CLAMP_MOTION_OPENING:
        s_weapon_clamp_pid_input = g_weapon_tune.clamp.cmd_open_pwm;
        g_weapon_clamp_motor_dbg.cmd_pwm = g_weapon_tune.clamp.cmd_open_pwm;

        g_weapon_clamp_motor_dbg.round_cur = weapon_clamp_motor.round_cnt;
        g_weapon_clamp_motor_dbg.round_delta = (int32_t)(labs((long)(g_weapon_clamp_motor_dbg.round_cur - g_weapon_clamp_motor_dbg.round_start)));

        if ((float)g_weapon_clamp_motor_dbg.round_delta >= g_weapon_tune.clamp.open_rounds)
        {
            g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_AT_OPEN;
            s_weapon_clamp_stop_cnt = 0U;
            break;
        }

        if (rpm_abs > (int)g_weapon_tune.clamp.run_thr_rpm)
        {
            s_weapon_clamp_seen_move = 1U;
        }
        if ((s_weapon_clamp_seen_move != 0U) && (rpm_abs < (int)g_weapon_tune.clamp.stop_thr_rpm))
        {
            if (++s_weapon_clamp_stop_cnt >= g_weapon_tune.clamp.stop_cnt_max)
            {
                g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_AT_OPEN;
                s_weapon_clamp_stop_cnt = 0U;
            }
        }
        else
        {
            s_weapon_clamp_stop_cnt = 0U;
        }
        break;
    case WEAPON_CLAMP_MOTION_CLOSING:
        s_weapon_clamp_pid_input = g_weapon_tune.clamp.cmd_close_pwm;
        g_weapon_clamp_motor_dbg.cmd_pwm = g_weapon_tune.clamp.cmd_close_pwm;

        g_weapon_clamp_motor_dbg.round_cur = weapon_clamp_motor.round_cnt;
        g_weapon_clamp_motor_dbg.round_delta = (int32_t)(labs((long)(g_weapon_clamp_motor_dbg.round_cur - g_weapon_clamp_motor_dbg.round_start)));

        if ((float)g_weapon_clamp_motor_dbg.round_delta >= g_weapon_tune.clamp.close_rounds)
        {
            g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_AT_CLOSE;
            s_weapon_clamp_stop_cnt = 0U;
            break;
        }

        if (rpm_abs > (int)g_weapon_tune.clamp.run_thr_rpm)
        {
            s_weapon_clamp_seen_move = 1U;
        }
        if ((s_weapon_clamp_seen_move != 0U) && (rpm_abs < (int)g_weapon_tune.clamp.stop_thr_rpm))
        {
            if (++s_weapon_clamp_stop_cnt >= g_weapon_tune.clamp.stop_cnt_max)
            {
                g_weapon_clamp_motor_dbg.motion = WEAPON_CLAMP_MOTION_AT_CLOSE;
                s_weapon_clamp_stop_cnt = 0U;
            }
        }
        else
        {
            s_weapon_clamp_stop_cnt = 0U;
        }
        break;
    case WEAPON_CLAMP_MOTION_AT_OPEN:
    case WEAPON_CLAMP_MOTION_AT_CLOSE:
    default:
        break;
    }

    hold_pwm = weapon_clamp_motor_hold_pwm_get(g_weapon_clamp_motor_dbg.motion);
    if ((g_weapon_clamp_motor_dbg.motion == WEAPON_CLAMP_MOTION_AT_OPEN) ||
        (g_weapon_clamp_motor_dbg.motion == WEAPON_CLAMP_MOTION_AT_CLOSE))
    {
        s_weapon_clamp_pid_input = hold_pwm;
    }

    g_weapon_clamp_motor_dbg.hold_pwm = hold_pwm;
    g_weapon_clamp_motor_dbg.pid_input = s_weapon_clamp_pid_input;
    weapon_clamp_motor_dbg_refresh_limits();
}

void Weapon_ClampMotor_RunStep(void)
{
    weapon_clamp_motor_motion_step();
    weapon_clamp_motor.PID_Calculate(&weapon_clamp_motor, s_weapon_clamp_pid_input);
    g_weapon_clamp_motor_dbg.pid_output = weapon_clamp_motor.pid_spd.Output;
}

float Weapon_ClampMotor_GetCanOutput(void)
{
    return weapon_clamp_motor.pid_spd.Output;
}

uint8_t Weapon_ClampMotor_AtOpenLimit(void)
{
    return (uint8_t)(g_weapon_clamp_motor_dbg.motion == WEAPON_CLAMP_MOTION_AT_OPEN);
}

uint8_t Weapon_ClampMotor_AtCloseLimit(void)
{
    return (uint8_t)(g_weapon_clamp_motor_dbg.motion == WEAPON_CLAMP_MOTION_AT_CLOSE);
}

uint8_t Weapon_ClampMotor_IsBusy(void)
{
    return (uint8_t)((g_weapon_clamp_motor_dbg.motion == WEAPON_CLAMP_MOTION_OPENING) ||
                     (g_weapon_clamp_motor_dbg.motion == WEAPON_CLAMP_MOTION_CLOSING));
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
