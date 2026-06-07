/**
 * @file weapon.h
 * @brief 武器模块
 */
#ifndef __WEAPON_H__
#define __WEAPON_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"

/************************ 夹爪电机ID：CAN2 0x200 4号电机 *************************/
#define WEAPON_CLAMP_MOTOR_ID           0x04U
#define WEAPON_CLAMP_MOTOR_CMD_ID       0x200U
#define WEAPON_CLAMP_MOTOR_FEEDBACK_ID  (0x200U + WEAPON_CLAMP_MOTOR_ID)

/** 夹爪 M2006 两点位控在线调参（Keil Watch：g_weapon_tune.clamp） */
typedef struct
{
    float close_pos_ticks;       /* 位置2：关爪目标 total_angle（正数 tick） */
    float pos_tol_ticks;         /* 到位位置误差阈值 */
    int16_t arrival_rpm_thr;     /* 到位转速阈值（绝对值） */
    uint8_t arrival_confirm_cnt; /* 连续到位确认周期数 */
    uint32_t move_timeout_ms;    /* 单次运动超时 */
} weapon_clamp_tune_t;

/** 舵机 PWM 在线调参（TIM2 CH1） */
typedef struct
{
    uint16_t pwm_mid;
    uint16_t pwm_upright;
} weapon_servo_tune_t;

/** 武器层在线调参总表（Keil Watch：g_weapon_tune） */
typedef struct
{
    weapon_clamp_tune_t clamp;
    weapon_servo_tune_t servo;
} weapon_tune_t;

/** 夹爪电机目标：0=张开 1=夹紧 */
typedef enum
{
    WEAPON_CLAMP_TARGET_OPEN = 0,
    WEAPON_CLAMP_TARGET_CLOSE = 1,
} weapon_clamp_target_t;

/** 夹爪两点位控 FSM（内部；dbg 镜像到 motion 兼容旧 Watch） */
typedef enum
{
    WEAPON_CLAMP_POS_IDLE = 0,
    WEAPON_CLAMP_POS_GO_ZERO,
    WEAPON_CLAMP_POS_GO_TARGET,
    WEAPON_CLAMP_POS_AT_ZERO,
    WEAPON_CLAMP_POS_AT_TARGET,
} weapon_clamp_pos_fsm_t;

/** 兼容 dbg：与 pos_fsm 同步的 motion 枚举 */
typedef enum
{
    WEAPON_CLAMP_MOTION_AT_OPEN = 0,
    WEAPON_CLAMP_MOTION_OPENING,
    WEAPON_CLAMP_MOTION_AT_CLOSE,
    WEAPON_CLAMP_MOTION_CLOSING,
} weapon_clamp_motion_t;

/** Keil Watch：夹爪电机运行快照 */
typedef struct
{
    weapon_clamp_target_t target;
    weapon_clamp_motion_t motion;
    weapon_clamp_pos_fsm_t pos_fsm;
    float pos_cmd;
    float total_angle;
    float pos_err;
    float pid_input;
    float pid_output;
    int16_t speed_rpm;
    int16_t speed_rpm_abs;
    uint8_t arrival_cnt;
    uint8_t at_open_limit;
    uint8_t at_close_limit;
    uint8_t is_busy;
    uint8_t move_fault;
    uint32_t step_tick;
    uint32_t move_start_ms;
    uint32_t move_elapsed_ms;
} weapon_clamp_motor_dbg_t;

extern uint8_t servo_state;
extern uint8_t clamp_state;
extern uint8_t sucker1_state;
extern uint8_t sucker2_state;
extern uint8_t sucker3_state;
extern uint8_t sucker4_state;

extern DJI_MotorModule weapon_clamp_motor;
extern float weapon_clamp_motor_pos_pid_param[PID_PARAMETER_NUM];
extern float weapon_clamp_motor_spd_pid_param[PID_PARAMETER_NUM];
extern volatile weapon_tune_t g_weapon_tune;
extern volatile weapon_clamp_motor_dbg_t g_weapon_clamp_motor_dbg;

void weapon_init(void);
void servo_use(void);
void clamp_use(void);
void sucker1_use(void);
void sucker2_use(void);
void sucker3_use(void);
void sucker4_use(void);
void manual_weapon_function(void);
void pump1_two_suckers_linkage_nominal_open(uint8_t sucker1_on, uint8_t sucker2_on);
void pump2_two_suckers_linkage_nominal_open(uint8_t sucker3_on, uint8_t sucker4_on);

void Weapon_ClampMotor_Init(void);
void Weapon_ClampMotor_Reset(void);
void Weapon_ClampMotor_GoToZero(void);
void Weapon_ClampMotor_GoToTarget(void);
void Weapon_ClampMotor_SetTarget(uint8_t close);
void Weapon_ClampMotor_RunStep(void);
float Weapon_ClampMotor_GetCanOutput(void);

uint8_t Weapon_ClampMotor_AtOpenLimit(void);
uint8_t Weapon_ClampMotor_AtCloseLimit(void);
uint8_t Weapon_ClampMotor_IsBusy(void);

uint8_t Weapon_ClampPath_IsActive(void);
void Weapon_Can2_PublishGuideOnly(void);
void Weapon_Can2_PublishWithClamp(void);

#endif /* __WEAPON_H__ */
