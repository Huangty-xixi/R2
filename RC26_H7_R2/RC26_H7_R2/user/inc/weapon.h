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

/** 夹爪电机在线调参 */
typedef struct
{
    int16_t run_thr_rpm;                        
    int16_t stop_thr_rpm;
    uint8_t stop_cnt_max;
    float cmd_open_pwm;
    float cmd_close_pwm;
    float hold_idle_pwm;
    float hold_open_pwm;
    float hold_close_pwm;
    float open_rounds;   /* 张开到位：round_cnt 变化量阈值（圈） */
    float close_rounds;  /* 夹紧到位：round_cnt 变化量阈值（圈） */
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

/** 夹爪电机运行/到位状态（转速软限位） */
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
    float pid_input;
    float cmd_pwm;
    float hold_pwm;
    float pid_output;
    int16_t speed_rpm;
    int16_t speed_rpm_abs;
    uint8_t seen_move;
    uint8_t stop_cnt;
    uint8_t at_open_limit;
    uint8_t at_close_limit;
    uint8_t is_busy;
    uint32_t step_tick;
    int32_t round_start;  /* SetTarget 时 round_cnt 快照 */
    int32_t round_cur;    /* 当前 round_cnt */
    int32_t round_delta;  /* |round_cur - round_start| */
} weapon_clamp_motor_dbg_t;

extern uint8_t servo_state;
extern uint8_t clamp_state;
extern uint8_t sucker1_state;
extern uint8_t sucker2_state;
extern uint8_t sucker3_state;
extern uint8_t sucker4_state;

extern DJI_MotorModule weapon_clamp_motor;
extern float weapon_clamp_motor_pid_param[PID_PARAMETER_NUM];
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
