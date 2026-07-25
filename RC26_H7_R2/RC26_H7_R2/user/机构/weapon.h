/**
 * @file weapon.h
 * @brief 武器模块
 */
#ifndef __WEAPON_H__
#define __WEAPON_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"

/** 舵机 PWM 在线调参（TIM2 CH1） */
typedef struct
{
    uint16_t pwm_mid;
    uint16_t pwm_upright;
} weapon_servo_tune_t;

/** 武器层在线调参总表（Keil Watch：g_weapon_tune） */
typedef struct
{
    weapon_servo_tune_t servo;
} weapon_tune_t;

extern uint8_t servo_state;
extern uint8_t clamp_state;
extern uint8_t sucker1_state;
extern uint8_t sucker2_state;
extern uint8_t sucker3_state;
extern uint8_t sucker4_state;

extern volatile weapon_tune_t g_weapon_tune;

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

#endif /* __WEAPON_H__ */
