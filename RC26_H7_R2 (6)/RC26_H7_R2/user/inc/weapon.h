#ifndef __WEAPON_H__
#define __WEAPON_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"

extern uint8_t servo_state;    // 舵机状态
extern uint8_t clamp_state;     // 夹爪开合
extern uint8_t sucker1_state;     // 吸盘1开合
extern uint8_t sucker2_state;     // 吸盘2开合
extern uint8_t sucker3_state;     // 吸盘3开合
extern uint8_t sucker4_state;     // 吸盘4开合
extern uint8_t pump1_state;     // 泵1开合
extern uint8_t pump2_state;     // 泵2开合

void weapon_init(void);
void servo_use(void);
void clamp_use(void);
void sucker1_use(void);
void sucker2_use(void);
void sucker3_use(void);
void sucker4_use(void);
void manual_weapon_function(void);
void pump1_use(void);
void pump2_use(void);
void pump1_two_suckers_linkage(uint8_t sucker1_on, uint8_t sucker2_on);
void pump2_two_suckers_linkage(uint8_t sucker3_on, uint8_t sucker4_on);

#endif
