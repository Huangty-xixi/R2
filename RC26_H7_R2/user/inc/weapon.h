#ifndef __WEAPON_H__
#define __WEAPON_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"


extern uint8_t CH5_trigger_flag0;//���
extern uint8_t CH5_trigger_flag1;//����

void weapon_init(void);
void steering_use(void);
void pump_use(void);
void manual_weapon_function(void);



#endif
