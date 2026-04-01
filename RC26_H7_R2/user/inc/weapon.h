#ifndef __WEAPON_H__
#define __WEAPON_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"


typedef struct _Weapon_Module{
    StructureModule super_struct; 
    
                             
} Weapon_Module;





extern uint8_t CH5_trigger_flag1; //¶æ»ú
extern uint8_t CH5_trigger_flag0; //Æø±Ã

void streeing_motion(void);
void pump_motion(void);


#endif
