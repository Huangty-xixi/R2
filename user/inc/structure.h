#ifndef __STRUCTURE_H__
#define __STRUCTURE_H__

#include "global.h"
#include "motor.h"

#define  MAX_MOTOR  6

typedef enum{
    chassis,            
    weapon,             
    kfs,                
    lift,               
}strctureName;

typedef struct _StructureModule{
    baseModule base;      
    
    strctureName name;
    uint8_t motor_num;
    
    MotorModule *motors[MAX_MOTOR]; 
    uint8_t motor_ids[MAX_MOTOR];   
    
    void (*Init)(struct _StructureModule *mech, MotorModule *motor); 
    void (*AddMotor)(struct _StructureModule *mech, MotorModule *motor); 
    void (*RemoveMotor)(struct _StructureModule *mech, uint8_t motor_id); 
    void (*ControlAllMotors)(struct _StructureModule *mech, uint8_t speed, int16_t dir); 
}StructureModule;

void StructureModule_Create(StructureModule *obj, strctureName name);

#endif
