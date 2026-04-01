#include "structure.h"

static void StructureModule_Init(baseModule *base) {
    StructureModule *Structure = (StructureModule *)base;
    BaseModule_Init(base); // 调用基类初始化
    
    base->type = structure;
}

void StructureModule_Run(baseModule *base)
{
    StructureModule *Structure = (StructureModule*)base;
    BaseModule_Run(base);
}

static void StructureModule_AddMotor(StructureModule *structure, MotorModule *motor) {
    if (structure == NULL || motor == NULL) {
        structure->base.error_code = 0x01; //空指针错误
        return;
    }
    if (structure->motor_num >= MAX_MOTOR) {
        structure->base.error_code = 0x02; // 电机数超出上限
        return;
    }

    for (uint8_t i = 0; i < structure->motor_num; i++) {
        if (structure->motors[i] == motor || structure->motor_ids[i] == motor->id) {
            structure->base.error_code = 0x03; // 电机已存在
            return;
        }
    }

    structure->motors[structure->motor_num] = motor;
    structure->motor_ids[structure->motor_num] = motor->id;
    structure->motor_num++; 

    motor->base.Init(&motor->base);
    structure->base.error_code = 0;
}


void StructureModule_Create(StructureModule *obj, strctureName name) {
    obj->name = name;
    
    obj->base.Init = StructureModule_Init;
    obj->base.Run = StructureModule_Run;
    obj->base.Stop = BaseModule_Stop;
    obj->base.GetState = BaseModule_GetState;
    obj->base.ClearError = BaseModule_ClearError;
    
    obj->AddMotor = StructureModule_AddMotor;
    
    for (uint8_t i = 0; i < MAX_MOTOR; i++) {
        obj->motors[i] = NULL;    
        obj->motor_ids[i] = 0;      
    }
    obj->motor_num = 0;
}
