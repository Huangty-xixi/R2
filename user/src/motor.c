#include "motor.h"

static void MotorModule_Init(baseModule *base) {
    MotorModule *Motor = (MotorModule *)base;
    BaseModule_Init(base); // 调用基类初始化
    
    base->type = motor;
}

void MotorModule_Run(baseModule *base)
{
    MotorModule *Motor = (MotorModule*)base;
}

void MotorModule_Create(MotorModule *obj, uint8_t motor_id, FDCAN_HandleTypeDef *hcan, Motor_Model model, Ctrl_mode mode) {
    obj->model = model;
    obj->mode = mode;
    obj->id = motor_id;
    
    obj->base.Init = MotorModule_Init;
    obj->base.Run = MotorModule_Run;
    obj->base.Stop = BaseModule_Stop;
    obj->base.GetState = BaseModule_GetState;
    obj->base.ClearError = BaseModule_ClearError;
    
    obj->hcan = *hcan;
}
