#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "main.h"

typedef enum {
    MODULE_UNINIT = 0,   // 未初始化
    MODULE_READY,        // 就绪
    MODULE_RUNNING,      // 运行中
    MODULE_ERROR         // 故障
} ModuleState;

typedef enum{
    structure,
    motor,
    sensor,
    communication,
}ModuleType;

typedef struct _baseModule{
    ModuleType     type;
    ModuleState    state;     
    uint32_t       error_code;  
    
    void (*Init)(struct _baseModule *base);     
    void (*Run)(struct _baseModule *base);      
    void (*Stop)(struct _baseModule *base);    
    ModuleState (*GetState)(struct _baseModule *base);
    void (*ClearError)(struct _baseModule *base); 
}baseModule;

void BaseModule_Init(baseModule *self);
void BaseModule_Stop(baseModule *self);
void BaseModule_Run(baseModule *self);
ModuleState BaseModule_GetState(baseModule *self);
void BaseModule_ClearError(baseModule *self);


#endif
