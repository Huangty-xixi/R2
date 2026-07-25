#include "global.h"


void BaseModule_Init(baseModule *base) {
    base->state = MODULE_READY;
    base->error_code = 0;
}

void BaseModule_Stop(baseModule *base) {
    base->state = MODULE_READY;
}

void BaseModule_Run(baseModule *base) {
    base->state = MODULE_RUNNING;
}
ModuleState BaseModule_GetState(baseModule *base) {
    return base->state;
}

void BaseModule_ClearError(baseModule *base) {
    base->error_code = 0;
    base->state = MODULE_READY;
}
