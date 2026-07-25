#ifndef __MOTION_TASK_H__
#define __MOTION_TASK_H__

#include <stdint.h>

typedef enum{
    remote_control,//0
    emergency_stop_mode,//1
}Control_mode;

typedef enum{
    remote_none,//0
    chassis_mode,//1
    weapon_mode,//2
    lift_mode,//3
    kfs_mode,//4
}Remote_mode;

extern Control_mode control_mode;
extern Remote_mode remote_mode;

#endif
