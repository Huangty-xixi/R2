#ifndef __MOTION_TASK_H__
#define __MOTION_TASK_H__

#include <stdint.h>
#include "lift.h"

typedef enum{
    remote_control,//0
    emergency_stop_mode,//1
    full_auto_control,//2  CH8 mid: full-auto (CH8)
}Control_mode;

typedef enum{
    remote_none,//0
    chassis_mode,//1
    weapon_mode,//2
    lift_mode,//3
    kfs_mode,//4
}Remote_mode;

/** CH5/CH7 etc.; zone1/2/3 use App_flow_mode */
typedef enum{
    flow_none = 0,
    flow_upstairs_mode,
    flow_downstairs_mode,
    flow_get_kfs_mode,
    flow_put_kfs_mode,
    flow_upslope_mode,
} Flow_mode;

typedef enum {
    app_flow_none = 0,
    app_flow_zone1,
    app_flow_zone2,
    app_flow_zone3,
} App_flow_mode;

extern Control_mode control_mode;
extern Remote_mode remote_mode;
extern Flow_mode flow_mode;
extern App_flow_mode app_flow_mode;

#endif
