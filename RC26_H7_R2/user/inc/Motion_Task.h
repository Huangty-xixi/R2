#ifndef __MOTION_TASK_H__
#define __MOTION_TASK_H__

#include <stdint.h>

//typedef enum{
//    weapon_none,
//  pump_mode,
//	steering_mode
//}Weapon_mode;

typedef enum{
    remote_control,
		part_remote_control,
    master_control,
}Control_mode;                                                              

typedef enum{
    remote_none,
    chassis_mode,
    weapon_mode,
		lift_mode,
		kfs_mode,
}Remote_mode;

typedef enum{
    raise,
    fall,
}R2_lift_mode;





extern Control_mode control_mode;
extern Remote_mode remote_mode;
extern R2_lift_mode r2_lift_mode;


#endif
