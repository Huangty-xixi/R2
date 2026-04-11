#ifndef __MOTION_TASK_H__
#define __MOTION_TASK_H__

#include <stdint.h>

//typedef enum{
//    weapon_none,
//  pump_mode,
//	steering_mode
//}Weapon_mode;

typedef enum{
    remote_control,//0
		part_remote_control,//1
    master_control,//2
}Control_mode;                                                              

typedef enum{
    remote_none,//0
    chassis_mode,//1
    weapon_mode,//2
		lift_mode,//3
		kfs_mode,//4
}Remote_mode;

typedef enum{
    raise,//0
    fall,//1
}R2_lift_mode;





extern Control_mode control_mode;
extern Remote_mode remote_mode;
extern R2_lift_mode r2_lift_mode;


#endif
