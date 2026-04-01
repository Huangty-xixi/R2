#ifndef __MOTION_TASK_H__
#define __MOTION_TASK_H__



typedef enum{
    weapon_none,
  pump_mode,
	steering_mode
}Weapon_mode;

typedef enum{
    remote_control,
    master_control
}Control_mode;                                                              

typedef enum{
    remote_none,
    chassis_move,
    weapon_switch,
}Remote_mode;





extern Weapon_mode weapon_mode;
extern Control_mode control_mode;
extern Remote_mode remote_mode;

#endif
