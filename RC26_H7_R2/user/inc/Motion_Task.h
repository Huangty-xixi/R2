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
    master_control
}Control_mode;                                                              

typedef enum{
    remote_none,
    chassis_mode,
    weapon_mode,
		lift_mode,
		kfs_mode
}Remote_mode;





extern Control_mode control_mode;
extern Remote_mode remote_mode;

// 手动模式下的 4 种子模式标志位（由 CH6/CH7 00/01/10/11 编码）
// 其中 CH8 最大 -> 使能手动模式；非手动时建议所有子模式标志为 0
//extern uint8_t manual_mode_flag;      // CH8 最大：1=手动，0=非手动
//extern uint8_t manual_chassis_flag;   // CH6/CH7=00：底盘模式
//extern uint8_t manual_weapon_flag;    // CH6/CH7=01：武器模式（此处先映射 steering_use）
//extern uint8_t manual_lift_flag;      // CH6/CH7=10：抬升模式（占位）
//extern uint8_t manual_kfs_flag;       // CH6/CH7=11：KFS模式（占位）

#endif
