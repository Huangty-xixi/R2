#ifndef __MOTION_TASK_H__
#define __MOTION_TASK_H__

#include <stdint.h>
#include "lift.h"

//typedef enum{
//    weapon_none,
//  pump_mode,
//	steering_mode
//}Weapon_mode;

typedef enum{
    remote_control,//0
    emergency_stop_mode,//1
    full_auto_control,//2  CH8 中档：全自动（原半自动档）
}Control_mode;                                                              

typedef enum{
    remote_none,//0
    chassis_mode,//1
    weapon_mode,//2
    lift_mode,//3
    kfs_mode,//4
}Remote_mode;

typedef enum{
    full_auto_none,//0
    full_auto_upstairs_mode,//1
    full_auto_downstairs_mode,//2
    full_auto_get_kfs_mode,//3
    full_auto_put_kfs_mode,//4
    full_auto_upslope_mode,//5
    full_auto_zone2_mode,//6  CH6 最大：二区 app_zone2_poll（与放/取/上坡互斥）
    full_auto_zone1_clamp_head_mode,//7 一区夹枪头（AppFlowDispatch）
}Full_auto_mode;

// typedef enum{
//     master_none = 0,      // 无主控动作
//     master_chassis_mode,   // 主控-底盘
//     master_weapon_mode,    // 主控-武器
//     master_lift_mode,      // 主控-抬升
//     master_kfs_mode,       // 主控-kfs
// }Master_mode;

// /* master并行使能位定义（data[0]） */
// #define MASTER_EN_CHASSIS   (1U << 0)
// #define MASTER_EN_WEAPON    (1U << 1)
// #define MASTER_EN_LIFT      (1U << 2)
// #define MASTER_EN_KFS       (1U << 3)

extern Control_mode control_mode;
extern Remote_mode remote_mode;
extern Full_auto_mode full_auto_mode;
// extern Master_mode master_mode;
// extern uint8_t master_enable_bits;
// extern uint8_t master_chassis_action_bits_0;
// extern uint8_t master_chassis_action_bits_1;
// extern uint8_t master_weapon_action_bits;
// extern uint8_t master_lift_action_bits;
// extern uint8_t master_kfs_action_bits_0;
// extern uint8_t master_kfs_action_bits_1;
#endif
