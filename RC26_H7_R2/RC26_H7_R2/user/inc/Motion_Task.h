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
    full_auto_control,//2  CH8 ÖÐµµ£ºÈ«×Ô¶¯£¨Ô­°ë×Ô¶¯µµ£©
}Control_mode;                                                              

typedef enum{
    remote_none,//0
    chassis_mode,//1
    weapon_mode,//2
    lift_mode,//3
    kfs_mode,//4
}Remote_mode;

typedef enum{
<<<<<<< HEAD
    full_auto_none,//0
    full_auto_upstairs_mode,//1
    full_auto_downstairs_mode,//2
    full_auto_get_kfs_mode,//3
    full_auto_put_kfs_mode,//4
    full_auto_face_field_dir_mode,//5
    full_auto_upslope_mode,//6
    full_auto_zone2_mode,//7  CH6 ×î´ó£º¶þÇø app_zone2_poll£¨Óë·Å/È¡/ÉÏÆÂ»¥³â£©
}Full_auto_mode;
=======
    semi_auto_none,//0
    semi_auto_upstairs_mode,//1
    semi_auto_downstairs_mode,//2
    semi_auto_get_kfs_mode,//3
//    semi_auto_put_kfs_mode,//4
    semi_auto_zone1_clamp_head_mode,//4
}Semi_auto_mode;
>>>>>>> origin/1åŒº

// typedef enum{
//     master_none = 0,      // ÎÞÖ÷¿Ø¶¯×÷
//     master_chassis_mode,   // Ö÷¿Ø-µ×ÅÌ
//     master_weapon_mode,    // Ö÷¿Ø-ÎäÆ÷
//     master_lift_mode,      // Ö÷¿Ø-Ì§Éý
//     master_kfs_mode,       // Ö÷¿Ø-kfs
// }Master_mode;

// /* master²¢ÐÐÊ¹ÄÜÎ»¶¨Òå£¨data[0]£© */
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
