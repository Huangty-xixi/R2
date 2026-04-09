#ifndef __KFS_H__
#define __KFS_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"

/***********************************************/
#define MAIN_LIFT_ID               0x03
#define MAIN_LIFT_CMD_ID           MAIN_LIFT_ID
#define MAIN_LIFT_FEEDBACK_ID      MAIN_LIFT_ID
#define MAIN_LIFT_MASTER_ID        0x10

#define KFS_SPIN_ID                0x04
#define KFS_SPIN_CMD_ID            KFS_SPIN_ID
#define KFS_SPIN_FEEDBACK_ID       KFS_SPIN_ID
#define KFS_SPIN_MASTER_ID         0x10

#define THREE_KFS_ID               0x05
#define THREE_KFS_CMD_ID           THREE_KFS_ID
#define THREE_KFS_FEEDBACK_ID      THREE_KFS_ID
#define THREE_KFS_MASTER_ID        0x10

/***********************************************/
//上
#define KFS_ABOVE_ID               0x01
#define KFS_ABOVE_CMD_ID           0x200
#define KFS_ABOVE_FEEDBACK_ID      0x200 + KFS_ABOVE_ID
//下
#define KFS_BELOW_ID               0x02
#define KFS_BELOW_CMD_ID           0x200
#define KFS_BELOW_FEEDBACK_ID      0x200 + KFS_BELOW_ID

/************************ 偏移量 ***********************/
// three_kfs 
#define THREE_KFS_OFFSET1    2.64f   // P1=初始位置
#define THREE_KFS_OFFSET2    4.75f   
#define THREE_KFS_OFFSET3    6.875f   

// main_lift
#define MAIN_LIFT_OFFSET1    0.0f
#define MAIN_LIFT_OFFSET2    1.0f
#define MAIN_LIFT_OFFSET3    2.0f

// kfs_spin 
#define KFS_SPIN_OFFSET1     0.7f
#define KFS_SPIN_OFFSET2     2.0f
//#define KFS_SPIN_OFFSET3     1.5f
/*******************************************************************/




typedef enum{
	three_kfs_p1,
	three_kfs_p2,
	three_kfs_p3
}Three_kfs_position;

typedef enum{
	kfs_spin_p1,
	kfs_spin_p2,
} Kfs_spin_position;

typedef enum{
	main_lift_p1,
	main_lift_p2,
	main_lift_p3
} Main_lift_position;

typedef enum{
  above,
	below
}Kfs_flexible ;

// flexible电机控制切换标志
static uint8_t kfs_motor_select = 0;  // above=0,below=1
static uint16_t ch5_prev = CH5_MID; 

extern Three_kfs_position three_kfs_position;
extern Kfs_spin_position kfs_spin_position;
extern Main_lift_position main_lift_position;
extern Kfs_flexible kfs_flexible;

extern float main_lift_Initpos;
extern float kfs_spin_Initpos;
extern float three_kfs_Initpos;


extern float kfs_above_pid_param[PID_PARAMETER_NUM];
extern float kfs_below_pid_param[PID_PARAMETER_NUM];


typedef struct _Kfs_Module{
    StructureModule super_struct; 
    
                             
} Kfs_Module;

extern Kfs_Module Kfs;

extern DJI_MotorModule kfs_above;  
extern DJI_MotorModule kfs_below;  

extern DM_MotorModule main_lift;
extern DM_MotorModule kfs_spin;
extern DM_MotorModule three_kfs;

void manual_kfs_function(void);
void Initpos_Get(void);
void kfs_three_kfs_spin_main_lift_pos_init(void);

#endif
