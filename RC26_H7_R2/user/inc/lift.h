#ifndef __LIFT_H__
#define __LIFT_H__


#include "global.h"
#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"

/************************抬升电机***********************/
#define R2_LIFT_MOTOR_LEFT_ID           0x05
#define R2_LIFT_MOTOR_LEFT_CMD_ID       R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_FEEDBACK_ID  R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_MASTER_ID    0x10

#define R2_LIFT_MOTOR_RIGHT_ID          0x06
#define R2_LIFT_MOTOR_RIGHT_CMD_ID      R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_FEEDBACK_ID R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_MASTER_ID   0x10

/************************收缩电机***********************/

//左
#define FLEXIBLE_MOTOR1_ID          0x03
#define FLEXIBLE_MOTOR1_CMD_ID      0x200
#define FLEXIBLE_MOTOR1_FEEDBACK_ID 0x200 + FLEXIBLE_MOTOR1_ID

//右
#define FLEXIBLE_MOTOR2_ID          0x04
#define FLEXIBLE_MOTOR2_CMD_ID      0x200
#define FLEXIBLE_MOTOR2_FEEDBACK_ID 0x200 + FLEXIBLE_MOTOR2_ID

extern float flexible_motor1_pid_param[PID_PARAMETER_NUM];
extern float flexible_motor2_pid_param[PID_PARAMETER_NUM];

typedef struct _Lift_Module{
    StructureModule super_struct; 
    
                             
} Lift_Module;


typedef enum {
	retraction,
    stretch
		
               
} Flexible_motor_state;

	

//抬升
extern Lift_Module Lift;
extern DM_MotorModule R2_lift_motor_left;
extern DM_MotorModule R2_lift_motor_right;

//收缩
extern DJI_MotorModule flexible_motor1;  // （左）
extern DJI_MotorModule flexible_motor2;  // （右）



extern int    lift_stop_mode ;     // 记录是上升停还是下降停，用于给刹车力矩

extern float flexible_motor_PID_input;

extern Flexible_motor_state flexible_motor_state;

//活动电机伸缩状态机状态
extern uint8_t flexible_motor_has_stopped;
extern uint8_t flexible_motor_running;
extern int    flexible_motor_stop_mode;  
extern int    last_flexible_motor_state;



void lift_init(void);
void manual_lift_function(void);
void flexible_motor_use(void);



#endif
