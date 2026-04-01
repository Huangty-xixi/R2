#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "global.h"
#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"

/************************底盘电机***********************/
//左前
#define CHASSIS_MOTOR1_ID          0x01
#define CHASSIS_MOTOR1_CMD_ID      0x200
#define CHASSIS_MOTOR1_FEEDBACK_ID 0x200 + CHASSIS_MOTOR1_ID

//右前
#define CHASSIS_MOTOR2_ID          0x02
#define CHASSIS_MOTOR2_CMD_ID      0x200
#define CHASSIS_MOTOR2_FEEDBACK_ID 0x200 + CHASSIS_MOTOR2_ID

//右后
#define CHASSIS_MOTOR3_ID          0x03
#define CHASSIS_MOTOR3_CMD_ID      0x200
#define CHASSIS_MOTOR3_FEEDBACK_ID 0x200 + CHASSIS_MOTOR3_ID

//左后
#define CHASSIS_MOTOR4_ID          0x04
#define CHASSIS_MOTOR4_CMD_ID      0x200
#define CHASSIS_MOTOR4_FEEDBACK_ID 0x200 + CHASSIS_MOTOR4_ID

/************************导轮电机***********************/
//左
#define GUIDE_MOTOR1_ID          0x01
#define GUIDE_MOTOR1_CMD_ID      0x200
#define GUIDE_MOTOR1_FEEDBACK_ID 0x200 + GUIDE_MOTOR1_ID

//右
#define GUIDE_MOTOR2_ID          0x02
#define GUIDE_MOTOR2_CMD_ID      0x200
#define GUIDE_MOTOR2_FEEDBACK_ID 0x200 + GUIDE_MOTOR2_ID

//左
#define FLEXIBLE_MOTOR1_ID          0x03
#define FLEXIBLE_MOTOR1_CMD_ID      0x200
#define FLEXIBLE_MOTOR1_FEEDBACK_ID 0x200 + FLEXIBLE_MOTOR1_ID
#define FLEXIBLE_MOTOR1_MASTER_ID    0x09

//右
#define FLEXIBLE_MOTOR2_ID          0x04
#define FLEXIBLE_MOTOR2_CMD_ID      0x200
#define FLEXIBLE_MOTOR2_FEEDBACK_ID 0x200 + FLEXIBLE_MOTOR2_ID
#define FLEXIBLE_MOTOR2_MASTER_ID    0x09

/************************抬升电机***********************/
#define R2_LIFT_MOTOR_LEFT_ID           0x05
#define R2_LIFT_MOTOR_LEFT_CMD_ID       R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_FEEDBACK_ID  R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_MASTER_ID    0x10

#define R2_LIFT_MOTOR_RIGHT_ID          0x06
#define R2_LIFT_MOTOR_RIGHT_CMD_ID      R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_FEEDBACK_ID R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_MASTER_ID   0x10


extern float chassis_motor1_pid_param[PID_PARAMETER_NUM];   
extern float chassis_motor2_pid_param[PID_PARAMETER_NUM];
extern float chassis_motor3_pid_param[PID_PARAMETER_NUM];
extern float chassis_motor4_pid_param[PID_PARAMETER_NUM];

extern float guide_motor1_pid_param[PID_PARAMETER_NUM];
extern float guide_motor2_pid_param[PID_PARAMETER_NUM];


/*******************************************************/

typedef struct{
    float Vx_in;
    float Vy_in;
    float Vw_in;
    float Accel;
    
    float V_out[4];
}Chassis_Param;

typedef struct _Lift_Module{
    StructureModule super_struct; 
    
                             
} Lift_Module;


typedef struct _Chassis_Module{
    StructureModule super_struct; 
    
    Chassis_Param param;                             
    
    void (*Chassis_Calc)(struct _Chassis_Module *chassis);
    void (*Chassis_Stop)(struct _Chassis_Module *chassis);
} Chassis_Module;
//底盘
extern Chassis_Module Chassis;
extern DJI_MotorModule chassis_motor1;  // （左前）
extern DJI_MotorModule chassis_motor2;  // （右前）
extern DJI_MotorModule chassis_motor3;  // （左后）
extern DJI_MotorModule chassis_motor4;  // （右后）
//导轮
extern DJI_MotorModule guide_motor1;  // （左）
extern DJI_MotorModule guide_motor2;  // （右）
extern DM_MotorModule flexible_motor1;  // （左）
extern DM_MotorModule flexible_motor2;  // （右）
//抬升
extern Lift_Module Lift;
extern DM_MotorModule R2_lift_motor_left;
extern DM_MotorModule R2_lift_motor_right;

extern float Initpos[2];

void Chassis_Calc(Chassis_Module *chassis);
void Chassis_Stop(Chassis_Module *chassis);

#endif
