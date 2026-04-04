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


void Chassis_Calc(Chassis_Module *chassis);
void Chassis_Stop(Chassis_Module *chassis);
void R2_lift(void);
void chassis_use(void);
void manual_chassis_function(void);

#endif
