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

typedef struct
{
    volatile float rotation_cmd_raw;//旋转命令
    volatile float yaw_body_deg;//车身航向角

    volatile float vx_in_raw;//x轴输入速度
    volatile float vy_in_raw;//y轴输入速度
    volatile float vw_in_raw;//w轴输入速度

    volatile float vy_after_decouple;//y轴解耦后速度    
    volatile float vw_after_decouple;//w轴解耦后速度

    volatile float heading_hold_vx_comp;//航向保持补偿
    volatile float transient_vx_comp;//瞬态补偿
    volatile float odom_vy_comp;//里程计交叉补偿（加到Vy）
    volatile float odom_vw_comp;//里程计交叉补偿（加到Vw）

    volatile float vx_after_limit;//x轴限幅后速度
    volatile float vy_after_limit;//y轴限幅后速度
    volatile float vw_after_limit;//w轴限幅后速度

    volatile float v_out0;//左前电机输出速度
    volatile float v_out1;//右前电机输出速度
    volatile float v_out2;//右后电机输出速度
    volatile float v_out3;//左后电机输出速度
} ChassisDebugSnapshot;

typedef struct
{
    float vx_cmd;
    float vy_cmd;
    float vw_cmd;
} ChassisControlCmd;

typedef struct
{
    float yaw_body_deg;
} ChassisControlFeedback;


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
extern volatile ChassisDebugSnapshot g_chassis_dbg;



void Chassis_Calc(Chassis_Module *chassis);
void ChassisControl_RunPipeline(Chassis_Module *chassis, const ChassisControlCmd *cmd_in, const ChassisControlFeedback *fb);
void Chassis_Stop(Chassis_Module *chassis);
void R2_lift(void);
void manual_chassis_function(void);

#endif
