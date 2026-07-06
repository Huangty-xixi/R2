#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "global.h"
#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"
#include "chassis_vel_pid.h"

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


typedef struct {
    volatile float kp, ki, kd, Deadband, limitIntegral, limitOutput;
} motor_pid_tune_t;

typedef struct {
    volatile motor_pid_tune_t m1;  /* 左前 */
    volatile motor_pid_tune_t m2;  /* 右前 */
    volatile motor_pid_tune_t m3;  /* 左后 */
    volatile motor_pid_tune_t m4;  /* 右后 */
} chassis_pid_tune_t;

extern volatile chassis_pid_tune_t g_chassis_pid;

extern volatile float guide_motor1_pid_param[PID_PARAMETER_NUM];
extern volatile float guide_motor2_pid_param[PID_PARAMETER_NUM];
typedef struct {
    volatile float m1;  /* 左前 */
    volatile float m2;  /* 右前 */
    volatile float m3;  /* 左后 */
    volatile float m4;  /* 右后 */
} chassis_speed_rpm_t;
extern volatile chassis_speed_rpm_t g_chassis_speed;


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

    volatile float chassis_vel_pid_vy_out;//底盘分轴速度PID vy输出
    volatile float chassis_vel_pid_vw_out;//底盘分轴速度PID vw输出
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
void Chassis_ServiceTick(void);
void ChassisControl_RunPipeline(Chassis_Module *chassis, const ChassisControlCmd *cmd_in, const ChassisControlFeedback *fb);
void Chassis_Stop(Chassis_Module *chassis);
/** 急停：三轴指令为 0，走 Chassis_Calc+PID 再发 CAN1（不直接清 pid 输出；CAN2 导轮由急停分支单独清零） */
void Chassis_EmergencyBrakeRun(Chassis_Module *chassis);
void Chassis_EmergencyBrake_Engage(void);
void Chassis_EmergencyBrake_DisengageIfStopped(void);
void Chassis_EmergencyBrake_Disengage(void);
extern volatile motor_pid_tune_t s_em_tune;
/** CAN2 0x200：导轮 1、2 号 */
void Chassis_Can2_PublishGuide(void);
void Chassis_Can2_PublishGuideZero(void);
void R2_lift(void);
/** 底盘周期入口：含锁死/正常输出，见 chassis_lock_hold.h */
void manual_chassis_function(void);

#endif
