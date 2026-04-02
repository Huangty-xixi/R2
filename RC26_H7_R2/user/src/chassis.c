#include "chassis.h"
#include "Motion_Task.h"

Chassis_Module Chassis;

//µ×ÅÌ
DJI_MotorModule chassis_motor1;  // £¨×óÇ°£©
DJI_MotorModule chassis_motor2;  // £¨ÓÒÇ°£©
DJI_MotorModule chassis_motor3;  // £¨×óºó£©
DJI_MotorModule chassis_motor4;  // £¨ÓÒºó£©

//µ¼ÂÖ
DJI_MotorModule guide_motor1;  // £¨×ó£©
DJI_MotorModule guide_motor2;  // £¨ÓÒ£©


float chassis_motor1_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.15f,1,500.0f,10000.0f};     //KP,KI,KD,DEADBAND,LIMITINTEGRAL,LIMITOUTPUT
float chassis_motor2_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor3_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor4_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.15f,1,500.0f,10000.0f};

float guide_motor1_pid_param[PID_PARAMETER_NUM] = {3.0f,0.1f,0.2f,1,500.0f,10000.0f};
float guide_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};


void Chassis_Calc(Chassis_Module *chassis)
{
    chassis->param.Accel = ACCEL;
    chassis->param.Vx_in =-LR_TRANSLATION;
    chassis->param.Vy_in = FB_TRANSLATION;
    chassis->param.Vw_in = ROTATION;
    
    chassis->param.V_out[0] = chassis->param.Vx_in + chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[1] = chassis->param.Vx_in - chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[2] = chassis->param.Vx_in + chassis->param.Vy_in - chassis->param.Vw_in;
    chassis->param.V_out[3] = chassis->param.Vx_in - chassis->param.Vy_in - chassis->param.Vw_in;

}

void Chassis_Stop(Chassis_Module *chassis)
{
    chassis->param.V_out[0] = 0.f;
    chassis->param.V_out[1] = 0.f;
    chassis->param.V_out[2] = 0.f;
    chassis->param.V_out[3] = 0.f;
    
}


