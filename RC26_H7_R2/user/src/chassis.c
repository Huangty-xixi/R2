#include "chassis.h"

Chassis_Module Chassis;

//底盘
DJI_MotorModule chassis_motor1;  // （左前）
DJI_MotorModule chassis_motor2;  // （右前）
DJI_MotorModule chassis_motor3;  // （左后）
DJI_MotorModule chassis_motor4;  // （右后）

//导轮
DJI_MotorModule guide_motor1;  // （左）
DJI_MotorModule guide_motor2;  // （右）
DM_MotorModule flexible_motor1;//（左）
DM_MotorModule flexible_motor2;//（右）

//抬升
Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;//（左）
DM_MotorModule R2_lift_motor_right;//（右）

float chassis_motor1_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};     //KP,KI,KD,DEADBAND,LIMITINTEGRAL,LIMITOUTPUT
float chassis_motor2_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor3_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor4_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};

//float chassis_motor1_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.f,1,500.0f,10000.0f};     //KP,KI,KD,DEADBAND,LIMITINTEGRAL,LIMITOUTPUT
//float chassis_motor2_pid_param[PID_PARAMETER_NUM] = {3.5f,0.05f,0.f,1,500.0f,10000.0f};
//float chassis_motor3_pid_param[PID_PARAMETER_NUM] = {3.5f,0.05f,0.f,1,500.0f,10000.0f};
//float chassis_motor4_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.f,1,500.0f,10000.0f};

float guide_motor1_pid_param[PID_PARAMETER_NUM] = {4.5f,0.1f,0.1f,1,500.0f,10000.0f};
float guide_motor2_pid_param[PID_PARAMETER_NUM] = {4.5f,0.1f,0.1f,1,500.0f,10000.0f};

void Chassis_Calc(Chassis_Module *chassis)
{
    chassis->param.Accel = ACCEL;
    chassis->param.Vx_in =-LR_TRANSLATION;
    chassis->param.Vy_in =FB_TRANSLATION;
    chassis->param.Vw_in = ROTATION;
    
    chassis->param.V_out[0] = chassis->param.Vx_in + chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[1] = chassis->param.Vx_in - chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[2] = chassis->param.Vx_in + chassis->param.Vy_in - chassis->param.Vw_in;
    chassis->param.V_out[3] = chassis->param.Vx_in - chassis->param.Vy_in - chassis->param.Vw_in;

}

void Chassis_Stop(Chassis_Module *chassis)
{
// 先把结构体里的输入置0
    chassis->param.Vx_in = 0.0f;
    chassis->param.Vy_in = 0.0f;
    chassis->param.Vw_in = 0.0f;

    // 再调用一次计算函数，自动生成刹车用的 V_out
    Chassis_Calc(chassis);
    
}

////抬升
//float Initpos[2] = {0};
//void Initpos_Get(void)
//{
//    float pre_pos[2];
//    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 5, 0, 0.3, 0);
//    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, 5, 0, 0.3, 0);
//    pre_pos[0] = R2_lift_motor_left.position;
//    pre_pos[1] = R2_lift_motor_right.position;
//    vTaskDelay(500);
//    while(((R2_lift_motor_left.position - pre_pos[0]) >= 0.05) || ((R2_lift_motor_right.position - pre_pos[1]) >= 0.05)){
//        R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 5, 0, 0.3, 0);
//        R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, 5, 0, 0.3, 0);
//        pre_pos[0] = R2_lift_motor_left.position;
//        pre_pos[1] = R2_lift_motor_right.position;
//        vTaskDelay(100);
//    }
//    Initpos[0] = R2_lift_motor_left.position;
//    Initpos[1] = R2_lift_motor_right.position;
//}

