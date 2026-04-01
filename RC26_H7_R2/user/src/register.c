#include "register.h"

#include "global.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"

void Chassis_Init(void) 
{
    StructureModule_Create(&Chassis.super_struct, chassis);
    Chassis.super_struct.base.Init(&Chassis.super_struct.base);
    
    DJImotor_Create(&chassis_motor1, CHASSIS_MOTOR1_CMD_ID, CHASSIS_MOTOR1_FEEDBACK_ID, &hfdcan1, DJI_3508, SPEED, PID_POSITION, chassis_motor1_pid_param);
    DJImotor_Create(&chassis_motor2, CHASSIS_MOTOR2_CMD_ID, CHASSIS_MOTOR2_FEEDBACK_ID, &hfdcan1, DJI_3508, SPEED, PID_POSITION, chassis_motor2_pid_param);
    DJImotor_Create(&chassis_motor3, CHASSIS_MOTOR3_CMD_ID, CHASSIS_MOTOR3_FEEDBACK_ID, &hfdcan1, DJI_3508, SPEED, PID_POSITION, chassis_motor3_pid_param);
    DJImotor_Create(&chassis_motor4, CHASSIS_MOTOR4_CMD_ID, CHASSIS_MOTOR4_FEEDBACK_ID, &hfdcan1, DJI_3508, SPEED, PID_POSITION, chassis_motor4_pid_param);

	  DJImotor_Create(&guide_motor1, GUIDE_MOTOR1_CMD_ID, GUIDE_MOTOR1_FEEDBACK_ID, &hfdcan2, DJI_2006, SPEED, PID_POSITION, guide_motor1_pid_param);
    DJImotor_Create(&guide_motor2, GUIDE_MOTOR2_CMD_ID, GUIDE_MOTOR2_FEEDBACK_ID, &hfdcan2, DJI_2006, SPEED, PID_POSITION, guide_motor2_pid_param);

		DJImotor_Create(&flexible_motor1,FLEXIBLE_MOTOR1_CMD_ID,FLEXIBLE_MOTOR1_FEEDBACK_ID,&hfdcan2,DJI_2006,SPEED,PID_POSITION,flexible_motor1_pid_param);
		DJImotor_Create(&flexible_motor2,FLEXIBLE_MOTOR1_CMD_ID,FLEXIBLE_MOTOR1_FEEDBACK_ID,&hfdcan2,DJI_2006,SPEED,PID_POSITION,flexible_motor1_pid_param);


	
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor1.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor2.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor3.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor4.super_motor); 
    //////////////////////////////////////////////////////////////////////////////////
    Chassis.Chassis_Calc = Chassis_Calc;
    Chassis.Chassis_Stop = Chassis_Stop;
		///////////////////////////////////////
	  StructureModule_Create(&Lift.super_struct, lift);
    Lift.super_struct.base.Init(&Lift.super_struct.base);
    
    DMmotor_Create(&R2_lift_motor_left, R2_LIFT_MOTOR_LEFT_CMD_ID, R2_LIFT_MOTOR_LEFT_MASTER_ID, &hfdcan1, DM_J4310, MIT);
    DMmotor_Create(&R2_lift_motor_right, R2_LIFT_MOTOR_RIGHT_CMD_ID, R2_LIFT_MOTOR_RIGHT_MASTER_ID, &hfdcan1, DM_J4310, MIT);
  
  
    Lift.super_struct.AddMotor(&Lift.super_struct, &R2_lift_motor_left.super_motor);
    Lift.super_struct.AddMotor(&Lift.super_struct, &R2_lift_motor_right.super_motor);
    
    R2_lift_motor_left.send_cmd(&R2_lift_motor_left,Motor_Enable);
    R2_lift_motor_right.send_cmd(&R2_lift_motor_right,Motor_Enable);
	
 
}

void Kfs_Init(void)
{
 
}

void Lift_Init(void)
{
   
}

void Weapon_Init(void)
{

}

void Structue_Init(void)
{
    Chassis_Init();
    HAL_Delay(5);
    Kfs_Init();
    HAL_Delay(5);
    Lift_Init();
    HAL_Delay(5);
    Weapon_Init();
    HAL_Delay(5);
}
