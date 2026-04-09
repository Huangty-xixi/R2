#include "register.h"
#include "global.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"
#include "tim.h"

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

	
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor1.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor2.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor3.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor4.super_motor); 
    //////////////////////////////////////////////////////////////////////////////////
    Chassis.Chassis_Calc = Chassis_Calc;
    Chassis.Chassis_Stop = Chassis_Stop;
		///////////////////////////////////////

 
}

void Kfs_Init(void)
{
	 	StructureModule_Create(&Kfs.super_struct, kfs);
    Kfs.super_struct.base.Init(&Kfs.super_struct.base);
	
  	DJImotor_Create(&kfs_above,KFS_ABOVE_CMD_ID,KFS_ABOVE_FEEDBACK_ID,&hfdcan3,DJI_2006,SPEED,PID_POSITION,kfs_above_pid_param);
		DJImotor_Create(&kfs_below,KFS_BELOW_CMD_ID,KFS_BELOW_FEEDBACK_ID,&hfdcan3,DJI_2006,SPEED,PID_POSITION,kfs_below_pid_param);
	
	  DMmotor_Create(&main_lift, MAIN_LIFT_CMD_ID, MAIN_LIFT_MASTER_ID, &hfdcan3, DM_S3519, MIT);
    DMmotor_Create(&kfs_spin, KFS_SPIN_CMD_ID, KFS_SPIN_MASTER_ID, &hfdcan3, DM_J4310, MIT);
	  DMmotor_Create(&three_kfs, THREE_KFS_CMD_ID, THREE_KFS_MASTER_ID, &hfdcan3, DM_6220, MIT);
  
	 	Kfs.super_struct.AddMotor(&Kfs.super_struct, &kfs_above.super_motor);
    Kfs.super_struct.AddMotor(&Kfs.super_struct, &kfs_below.super_motor);
	
	  Kfs.super_struct.AddMotor(&Kfs.super_struct, &main_lift.super_motor);
    Kfs.super_struct.AddMotor(&Kfs.super_struct, &kfs_spin.super_motor);
	  Kfs.super_struct.AddMotor(&Kfs.super_struct, &three_kfs.super_motor);
	
		main_lift.send_cmd(&main_lift,Motor_Enable);
		HAL_Delay(5);
		kfs_spin.send_cmd(&kfs_spin,Motor_Enable);
	  HAL_Delay(5);
		three_kfs.send_cmd(&three_kfs,Motor_Enable);
	  HAL_Delay(5);
		
}

void Lift_Init(void)
{
  	DJImotor_Create(&flexible_motor1,FLEXIBLE_MOTOR1_CMD_ID,FLEXIBLE_MOTOR1_FEEDBACK_ID,&hfdcan2,DJI_2006,SPEED,PID_POSITION,flexible_motor1_pid_param);
		DJImotor_Create(&flexible_motor2,FLEXIBLE_MOTOR1_CMD_ID,FLEXIBLE_MOTOR1_FEEDBACK_ID,&hfdcan2,DJI_2006,SPEED,PID_POSITION,flexible_motor2_pid_param);
	 
  	StructureModule_Create(&Lift.super_struct, lift);
    Lift.super_struct.base.Init(&Lift.super_struct.base);
    
    DMmotor_Create(&R2_lift_motor_left, R2_LIFT_MOTOR_LEFT_CMD_ID, R2_LIFT_MOTOR_LEFT_MASTER_ID, &hfdcan1, DM_J4310, MIT);
    DMmotor_Create(&R2_lift_motor_right, R2_LIFT_MOTOR_RIGHT_CMD_ID, R2_LIFT_MOTOR_RIGHT_MASTER_ID, &hfdcan1, DM_J4310, MIT);
  
  
    Lift.super_struct.AddMotor(&Lift.super_struct, &R2_lift_motor_left.super_motor);
    Lift.super_struct.AddMotor(&Lift.super_struct, &R2_lift_motor_right.super_motor);
    
    R2_lift_motor_left.send_cmd(&R2_lift_motor_left,Motor_Enable);
    R2_lift_motor_right.send_cmd(&R2_lift_motor_right,Motor_Enable);
  
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
