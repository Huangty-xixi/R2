#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__


#include "fdcan.h"
#include "motor.h"
#include "math.h"

typedef enum
{
	Motor_Enable,
	Motor_Disable,
	Motor_Save_Zero_Position,
	Motor_Clear_Error,
}Motor_CMD;

typedef enum
{
	OFF,
	ON,
	OVER_VOL,
	LOW_VOL,
	OVER_CUR,
	MOS_HOT,
	LOSS,
	OVER_LOAD
	
}Motor_State;

typedef struct _DM_MotorModule{
    MotorModule super_motor;
    struct _DM_MotorModule* self;
    Motor_State state;
	uint8_t			Master_ID;
	
	float 	position;
	float 	last_position;
	float	abs_position;
	float 	speed_w;
	float 	torque;
	
	uint8_t  	temp_mos;
	uint8_t  	temp_rotor;
	int32_t		round_cnt;
	uint8_t		buf_idx;
	uint16_t	angle_buf[FILTER_BUF_LEN];
	uint16_t	fited_angle;
	uint32_t	msg_cnt;
    
    HAL_StatusTypeDef (*send_cmd)(struct _DM_MotorModule *obj,Motor_CMD CMD);
    void (*get_motor_measure)(struct _DM_MotorModule *obj, uint8_t rx_data[8]);
	HAL_StatusTypeDef (*set_mit_data)(struct _DM_MotorModule *obj, float Position, float Velocity, float KP, float KD, float Torque);
    HAL_StatusTypeDef (*set_posvel_data)(struct _DM_MotorModule *obj, float Position, float Velocity);
}DM_MotorModule;


//extern DM_MotorModule kfs_lift_motor;  
//extern DM_MotorModule kfs_flex_motor;  
extern DM_MotorModule R2_lift_motor_left;
extern DM_MotorModule R2_lift_motor_right;

void DMmotor_Create (DM_MotorModule *obj, uint16_t command_id, uint16_t master_id,FDCAN_HandleTypeDef *hcan, Motor_Model motorModel,Ctrl_mode mode);
HAL_StatusTypeDef DM_Motor_CMD(DM_MotorModule *obj,Motor_CMD CMD);
void DMget_motor_measure(DM_MotorModule *obj, uint8_t rx_data[8]);
HAL_StatusTypeDef DMset_mit_data(DM_MotorModule *obj, float Position, float Velocity, float KP, float KD, float Torque);

#endif
