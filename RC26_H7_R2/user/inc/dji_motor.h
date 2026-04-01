#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__


#include "fdcan.h"
#include "pid.h"
#include "motor.h"



typedef struct _DJI_MotorModule{
    
    MotorModule super_motor;
    uint16_t    CAN_FEEDBACK_ID;
    
    PID_Info_TypeDef	pid_spd;
	PID_Info_TypeDef 	pid_pos;
    
    int16_t	 	speed_rpm;
	int16_t  	real_current;
	int16_t  	given_current;
	uint8_t  	temp;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;		//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint8_t		buf_idx;
	uint16_t	angle_buf[FILTER_BUF_LEN];
	uint16_t	fited_angle;
	uint32_t	msg_cnt;
    
    float (*PID_Calculate)(struct _DJI_MotorModule *obj, float input);
    void (*get_moto_measure)(struct _DJI_MotorModule *obj, uint8_t rx_data[8]);
    void (*get_moto_offset)(struct _DJI_MotorModule *obj, uint8_t rx_data[8]);
    
} DJI_MotorModule;


float Motor_PID_Calculate(DJI_MotorModule *moto_info, float input);
void DJI_Motor_Init(DJI_MotorModule *moto_info, uint16_t command_id, uint16_t feedback_id, Motor_Model motorModel, Ctrl_mode ctrlMode, 
								PID_Type_e pidType, float pid_spd_Param[PID_PARAMETER_NUM], float pid_pos_Param[PID_PARAMETER_NUM]);

void DJIget_motor_measure(DJI_MotorModule *ptr, uint8_t data[8]);
void DJIget_moto_offset(DJI_MotorModule *ptr,uint8_t data[8]);
HAL_StatusTypeDef DJIset_motor_data(FDCAN_HandleTypeDef* hcan, uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4);
void DJImotor_Create(DJI_MotorModule *obj, uint16_t command_id, uint16_t feedback_id,FDCAN_HandleTypeDef *hcan, Motor_Model motorModel,
                     Ctrl_mode mode,PID_Type_e pidType, float pid_Param[PID_PARAMETER_NUM]) ;

extern DJI_MotorModule chassis_motor1;  // （左前）
extern DJI_MotorModule chassis_motor2;  // （右前）
extern DJI_MotorModule chassis_motor3;  // （左后）
extern DJI_MotorModule chassis_motor4;  // （右后）

extern DJI_MotorModule guide_motor1;  // （左）
extern DJI_MotorModule guide_motor2;  // （右）


#endif
