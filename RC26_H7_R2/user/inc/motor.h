#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "global.h"
#include "fdcan.h"
#include "pid.h"

#define FILTER_BUF_LEN		5


typedef enum
{
	SPEED,
	POSITION,
    MIT,
}Ctrl_mode;


typedef enum
{
	DJI_2006,
	DJI_3508,
	DJI_6020,
	DM_3520,
	DM_6220,
	DJI_4in1,
	DM_MIT,
	DM_J4310,
    DM_2325
}Motor_Model;


typedef struct _MotorModule{
    
    baseModule  base;
    Motor_Model model;
    Ctrl_mode   mode;
    
    uint8_t id;
    FDCAN_HandleTypeDef hcan;
} MotorModule;

void MotorModule_Create(MotorModule *obj, uint8_t motor_id, FDCAN_HandleTypeDef *hcan, Motor_Model model, Ctrl_mode mode);

#endif
