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
	DM_J4340,
	DJI_4in1,
	DM_MIT,
	DM_J4310,
  DM_2325,
	DM_S3519
}Motor_Model;


typedef struct _MotorModule{
    
    baseModule  base;
    Motor_Model model;
    Ctrl_mode   mode;
    
    uint8_t id;
    FDCAN_HandleTypeDef hcan;
} MotorModule;

void MotorModule_Create(MotorModule *obj, uint8_t motor_id, FDCAN_HandleTypeDef *hcan, Motor_Model model, Ctrl_mode mode);

/* 全电机过温保护：
 * 返回1表示当前处于过温保护中，应停止输出；
 * 返回0表示温度正常。
 */
uint8_t Motor_OverTempProtect_Update(void);

/* 简单过温自测（默认关闭，打开宏后在Can_Task循环中调用） */
void Motor_OverTemp_SimpleTest(void);
extern volatile uint8_t g_overtemp_test_step;
extern volatile uint8_t g_overtemp_test_result;

#endif
