#ifndef __LIFT_H__
#define __LIFT_H__


#include "global.h"
#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"

/************************抬升电机***********************/
#define R2_LIFT_MOTOR_LEFT_ID           0x05
#define R2_LIFT_MOTOR_LEFT_CMD_ID       R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_FEEDBACK_ID  R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_MASTER_ID    0x10

#define R2_LIFT_MOTOR_RIGHT_ID          0x06
#define R2_LIFT_MOTOR_RIGHT_CMD_ID      R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_FEEDBACK_ID R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_MASTER_ID   0x10

/************************收缩电机***********************/

//左
#define FLEXIBLE_MOTOR1_ID          0x03
#define FLEXIBLE_MOTOR1_CMD_ID      0x200
#define FLEXIBLE_MOTOR1_FEEDBACK_ID 0x200 + FLEXIBLE_MOTOR1_ID

//右
#define FLEXIBLE_MOTOR2_ID          0x04
#define FLEXIBLE_MOTOR2_CMD_ID      0x200
#define FLEXIBLE_MOTOR2_FEEDBACK_ID 0x200 + FLEXIBLE_MOTOR2_ID

#define FLEX_RUN_THR_RPM   10          // 判定“在运动中”的转速阈值
#define FLEX_STOP_THR_RPM  10          // 判定“接近停止”的转速阈值
#define FLEX_STOP_CNT_MAX  1           // 连续满足停止阈值N次才认为到尽头
#define FLEX_CMD_EXTEND_PWM   (-2500.0f) // 伸出阶段驱动值
#define FLEX_CMD_RETRACT_PWM  (2500.0f)  // 收回阶段驱动值

/* master抬升动作字节（8位）预留定义：
 * bit0: 抬升方向 0=下降 1=上升
 * bit1: 快速下降（仅下降方向有效，与遥控 CH4 快速下降语义一致）
 * bit2: 伸缩方向 0=收回 1=伸出
 * bit3~bit7: 预留
 */
#define MASTER_LIFT_UPDOWN_BIT    (1U << 0)
#define MASTER_LIFT_FALL_FAST_BIT (1U << 1)
#define MASTER_LIFT_FLEX_BIT      (1U << 2)

extern float flexible_motor1_pid_param[PID_PARAMETER_NUM];
extern float flexible_motor2_pid_param[PID_PARAMETER_NUM];

typedef struct _Lift_Module{
    StructureModule super_struct; 
    
                             
} Lift_Module;


// typedef enum {
// 	retraction,
//     stretch
		
               
// } Flexible_motor_state;


	

//抬升
extern Lift_Module Lift;
extern DM_MotorModule R2_lift_motor_left;
extern DM_MotorModule R2_lift_motor_right;

//收缩
extern DJI_MotorModule flexible_motor1;  // （左）
extern DJI_MotorModule flexible_motor2;  // （右）



extern int    lift_stop_mode ;     // 记录是上升停还是下降停，用于给刹车力矩



// extern Flexible_motor_state flexible_motor_state;

// //活动电机伸缩状态机状态
// extern uint8_t flexible_motor_has_stopped;
// extern uint8_t flexible_motor_running;
// extern int    flexible_motor_stop_mode;  
// extern int    last_flexible_motor_state;


//活动电机伸缩状态机状态
typedef enum
{
	FLEX_CMD_NONE = 0,
	FLEX_CMD_EXTEND,
	FLEX_CMD_RETRACT
} FlexibleMotorCmd;

typedef enum
{
	FLEX_ST_EXTENDING = 0,
	FLEX_ST_EXTENDED,
	FLEX_ST_RETRACTING,
	FLEX_ST_RETRACTED
} FlexibleMotorState4;



extern FlexibleMotorCmd flex_cmd;
extern FlexibleMotorState4 flex_state4;
extern uint16_t flex_input_prev;
extern uint8_t flex_seen_move;
extern uint8_t flex_stop_cnt;
extern float flexible_motor_PID_input;



void lift_init(void);
void manual_lift_function(void);
void flexible_motor_use(void);




void flexible_motor_update_command(uint16_t ch_value);
void flexible_motor_state_machine_step(void);



#endif
