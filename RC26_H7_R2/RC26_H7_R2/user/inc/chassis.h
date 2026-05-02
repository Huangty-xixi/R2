#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "global.h"
#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"

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

/************************固定旋转90°***********************/
#define ROTATE_KP          2.0f
#define ROTATE_KI          0.1f
#define ROTATE_KD          0.5f
#define ROTATE_I_LIMIT     10.0f
#define ROTATE_OUT_LIMIT   15.0f
#define ROTATE_ANGLE_TH    1.5f
#define ROTATE_SPEED_TH    3.0f
#define ROTATE_TIMEOUT_MS  3000U



extern float chassis_motor1_pid_param[PID_PARAMETER_NUM];   
extern float chassis_motor2_pid_param[PID_PARAMETER_NUM];
extern float chassis_motor3_pid_param[PID_PARAMETER_NUM];
extern float chassis_motor4_pid_param[PID_PARAMETER_NUM];

extern float guide_motor1_pid_param[PID_PARAMETER_NUM];
extern float guide_motor2_pid_param[PID_PARAMETER_NUM];


/*******************************************************/

typedef struct{
    float Vx_in;
    float Vy_in;
    float Vw_in;
    float Accel;
    
    float V_out[4];
}Chassis_Param;

/* master底盘动作字节0（8位）定义：
 * bit7~bit6: 速度档位   00=低速 01=常速 10=高速 11=super_high
 * bit5~bit3: 平移方向   001=前 010=后 011=左 100=右（其余预留）
 * bit2~bit1: 旋转方向   00=不旋转 01=左旋 10=右旋（11预留）
 * bit0     : 活动电机   0=收回 1=伸出
 */
typedef enum
{
    CHASSIS_SPEED_LOW =0,
    CHASSIS_SPEED_NORMAL,
    CHASSIS_SPEED_HIGH,
    CHASSIS_SPEED_SUPER_HIGH
} chassis_speed_level_t;

typedef enum
{
    CHASSIS_DIR_NONE = 0,
    CHASSIS_DIR_FORWARD = 1,
    CHASSIS_DIR_BACKWARD = 2,
    CHASSIS_DIR_LEFT = 3,
    CHASSIS_DIR_RIGHT = 4
} chassis_move_dir_t;

typedef enum
{
    CHASSIS_ROT_NONE = 0,
    CHASSIS_ROT_LEFT = 1,
    CHASSIS_ROT_RIGHT = 2,
    CHASSIS_ROT_RESERVED = 3
} chassis_rot_dir_t;

typedef struct
{
    chassis_speed_level_t speed_level;
    chassis_move_dir_t move_dir;
    chassis_rot_dir_t rot_dir;
    uint8_t flexible_extend;
    uint8_t reserved_byte1; /* 预留：第二字节后续可扩展 */
} master_chassis_cmd_t;


typedef struct _Chassis_Module{
    StructureModule super_struct; 
    
    Chassis_Param param;                             
    
    void (*Chassis_Calc)(struct _Chassis_Module *chassis);
    void (*Chassis_Stop)(struct _Chassis_Module *chassis);
} Chassis_Module;

typedef enum {
    ROTATE_NONE = 0,
    ROTATE_LEFT_90,
    ROTATE_RIGHT_90,
    ROTATE_COMPLETE
} RotateState;


RotateState Chassis_Rotate90(RotateState direction);

//底盘
extern Chassis_Module Chassis;
extern DJI_MotorModule chassis_motor1;  // （左前）
extern DJI_MotorModule chassis_motor2;  // （右前）
extern DJI_MotorModule chassis_motor3;  // （左后）
extern DJI_MotorModule chassis_motor4;  // （右后）
//导轮
extern DJI_MotorModule guide_motor1;  // （左）
extern DJI_MotorModule guide_motor2;  // （右）


void Chassis_Calc(Chassis_Module *chassis);
void Chassis_Stop(Chassis_Module *chassis);
void R2_lift(void);
void manual_chassis_function(void);
RotateState Chassis_Rotate90(RotateState direction);
extern RotateState g_rotate_state;

#endif
