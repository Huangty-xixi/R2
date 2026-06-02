#ifndef __KFS_H__
#define __KFS_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"

/***********************************************/

// (can2)
#define MAIN_LIFT_ID               0x05
#define MAIN_LIFT_CMD_ID           MAIN_LIFT_ID
#define MAIN_LIFT_FEEDBACK_ID      MAIN_LIFT_ID
#define MAIN_LIFT_MASTER_ID        0x10

#define KFS_SPIN_ID                0x04
#define KFS_SPIN_CMD_ID            KFS_SPIN_ID
#define KFS_SPIN_FEEDBACK_ID       KFS_SPIN_ID
#define KFS_SPIN_MASTER_ID         0x10

#define THREE_KFS_ID               0x05
#define THREE_KFS_CMD_ID           THREE_KFS_ID
#define THREE_KFS_FEEDBACK_ID      THREE_KFS_ID
#define THREE_KFS_MASTER_ID        KFS_SPIN_MASTER_ID

/***********************************************/
// 上
#define KFS_ABOVE_ID               0x01
#define KFS_ABOVE_CMD_ID           0x200
#define KFS_ABOVE_FEEDBACK_ID      0x200 + KFS_ABOVE_ID
// 下
#define KFS_BELOW_ID               0x02
#define KFS_BELOW_CMD_ID           0x200
#define KFS_BELOW_FEEDBACK_ID      0x200 + KFS_BELOW_ID

/************************ 偏移量 ***********************/
// three_kfs 
#define THREE_KFS_OFFSET1    -3.05f//2.64f  吸盘2
#define THREE_KFS_OFFSET2    -0.921f//6.875f  吸盘3
#define THREE_KFS_OFFSET3    1.185f//4.75f  吸盘4
#define THREE_KFS_OFFSET4    2.2f /* p4 角度，上场前按机械标定 */



//// main_lift
//#define MAIN_LIFT_OFFSET1    -6.0f
//#define MAIN_LIFT_OFFSET2    -6.0f
//#define MAIN_LIFT_OFFSET3    -6.0f
//#define MAIN_LIFT_OFFSET4    -6.0f

// kfs_spin 
#define KFS_SPIN_OFFSET1     0.9f//0.7f
#define KFS_SPIN_OFFSET2     2.6f

/*******************************************************************/




typedef enum{
	three_kfs_p1,
	three_kfs_p2,
	three_kfs_p3,
	three_kfs_p4,
}Three_kfs_position;

typedef enum{
	kfs_spin_p1,
	kfs_spin_p2,
} Kfs_spin_position;

typedef enum{
	main_lift_p0, /* 000: 停止/不动作（等待001再动） */
	main_lift_p1,
	main_lift_p2,
	main_lift_p3,
	main_lift_p4
} Main_lift_position;

typedef enum{
  above,
	below
}Kfs_flexible ;

// CH5 改用于切换 kfs_below 速度/位置模式（原上下电机选择已移除）
static uint16_t ch5_prev = CH5_MID; 

/* ==================== 伸缩电机位置环（above/below 共用，volatile 方便在线调参） ==================== */
typedef struct {
    volatile float pos_kp;          /* 位置环 P */
    volatile float pos_ki;          /* 位置环 I */
    volatile float pos_kd;          /* 位置环 D */
    volatile float max_speed;       /* 位置环输出限幅（CH2 等效值，x200 后为实际速度指令） */
    volatile float pos_rounds[4];   /* 四档目标圈数（相对base），各自独立可调 */
    volatile float pos_i_limit;     /* 积分限幅（CH2 等效值） */
} Kfs_Flex_PosCtrl_Param;

/* 伸缩电机控制模式（CH5 四档循环切换） */
typedef enum {
    flex_below_speed = 0,     /* below 速度控制（默认） */
    flex_above_speed = 1,     /* above 速度控制 */
    flex_below_position = 2,  /* below 位置控制 */
    flex_above_position = 3   /* above 位置控制 */
} Flexible_Mode;

/* 伸缩电机目标档位（位置模式下共用） */
typedef enum {
    flex_pos0 = 0,  /* 切入位置模式时的当前位置 */
    flex_pos1 = 1,  /* pos_rounds[1] */
    flex_pos2 = 2,  /* pos_rounds[2] */
    flex_pos3 = 3   /* pos_rounds[3] */
} Flex_TargetPos;

extern volatile Kfs_Flex_PosCtrl_Param kfs_below_pos_param;  /* below 伸缩位置参数 */
extern volatile Kfs_Flex_PosCtrl_Param kfs_above_pos_param;  /* above 伸缩位置参数 */
extern volatile Flexible_Mode flexible_mode;
extern volatile Flex_TargetPos flex_target_pos;

/* 全自动模式位置指令（类似 main_lift_position） */
typedef enum {
    kfs_below_cmd_stop = 0,  /* 停止 */
    kfs_below_cmd_p0   = 1,  /* pos_rounds[0] */
    kfs_below_cmd_p1   = 2,  /* pos_rounds[1] */
    kfs_below_cmd_p2   = 3,  /* pos_rounds[2] */
    kfs_below_cmd_p3   = 4   /* pos_rounds[3] */
} Kfs_Below_Cmd;

typedef enum {
    kfs_above_cmd_stop = 0,  /* 停止 */
    kfs_above_cmd_p0   = 1,  /* pos_rounds[0] */
    kfs_above_cmd_p1   = 2,  /* pos_rounds[1] */
    kfs_above_cmd_p2   = 3,  /* pos_rounds[2] */
    kfs_above_cmd_p3   = 4   /* pos_rounds[3] */
} Kfs_Above_Cmd;

extern volatile Kfs_Below_Cmd kfs_below_cmd;
extern volatile Kfs_Above_Cmd kfs_above_cmd;

/* main_lift 分段计时(ms)，pX_pY = pX->pY，debugger 可实时改 */
typedef struct {
    volatile uint32_t t_up_p0_p1;
    volatile uint32_t t_up_p1_p2;
    volatile uint32_t t_up_p2_p3;
    volatile uint32_t t_up_p3_p4;
    volatile uint32_t t_down_p0_p1;
    volatile uint32_t t_down_p1_p2;
    volatile uint32_t t_down_p2_p3;
    volatile uint32_t t_down_p3_p4;
} Main_Lift_Timing_Param;

extern volatile Main_Lift_Timing_Param main_lift_timing_param;
extern volatile float kfs_below_auto_speed;   /* 全自动模式 below 速度指令 */
extern volatile float kfs_above_auto_speed;   /* 全自动模式 above 速度指令 */

extern Three_kfs_position three_kfs_position;
extern Kfs_spin_position kfs_spin_position;
extern Main_lift_position main_lift_position;
extern Kfs_flexible kfs_flexible;

extern float main_lift_Initpos;
extern float kfs_spin_Initpos;
extern float three_kfs_Initpos;


extern float kfs_above_pid_param[PID_PARAMETER_NUM];
extern float kfs_below_pid_param[PID_PARAMETER_NUM];


typedef struct _Kfs_Module{
    StructureModule super_struct; 
    
                             
} Kfs_Module;

extern Kfs_Module Kfs;

extern DJI_MotorModule kfs_above;  
extern DJI_MotorModule kfs_below;  

extern DM_MotorModule main_lift;
extern DM_MotorModule kfs_spin;
extern DM_MotorModule three_kfs;

void manual_kfs_function(void);
void Initpos_Get(void);
void kfs_three_kfs_spin_main_lift_pos_init(void);

#endif
