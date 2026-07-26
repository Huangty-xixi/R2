#ifndef __LIFT_H__
#define __LIFT_H__

#include "global.h"
#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"

/* 抬升方向（fall=0, raise=1?? */
typedef enum
{
    fall = 0,
    raise = 1,
} R2_lift_mode;

/* 抬升 DM 到位判定 */
#define LIFT_RUN_SPEED_THRESH_RAD_S   (2.0f)
#define LIFT_STOP_SPEED_THRESH_RAD_S  (1.8f)
#define LIFT_STALL_SPEED_ABN_TH       (29.0f)
#define LIFT_CMD_IGNORE_CNT           (40U)
#define LIFT_STOP_DEBOUNCE_CNT        (300U)
#define LIFT_STOP_LOW_STREAK_MIN      (5U)
#define LIFT_STOP_STALL_LATCH_CNT     (50U)
#define LIFT_FAULT_DEBOUNCE_CNT       (10U)

/** LiftMotorTune: volatile MIT params for DM lift motors */
typedef struct {
    float fall_v_l;
    float fall_v_r;
    float fall_kd;
    float fall_t_l;
    float fall_t_r;
    float fall_fast_v_l;
    float fall_fast_v_r;
    float fall_fast_kd;
    float fall_fast_t_l;
    float fall_fast_t_r;
    float rise_v_l;
    float rise_v_r;
    float rise_kd;
    float rise_t_l;
    float rise_t_r;
    float rise_fast_v_l;
    float rise_fast_v_r;
    float rise_fast_kp;
    float rise_fast_kd;
    float rise_fast_t_l;
    float rise_fast_t_r;
    float stop_fall_kd;
    float stop_fall_t_l;
    float stop_fall_t_r;
    float stop_rise_kd;
    float stop_rise_t_l;
    float stop_rise_t_r;
} LiftMotorTune;
extern R2_lift_mode r2_lift_mode;
extern volatile LiftMotorTune g_lift_tune;

/* 抬升电机 */
#define R2_LIFT_MOTOR_LEFT_ID           0x05
#define R2_LIFT_MOTOR_LEFT_CMD_ID       R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_FEEDBACK_ID  R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_MASTER_ID    0x10

#define R2_LIFT_MOTOR_RIGHT_ID          0x06
#define R2_LIFT_MOTOR_RIGHT_CMD_ID      R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_FEEDBACK_ID R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_MASTER_ID   R2_LIFT_MOTOR_LEFT_MASTER_ID

/* master 抬升动作?? */
#define MASTER_LIFT_UPDOWN_BIT    (1U << 0)
#define MASTER_LIFT_FALL_FAST_BIT (1U << 1)
#define MASTER_LIFT_RISE_FAST_BIT (1U << 2)

typedef struct _Lift_Module
{
    StructureModule super_struct;
} Lift_Module;

extern Lift_Module Lift;
extern DM_MotorModule R2_lift_motor_left;
extern DM_MotorModule R2_lift_motor_right;

extern int lift_stop_mode;
extern uint8_t lift_has_stopped;
extern uint8_t lift_fall_fast;
extern uint8_t lift_rise_fast;
extern uint8_t lift_running;

void lift_clear_stop_latch(void);
void lift_init(void);
void lift_motor_run_output(void);
void manual_lift_function(void);

#endif
