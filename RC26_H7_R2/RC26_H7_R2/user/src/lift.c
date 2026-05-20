#include "lift.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h>
#include "Motion_Task.h"
#include "dm_motor.h"
#include "chassis.h"


//抬升
Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;//（左）
DM_MotorModule R2_lift_motor_right;//（右）

R2_lift_mode r2_lift_mode = fall;

//收缩
DJI_MotorModule flexible_motor1;//（左）
DJI_MotorModule flexible_motor2;//（右）

uint8_t lift_has_stopped = 0;
uint8_t lift_running = 0;
int    lift_stop_mode  = 0;
uint8_t lift_fall_fast = 0;
uint8_t lift_rise_fast = 0;

void lift_clear_stop_latch(void)
{
	lift_has_stopped = 0U;
	lift_running = 0U;
}

static uint8_t lift_motor_fault(const DM_MotorModule *m)
{
	return (uint8_t)((m->state == OVER_CUR) || (m->state == OVER_LOAD) || (m->state == MOS_HOT));
}

static uint8_t lift_motor_speed_stall(const DM_MotorModule *m)
{
	const float abs_spd = fabsf(m->speed_w);

	if (abs_spd > LIFT_STALL_SPEED_ABN_TH)
	{
		return 1U;
	}
	if (abs_spd < LIFT_STALL_SPEED_TH)
	{
		return 1U;
	}
	return 0U;
}

static uint8_t lift_any_fault_detected(void)
{
	return (uint8_t)((lift_motor_fault(&R2_lift_motor_left) != 0U) ||
	                   (lift_motor_fault(&R2_lift_motor_right) != 0U));
}

static uint8_t lift_any_speed_stall_detected(void)
{
	return (uint8_t)((lift_motor_speed_stall(&R2_lift_motor_left) != 0U) ||
	                   (lift_motor_speed_stall(&R2_lift_motor_right) != 0U));
}

static void lift_stall_check_update(R2_lift_mode cmd_mode, uint16_t *stop_cnt)
{
	if (lift_any_fault_detected() != 0U)
	{
		lift_has_stopped = 1;
		lift_stop_mode = (int)cmd_mode;
		lift_fall_fast = 0U;
		lift_rise_fast = 0U;
		lift_running = 0U;
		*stop_cnt = 0U;
		return;
	}

	if (*stop_cnt < 0xFFFFU)
	{
		(*stop_cnt)++;
	}

	if (lift_running != 0U)
	{
		if ((*stop_cnt >= LIFT_STALL_CONFIRM_CNT) &&
		    (lift_any_speed_stall_detected() != 0U))
		{
			lift_has_stopped = 1;
			lift_stop_mode = (int)cmd_mode;
			lift_fall_fast = 0U;
			lift_rise_fast = 0U;
			lift_running = 0U;
			*stop_cnt = 0U;
		}
		return;
	}

	/* 未转起来也已在端点：避免一直大力矩堵转（如上电已在底部） */
	if ((*stop_cnt >= (LIFT_CMD_IGNORE_CNT + LIFT_STALL_CONFIRM_CNT)) &&
	    (lift_any_speed_stall_detected() != 0U))
	{
		lift_has_stopped = 1;
		lift_stop_mode = (int)cmd_mode;
		lift_fall_fast = 0U;
		lift_rise_fast = 0U;
		lift_running = 0U;
		*stop_cnt = 0U;
	}
}

//活动电机状态
FlexibleMotorCmd flex_cmd = FLEX_CMD_NONE;
FlexibleMotorState4 flex_state4 = FLEX_ST_RETRACTED;
uint16_t flex_input_prev = CH2_MID;
uint8_t flex_seen_move = 0;
uint8_t flex_stop_cnt = 0;
float flexible_motor_PID_input;

float flexible_motor1_pid_param[PID_PARAMETER_NUM] = {5.0f,0.4f,0.2f,1,500.0f,10000.0f};
float flexible_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.4f,0.2f,1,500.0f,10000.0f};

volatile float lift_rise_fast_left_v  = 3.0f;
volatile float lift_rise_fast_kp = 0.15f;
volatile float lift_rise_fast_kd = 0.15f;
volatile float lift_rise_fast_left_t  = 3.6f;
volatile float lift_rise_fast_right_v  = -3.4f;
volatile float lift_rise_fast_right_t  = -3.9f;

void lift_init()
{
	r2_lift_mode = fall;
	lift_has_stopped = 0;
	lift_running = 0;
	lift_stop_mode = 0;
	lift_fall_fast = 0;
	lift_rise_fast = 0;

	flex_cmd = FLEX_CMD_NONE;
	flex_state4 = FLEX_ST_RETRACTED;
	flex_input_prev = CH2_MID;
	flex_seen_move = 0;
	flex_stop_cnt = 0;
	flexible_motor_PID_input = 0.0f;
}

void manual_lift_function(void)
{
	if (control_mode == remote_control)
	{
		Chassis.Chassis_Stop(&Chassis);
		DJIset_motor_data(&hfdcan1, 0x200, 0,0,0,0);
	}

	if (control_mode == remote_control)
	{
		if (RCctrl.CH3 >= 1500)
		{
			r2_lift_mode = raise;
		}
		else if (RCctrl.CH3 <= 500)
		{
			r2_lift_mode = fall;
		}
		else if (RCctrl.CH4 <= 500)
		{
			r2_lift_mode = fall;
			lift_fall_fast = 1;
		}
		else if (RCctrl.CH4 >= 1500)
		{
			r2_lift_mode = raise;
			lift_rise_fast = 1;
		}

		flexible_motor_update_command(RCctrl.CH2);
	}

	flexible_motor_state_machine_step();

	flexible_motor1.PID_Calculate(&flexible_motor1, flexible_motor_PID_input);
	flexible_motor2.PID_Calculate(&flexible_motor2, -flexible_motor_PID_input);

	DJIset_motor_data(&hfdcan2, 0X200, 0.0f, 0.0f, flexible_motor1.pid_spd.Output, flexible_motor2.pid_spd.Output);

	static int last_r2_lift_mode = -1;
	static uint16_t lift_stop_check_cnt = 0U;

	if (r2_lift_mode != last_r2_lift_mode)
	{
		last_r2_lift_mode = r2_lift_mode;
		lift_has_stopped = 0;
		lift_running = 0;
		lift_stop_check_cnt = 0;
	}

	if (lift_has_stopped)
	{
		if (lift_stop_mode == fall)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f, -0.7f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, 0, 0, 0.5f, 1.0f);
		}
		else if (lift_stop_mode == raise)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f, 2.1f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, 0, 0, 0.5f, -3.0f);
		}
	}
	else if (r2_lift_mode == fall)
	{
		if (lift_fall_fast == 0)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -1.0f, 0, 0.30f, -1.3f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, 1.0f, 0, 0.30f, 1.5f);
		}
		else
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -6.0f, 0, 0.30f, -3.0f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, 6.0f, 0, 0.30f, 3.2f);
		}

		if (fabsf(R2_lift_motor_left.speed_w) > 2.0f || fabsf(R2_lift_motor_right.speed_w) > 2.0f)
		{
			lift_running = 1;
		}

		lift_stall_check_update(fall, &lift_stop_check_cnt);
	}
	else if (r2_lift_mode == raise)
	{
		if (lift_rise_fast == 0U)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 2.8f, 0, 0.11f, 4.5f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, -3.3f, 0, 0.11f, -4.8f);
		}
		else
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, lift_rise_fast_left_v, lift_rise_fast_kp, lift_rise_fast_kd, lift_rise_fast_left_t);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0, lift_rise_fast_right_v, lift_rise_fast_kp, lift_rise_fast_kd, lift_rise_fast_right_t);
		}

		if (fabsf(R2_lift_motor_left.speed_w) > 2.0f || fabsf(R2_lift_motor_right.speed_w) > 2.0f)
		{
			lift_running = 1;
		}

		lift_stall_check_update(raise, &lift_stop_check_cnt);
	}
}

void flexible_motor_update_command(uint16_t ch_value)
{
	flex_cmd = FLEX_CMD_NONE;

	if (ch_value >= 1500 && flex_input_prev <= 1500)
	{
		flex_cmd = FLEX_CMD_RETRACT;
	}
	else if (ch_value <= 500 && flex_input_prev >= 500)
	{
		flex_cmd = FLEX_CMD_EXTEND;
	}

	flex_input_prev = ch_value;
}

void flexible_motor_state_machine_step(void)
{
	int rpm1 = abs((int)flexible_motor1.speed_rpm);
	int rpm2 = abs((int)flexible_motor2.speed_rpm);

	if (flex_cmd == FLEX_CMD_EXTEND)
	{
		flex_state4 = FLEX_ST_EXTENDING;
		flex_seen_move = 0;
		flex_stop_cnt = 0;
	}
	else if (flex_cmd == FLEX_CMD_RETRACT)
	{
		flex_state4 = FLEX_ST_RETRACTING;
		flex_seen_move = 0;
		flex_stop_cnt = 0;
	}

	switch (flex_state4)
	{
	case FLEX_ST_EXTENDING:
		flexible_motor_PID_input = FLEX_CMD_EXTEND_PWM;
		if (rpm1 > FLEX_RUN_THR_RPM || rpm2 > FLEX_RUN_THR_RPM)
		{
			flex_seen_move = 1;
		}
		if (flex_seen_move && rpm1 < FLEX_STOP_THR_RPM && rpm2 < FLEX_STOP_THR_RPM)
		{
			if (++flex_stop_cnt >= FLEX_STOP_CNT_MAX)
			{
				flexible_motor_PID_input = 0.0f;
				flex_state4 = FLEX_ST_EXTENDED;
				flex_stop_cnt = 0;
			}
		}
		else
		{
			flex_stop_cnt = 0;
		}
		break;

	case FLEX_ST_RETRACTING:
		flexible_motor_PID_input = FLEX_CMD_RETRACT_PWM;
		if (rpm1 > FLEX_RUN_THR_RPM || rpm2 > FLEX_RUN_THR_RPM)
		{
			flex_seen_move = 1;
		}
		if (flex_seen_move && rpm1 < FLEX_STOP_THR_RPM && rpm2 < FLEX_STOP_THR_RPM)
		{
			if (++flex_stop_cnt >= FLEX_STOP_CNT_MAX)
			{
				flexible_motor_PID_input = 0.0f;
				flex_state4 = FLEX_ST_RETRACTED;
				flex_stop_cnt = 0;
			}
		}
		else
		{
			flex_stop_cnt = 0;
		}
		break;

	case FLEX_ST_EXTENDED:
		flexible_motor_PID_input = 0.0f;
		break;

	case FLEX_ST_RETRACTED:
		if (r2_lift_mode == raise)
		{
			flexible_motor_PID_input = 0.0f;
		}
		else
		{
			flexible_motor_PID_input = 0.0f;
		}
		break;

	default:
		flexible_motor_PID_input = 0.0f;
		break;
	}
}
