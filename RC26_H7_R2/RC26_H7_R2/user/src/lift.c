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

Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;
DM_MotorModule R2_lift_motor_right;

R2_lift_mode r2_lift_mode = fall;

uint8_t lift_has_stopped = 0;
uint8_t lift_running = 0;
int    lift_stop_mode  = 0;
uint8_t lift_fall_fast = 0;
uint8_t lift_rise_fast = 0;

static uint16_t s_lift_stop_check_cnt = 0U;
static uint8_t s_lift_stop_low_streak = 0U;
static uint16_t s_lift_stall_at_limit_cnt = 0U;
static uint8_t s_lift_fault_cnt = 0U;

void lift_clear_stop_latch(void)
{
	lift_has_stopped = 0U;
	lift_running = 0U;
	s_lift_stop_check_cnt = 0U;
	s_lift_stop_low_streak = 0U;
	s_lift_stall_at_limit_cnt = 0U;
	s_lift_fault_cnt = 0U;
}

static uint8_t lift_speed_is_running(float speed_w)
{
	return (uint8_t)(fabsf(speed_w) > LIFT_RUN_SPEED_THRESH_RAD_S);
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
	if (abs_spd < LIFT_STOP_SPEED_THRESH_RAD_S)
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

static void lift_latch_stop_now(uint8_t stop_mode)
{
	lift_has_stopped = 1U;
	lift_stop_mode = (int)stop_mode;
	lift_fall_fast = 0U;
	lift_rise_fast = 0U;
	lift_running = 0U;
	s_lift_stop_check_cnt = 0U;
	s_lift_stop_low_streak = 0U;
	s_lift_stall_at_limit_cnt = 0U;
	s_lift_fault_cnt = 0U;
}

static void lift_poll_limit_latch(uint8_t stop_mode)
{
	if (lift_any_fault_detected() != 0U)
	{
		if (s_lift_fault_cnt < 0xFFU)
		{
			s_lift_fault_cnt++;
		}
		if (s_lift_fault_cnt >= LIFT_FAULT_DEBOUNCE_CNT)
		{
			lift_latch_stop_now(stop_mode);
		}
		return;
	}
	s_lift_fault_cnt = 0U;

	if (lift_speed_is_running(R2_lift_motor_left.speed_w) != 0U ||
	    lift_speed_is_running(R2_lift_motor_right.speed_w) != 0U)
	{
		lift_running = 1U;
	}

	if (lift_any_speed_stall_detected() != 0U)
	{
		if (lift_running != 0U)
		{
			if (s_lift_stop_check_cnt < 0xFFFU)
			{
				s_lift_stop_check_cnt++;
			}
		}
		else if (s_lift_stall_at_limit_cnt < 0xFFFU)
		{
			s_lift_stall_at_limit_cnt++;
		}

		uint8_t debounce_ok = 0U;
		if ((lift_running != 0U) &&
		    (s_lift_stop_check_cnt >= LIFT_STOP_DEBOUNCE_CNT))
		{
			debounce_ok = 1U;
		}
		if ((lift_running == 0U) &&
		    (s_lift_stall_at_limit_cnt >= (LIFT_STOP_STALL_LATCH_CNT + LIFT_CMD_IGNORE_CNT)))
		{
			debounce_ok = 1U;
		}

		if (debounce_ok != 0U)
		{
			if (s_lift_stop_low_streak < 255U)
			{
				s_lift_stop_low_streak++;
			}
			if (s_lift_stop_low_streak >= LIFT_STOP_LOW_STREAK_MIN)
			{
				lift_latch_stop_now(stop_mode);
			}
		}
	}
	else
	{
		s_lift_stop_low_streak = 0U;
		if (lift_running == 0U)
		{
			s_lift_stop_check_cnt = 0U;
			s_lift_stall_at_limit_cnt = 0U;
		}
	}
}

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
	s_lift_stop_check_cnt = 0U;
	s_lift_stop_low_streak = 0U;
	s_lift_stall_at_limit_cnt = 0U;
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
	}

	lift_motor_run_output();
}

void lift_motor_run_output(void)
{
	static int last_r2_lift_mode = -1;

	if (r2_lift_mode != last_r2_lift_mode)
	{
		last_r2_lift_mode = r2_lift_mode;
		lift_has_stopped = 0U;
		lift_running = 0U;
		s_lift_stop_check_cnt = 0U;
		s_lift_stop_low_streak = 0U;
		s_lift_stall_at_limit_cnt = 0U;
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

		lift_poll_limit_latch((uint8_t)fall);
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

		lift_poll_limit_latch((uint8_t)raise);
	}
}
