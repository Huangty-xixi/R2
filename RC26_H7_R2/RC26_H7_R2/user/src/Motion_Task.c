#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"
#include "Process_Flow.h"
#include "app_zone1.h"
#include "app_zone2.h"
#include "app_zone3.h"
#include "clamp_head_ctrl.h"
#include "yaw_heading_ctrl.h"
#include "app_zone3_prep.h"
#include "r1_link.h"
#include "app_init.h"

Control_mode control_mode;
Remote_mode remote_mode;
Flow_mode flow_mode = flow_none;
App_flow_mode app_flow_mode = app_flow_none;

/* ====== 静态变量声明 ====== */

/* 通信触发防抖500ms延时 */
static uint32_t s_trigger_settle_ms   = 0;
static uint8_t  s_trigger_settle_cmd  = 0;
static uint8_t  s_match_auto_active   = 0;  /* 比赛自动运行激活标志 */


#if APP_MATCH_IS_ARENA || APP_MATCH_SKILL_Z12 || APP_MATCH_SKILL_Z3
static void app_flow_start_match(void)
{
	s_match_auto_active = 1U;

#if APP_MATCH_SKILL_Z12
	/* Z12比赛：启动一区(双圈) → 二区(高地) */
	AppZone1_Start(); app_flow_mode = app_flow_zone1;
#elif APP_MATCH_SKILL_Z3
	/* Z3比赛：启动预备阶段 → 三区(放一个弹药) */
	AppZone3Prep_Start();
	app_flow_mode = app_flow_zone3_prep;
#else /* APP_MATCH_IS_ARENA */
	/* 对抗赛：启动一区(单圈) → 二区(全场) → 三区(扔R1) */
	AppZone1_Start(); app_flow_mode = app_flow_zone1;
#endif
}
#endif

/* ====== Motion_Task main loop ====== */

static Control_mode s_motion_prev_control_mode = remote_control;
static uint8_t s_ch7_prev_high = 0U;

static uint8_t rc_bit_minmax_decode(uint16_t ch_val)
{
	if (ch_val <= 500u) return 0u;
	if (ch_val >= 1500u) return 1u;
	return 2u;
}

/* ====== Motion_Task 内部辅助函数 ====== */

/* 重置所有 zone / flow / mission（遥控 & 急停复用） */
static void motion_enter_remote(void)
{
	Process_Flow_ResetAll();
	flow_mode = flow_none;
	app_flow_mode = app_flow_none;
	app_zone2_mission_clear();
	app_zone1_mission_clear();
	AppZone3_Reset();
	AppZone3Prep_Reset();
	s_trigger_settle_ms = 0;
}

/* CH6 + CH7 → chassis / weapon / lift / kfs */
static void motion_remote_decode(uint8_t ch6_bit, uint8_t ch7_bit)
{
	uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);

	if ((ch6_bit <= 1u) && (ch7_bit <= 1u))
	{
		switch (mode_code)
		{
		case 0u:
			remote_mode = chassis_mode;
			break;
		case 1u:
			remote_mode = weapon_mode;
			break;
		case 2u:
			remote_mode = lift_mode;
			break;
		case 3u:
			remote_mode = kfs_mode;
			break;
		default:
			break;
		}
	}
}

/* CH7 上升/下降沿检测 → 触发/取消 Zone1 */
static void motion_ch7_edge(uint8_t ch7_bit)
{
#if CH7_MATCH
	(void)ch7_bit;
	return;
#endif
	uint8_t ch7_high = (uint8_t)((control_mode == full_auto_control) && (ch7_bit == 1u));

	if (control_mode != full_auto_control) return;

	if ((ch7_high == 0U) && (s_ch7_prev_high != 0U))
	{
		app_zone1_mission_clear();
		if (flow_mode == flow_camera_debug)
		{
			flow_mode = flow_none;
		}
		if (app_flow_mode == app_flow_zone1)
		{
		app_flow_mode = app_flow_none;
	}
	}
	else if ((ch7_high != 0U) && (s_ch7_prev_high == 0U))
	{
		if (flow_mode == flow_none && app_flow_mode == app_flow_none)
		{
			app_flow_mode = app_flow_zone1;
		}
	}
	s_ch7_prev_high = ch7_high;
}

/* 二区轮询 + R1 使命 + done → zone3 */
static void motion_poll_zone2(void)
{
	if (R1Link_HasNewMission()) { R1Link_TakeAndApply(); }
	app_zone2_poll();
	if (app_zone2_is_done() != 0U)
	{
#if APP_MATCH_IS_ARENA
		/* 对抗赛二区完成 → 三区(扔R1弹药) */
		AppZone3_Start();
		app_flow_mode = app_flow_zone3;
#else
		/* Z12比赛二区完成 → 结束 */
		s_match_auto_active = 0U;
		app_flow_mode = app_flow_none;
#endif
	}
}

/* 一区轮询 + idle→start + done→zone2 */
static void motion_poll_zone1(void)
{
	if ((AppZone1_IsBusy()   == 0U)
		&& (AppZone1_IsDone()   == 0U)
		&& (AppZone1_IsFailed() == 0U))
	{
		AppZone1_Start();
	}
	app_zone1_poll();
	if ((AppZone1_IsBusy() == 0U)
		&& ((AppZone1_IsDone() != 0U) || (AppZone1_IsFailed() != 0U)))
	{
		if (s_match_auto_active != 0U)
		{
			/* 比赛自动运行：一区完 → 二区 */
			app_flow_mode = app_flow_zone2;
		}
		else
		{
			app_flow_mode = app_flow_none;
		}
	}
}

/* 三区预备轮询 */
static void motion_poll_zone3_prep(void)
{
	if (app_flow_mode != app_flow_zone3_prep)
		app_flow_mode = app_flow_zone3_prep;
	AppZone3Prep_Run();
	if ((AppZone3Prep_IsActive() == 0U) &&
		(AppZone3Prep_IsDone() != 0U))
		app_flow_mode = app_flow_zone3;  /* prep成功: AppZone3_Start()已设此值 */
	else if ((AppZone3Prep_IsActive() == 0U) &&
		(AppZone3Prep_IsFailed() != 0U))
		app_flow_mode = app_flow_none;   /* prep失败: 不启动zone3 */
}

/* 三区轮询 */
static void motion_poll_zone3(void)
{
	if (app_flow_mode != app_flow_zone3)
		app_flow_mode = app_flow_zone3;
	AppZone3_Run();
	if ((AppZone3_IsActive() == 0U) &&
		((AppZone3_IsDone() != 0U) || (AppZone3_IsFailed() != 0U)))
		app_flow_mode = app_flow_none;
}

/* cmd_count + 单通道路由 + 宏展开 */
static void motion_route_channels(uint8_t r_ch5_low, uint8_t r_ch5_high, uint8_t r_ch7, uint8_t r_ch6)
{
	uint8_t cmd_count = (uint8_t)(r_ch6 + r_ch5_low + r_ch5_high + r_ch7);

	if (cmd_count == 0u)
	{
		s_trigger_settle_ms = 0;
	}
	else if (cmd_count == 1u)
	{
		/* 普通单通道触发执行 */
		s_trigger_settle_ms = 0;
		if      (r_ch5_low != 0u)  CH5_LOW_ACTION
		else if (r_ch5_high != 0u)  CH5_HIGH_ACTION
		else if (r_ch7 != 0u)    CH7_ACTION
		else                       CH6_ACTION
	}
#if APP_MATCH_IS_ARENA || APP_MATCH_SKILL_Z12 || APP_MATCH_SKILL_Z3
	else
	{
		app_flow_start_match();
	}
#endif
}

/* ====== Motion_Task 主循环 ====== */

void Motion_Task(void const * argument)
{
	for (;;)
	{
		/* 通道解码 */
		uint8_t ch6_bit = rc_bit_minmax_decode(RCctrl.CH6);
		uint8_t ch7_bit = rc_bit_minmax_decode(RCctrl.CH7);

		/* CH8 → 控制模式 */
		if (RCctrl.CH8 < 500)
		{
			control_mode = emergency_stop_mode;
		}
		else if (RCctrl.CH8 > 500 && RCctrl.CH8 < 1500)
		{
			control_mode = full_auto_control;
		}
		else
		{
			control_mode = remote_control;
		}

		/* 退出全自动时复位夹爪 */
		if (((control_mode == remote_control) || (control_mode == emergency_stop_mode))
			&& (s_motion_prev_control_mode == full_auto_control))
		{
			ClampHeadCtrl_Init();
			app_zone1_mission_clear();
			app_flow_mode = app_flow_none;
		}
		s_motion_prev_control_mode = control_mode;

		/* CH7 边沿 → 一区 */
		motion_ch7_edge(ch7_bit);

		/* 模式分发 */
		switch (control_mode)
		{
		case remote_control:
			motion_enter_remote();
			motion_remote_decode(ch6_bit, ch7_bit);
			break;

		case emergency_stop_mode:
			motion_enter_remote();
			break;

		case full_auto_control: {
#if MOTION_YAW_TUNE_CH5
			uint8_t r_ch5_low = 0u;
			uint8_t r_ch5_high = 0u;
#else
			uint8_t ch5_bit = rc_bit_minmax_decode(RCctrl.CH5);
			/* CH5低位=取KFS 高位=放KFS；CH7=一区 */
			uint8_t r_ch5_low = (uint8_t)(ch5_bit == 0u);
			uint8_t r_ch5_high = (uint8_t)(ch5_bit == 1u);
#endif
			uint8_t r_ch7 = (uint8_t)(ch7_bit == 1u);
			uint8_t r_ch6 = (uint8_t)(ch6_bit == 1u);

			remote_mode = chassis_mode;

#if MOTION_YAW_TUNE_CH5
			/* CH5: 转固定角度；>1500转90°，<500转-90°（防重复触发） */
			{
				static uint16_t ch5_prev = 1024U;
				uint16_t ch5_now = RCctrl.CH5;

				if (ch5_now >= 1500U && ch5_prev < 1500U)
				{
					YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_right_90);
				}
				else if (ch5_now <= 500U && ch5_prev > 500U)
				{
					YawHeadingCtrl_PostCommand(yaw_heading_cmd_turn_left_90);
				}
				ch5_prev = ch5_now;
			}
#endif

			/* 应用层轮询（按优先级） */
			if      (app_flow_mode == app_flow_zone2)       motion_poll_zone2();
			else if (app_flow_mode == app_flow_zone1)       motion_poll_zone1();
			else if ((app_flow_mode == app_flow_zone3_prep)
				|| (AppZone3Prep_IsActive() != 0U))   motion_poll_zone3_prep();
			else if ((app_flow_mode == app_flow_zone3)
				|| (AppZone3_IsActive() != 0U))       motion_poll_zone3();
			else if (flow_mode == flow_none)
				motion_route_channels(r_ch5_low, r_ch5_high, r_ch7, r_ch6);
			break;
		}
		}

		osDelay(1);
	}
}
