/**
 * @file app_zone2.c
 * @brief 二区梅花桩：上台面、走路径、邻格取矿粉、换桩对齐车头与层高（与 app_zone2.h 一致）。
 *
 * 分层：z2_exec_* 执行层（直接调用 nav/Process/摆头）；z2_sched_* 调度层（主状态与任务决策）；z2_step_* 记录当前脚本步骤。
 */
#include "app_zone2.h"
#include "yaw_heading_ctrl.h"
#include "map.h"
#include "Motion_Task.h"
#include "Process_Flow.h"
#include "kfs.h"
#include "main.h"

#include <string.h>

#define Z2_KFS_ACTIVE_J_NONE 0xFFU

static uint8_t z2_exec_motion_gate_ok(void)
{
    if (control_mode != full_auto_control)
        return 0U;
    if (app_flow_mode == app_flow_zone2)
        return 1U;
    if (app_flow_mode != app_flow_none)
        return 0U;
    return (uint8_t)(flow_mode == flow_none);
}

static uint8_t user_pile_center_map_m(uint8_t user_pile, float *cx_m, float *cy_m)
{
    return map_zone2_pile_center_m(APP_ZONE2_RED_SIDE, user_pile, cx_m, cy_m);
}

static uint16_t user_pile_height_mm(uint8_t user_pile)
{
    return map_zone2_pile_height_mm(user_pile);
}

/* 桩顶高度 mm → 层档 0/1/2（200/400/600），其它值当 0 档 */
static uint8_t pile_height_mm_to_tier(uint16_t pile_height_mm)
{
    if (pile_height_mm == 200U)
        return 0U;
    if (pile_height_mm == 400U)
        return 1U;
    if (pile_height_mm == 600U)
        return 2U;
    return 0U;
}

/* 站立桩与邻格矿粉桩的桩顶档比较（200/400/600 → tier 0/1/2）；仅区分高→低 / 低→高 */
static app_zone2_get_kfs_rel_t app_zone2_get_kfs_rel(uint8_t user_station_pile, uint8_t user_kfs_pile)
{
    uint8_t ts = pile_height_mm_to_tier(user_pile_height_mm(user_station_pile));
    uint8_t tk = pile_height_mm_to_tier(user_pile_height_mm(user_kfs_pile));

    if (ts > tk)
    {
        return APP_ZONE2_GET_KFS_HIGH_TO_LOW;
    }
    if (ts < tk)
    {
        return APP_ZONE2_GET_KFS_LOW_TO_HIGH;
    }
    /* 同档不应出现；若表数据/任务异常导致相等，兜底为高→低 */
    return APP_ZONE2_GET_KFS_HIGH_TO_LOW;
}

/** 地面预备取件：桩2 低取高 p3；桩1/桩3 最高档 p4 */
static app_zone2_get_kfs_rel_t app_zone2_ground_prep_get_kfs_rel(uint8_t prep_pile)
{
    if (prep_pile == 2U)
        return APP_ZONE2_GET_KFS_LOW_TO_HIGH;
    return APP_ZONE2_GET_KFS_GROUND_HIGHEST;
}

static uint8_t piles_adjacent(uint8_t pile_a, uint8_t pile_b) // 判断两个桩是否相邻
{
    uint8_t ra = (uint8_t)((pile_a - 1U) / 3U);
    uint8_t ca = (uint8_t)((pile_a - 1U) % 3U);
    uint8_t rb = (uint8_t)((pile_b - 1U) / 3U);
    uint8_t cb = (uint8_t)((pile_b - 1U) % 3U);
    uint8_t dr = (uint8_t)((ra > rb) ? (ra - rb) : (rb - ra));
    uint8_t dc = (uint8_t)((ca > cb) ? (ca - cb) : (cb - ca));
    return (uint8_t)((dr + dc) == 1U);
}

/*
 * 邻格场向：本区格心 dx/dy。红区 +x 向右，蓝区 +x 向左，故蓝区判左右必须对 dx 取反。
 * LEFT=+90°，RIGHT=-90°（见 yaw_heading_ctrl）。
 */
static app_zone2_field_dir_t field_dir_between_user_piles(uint8_t pile_from, uint8_t pile_to)
{
    float cx_from;
    float cy_from;
    float cx_to;
    float cy_to;
    float dx;
    float dy;

    if (!user_pile_center_map_m(pile_from, &cx_from, &cy_from))
        return APP_ZONE2_FIELD_FACE_SKIP;
    if (!user_pile_center_map_m(pile_to, &cx_to, &cy_to))
        return APP_ZONE2_FIELD_FACE_SKIP;

    dx = cx_to - cx_from;
    dy = cy_to - cy_from;
#if !APP_ZONE2_RED_SIDE
    dx = -dx;
#endif

    if (dy > 0.f)
        return APP_ZONE2_FIELD_FRONT;
    if (dy < 0.f)
        return APP_ZONE2_FIELD_BACK;
    if (dx < 0.f)
        return APP_ZONE2_FIELD_LEFT;
    if (dx > 0.f)
        return APP_ZONE2_FIELD_RIGHT;
    return APP_ZONE2_FIELD_FACE_SKIP;
}

static app_zone2_field_dir_t field_dir_opposite(app_zone2_field_dir_t d)
{
    switch (d)
    {
        case APP_ZONE2_FIELD_FRONT:
            return APP_ZONE2_FIELD_BACK;
        case APP_ZONE2_FIELD_BACK:
            return APP_ZONE2_FIELD_FRONT;
        case APP_ZONE2_FIELD_LEFT:
            return APP_ZONE2_FIELD_RIGHT;
        case APP_ZONE2_FIELD_RIGHT:
            return APP_ZONE2_FIELD_LEFT;
        default:
            return APP_ZONE2_FIELD_FACE_SKIP;
    }
}

/* ==================== 执行层：下发动作命令并轮询完成 ==================== */

typedef enum {
    Z2_EXEC_BUSY = 0,
    Z2_EXEC_DONE = 1,
} z2_exec_result_t;

typedef enum {
    Z2_STEP_NONE = APP_ZONE2_DEBUG_STEP_NONE,
    Z2_STEP_ENTRY_NAV = APP_ZONE2_DEBUG_STEP_ENTRY_NAV,
    Z2_STEP_GROUND_PREP_NAV = APP_ZONE2_DEBUG_STEP_GROUND_PREP_NAV,
    Z2_STEP_GROUND_PREP_GET = APP_ZONE2_DEBUG_STEP_GROUND_PREP_GET,
    Z2_STEP_ENTER_MOUNT = APP_ZONE2_DEBUG_STEP_ENTER_MOUNT,
    Z2_STEP_NAV_TO_PILE = APP_ZONE2_DEBUG_STEP_NAV_TO_PILE,
    Z2_STEP_FACE_KFS = APP_ZONE2_DEBUG_STEP_FACE_KFS,
    Z2_STEP_GET_KFS = APP_ZONE2_DEBUG_STEP_GET_KFS,
    Z2_STEP_FACE_NEXT = APP_ZONE2_DEBUG_STEP_FACE_NEXT,
    Z2_STEP_RECENTER = APP_ZONE2_DEBUG_STEP_RECENTER,
    Z2_STEP_STAIR = APP_ZONE2_DEBUG_STEP_STAIR,
    Z2_STEP_LAST_FACE = APP_ZONE2_DEBUG_STEP_LAST_FACE,
    Z2_STEP_LAST_RECENTER = APP_ZONE2_DEBUG_STEP_LAST_RECENTER,
    Z2_STEP_GROUND_DISMOUNT = APP_ZONE2_DEBUG_STEP_GROUND_DISMOUNT,
    Z2_STEP_UPSLOPE = APP_ZONE2_DEBUG_STEP_UPSLOPE,
    Z2_STEP_EXIT_NAV = APP_ZONE2_DEBUG_STEP_EXIT_NAV,
    Z2_STEP_DONE = APP_ZONE2_DEBUG_STEP_DONE,
} z2_step_kind_t;

static app_zone2_mission_t s_mission; // 任务
static uint8_t s_has_mission;        // 是否有任务
static uint8_t s_robot_tier;         // 机器人层档

typedef enum { // 状态机
    Z2_IDLE = 0,           // 空闲
    Z2_DONE,               // 完成
    Z2_ENTRY_NAV,          // 入口预定位：main_lift p3 + 导航至固定点
    Z2_ENTRY_WAIT_NAV,     /* 入口预定位导航等待 */
    Z2_GROUND_PREP,        /* 二区预备：桩1/3 最高档 p4，桩2 低取高 p3；不含上桩 */
    Z2_MAIN_PREP_NAV,      /* main_flow：导航至桩2预备位 (3.0,2.6) */
    Z2_MAIN_PREP_WAIT_NAV,
    Z2_DEFERRED_KFS_GET,   /* 桩2+桩3：上桩后邻格取桩3 */
    Z2_ENTER_UP,           /* 进入上桩 */
    Z2_ENTER_NAV,          // 进入导航
    Z2_ENTER_WAIT_NAV,     // 进入导航等待
    Z2_KFS_TURN,           // 取件转向
    Z2_KFS_RUN,            // 取件运行
    Z2_PATH_NEXT_PILE,     /* path 上一桩 → 下一桩：摆头 + 按层高上/下桩（无“台阶”语义） */
    Z2_LAST_DOWN_TURN,     /* path end pile 10/12/6: face, recenter, dismount to ground */
    Z2_LAST_DOWN_DISMOUNT,
    Z2_LAST_UPSLOPE,       /* run Process_UpSlope after landing */
    Z2_LAST_EXIT_NAV,      /* navigate to final zone2 exit point */
    Z2_LAST_EXIT_WAIT_NAV,
} z2_major_t;

static z2_major_t s_major;       // 状态机
static uint8_t s_path_idx;       // 路径索引
static uint16_t s_kfs_done_mask; // 取件完成掩码

static uint8_t s_sent_mount;     // 已发上桩
static uint8_t s_sent_dismount;  // 已发下桩
static uint8_t s_sent_turn;      // 已发转向
static uint8_t s_sent_getkfs;    // 已发取件
static uint8_t s_sent_upslope;   // sent upslope flow

static uint8_t s_face_dir_step_done;       /* 「request_face_field_dir」子步是否跑完；PATH_NEXT_PILE 换桩前摆头用，0=未做完 */
static uint8_t s_path_next_recenter_done;  /* PATH_NEXT：摆头后回中 from 桩心 */
static uint8_t s_kfs_face_step_done;         /* KFS_TURN：在邻格矿粉桩摆头（下发 Vx，与取件流程并行） */
static uint8_t s_kfs_recenter_done;        /* KFS_TURN：回到当前 path 桩桩心导航已结束（Vy/Vw） */
static uint8_t s_last_down_recenter_done;  /* LAST_DOWN_TURN：摆头后回末桩桩心 */
static uint8_t s_nav_leg_running;          /* 1=本段导航已 arm，未 ARRIVED/失败前不下一段 */
static uint32_t s_nav_leg_session;         /* 本段 session，peek 须一致，等同单独调试一段 nav */
static uint32_t s_nav_leg_fail_rc;         /* 最近一次导航段失败码；NONE 表示无失败 */

static uint8_t s_kfs_j;                    // 取件索引
static uint8_t s_kfs_active_j;             /* 当前 Process_GetKFS 序号，尾部未结束前为 j；Z2_KFS_ACTIVE_J_NONE=无 */
static app_zone2_get_kfs_rel_t s_kfs_active_rel; /* 尾部推进用，与 s_kfs_active_j 成对 */
static uint8_t s_enter_up_mount_enabled;     /* 1=进 Z2_ENTER_UP 要上桩再导航；0=在 ENTER_UP 里直接转导航（见 case） */
static uint8_t s_last_exit_pile;           /* Z2_LAST_DOWN_*：path 末桩示意图号 */
static app_zone2_field_dir_t s_last_face_dir_cmd; /* 最近一次 request_face_field_dir，供 turn_dir 与实发一致 */
static z2_step_kind_t s_step_kind;         /* 当前脚本步骤，供调度/Watch 对齐 */
static uint32_t s_step_seq;                /* 脚本步骤切换序号 */
static uint8_t s_step_from_pile;
static uint8_t s_step_to_pile;
static uint8_t s_step_kfs_pile;
static uint8_t s_step_kfs_idx;
static int16_t s_step_tier_delta;
static app_zone2_field_dir_t s_step_face_dir;

typedef enum {
    Z2_PREP_IDLE = 0,
    Z2_PREP_NAV,
    Z2_PREP_WAIT_NAV,
    Z2_PREP_PICK,
} z2_prep_phase_t;

static z2_prep_phase_t s_prep_phase;
static uint8_t s_prep_pick_pile;
static uint8_t s_prep_pick_j;
static uint8_t s_prep_deferred_kfs_j;

/** 主状态切换后先等 APP_ZONE2_STEP_PRE_DELAY_MS 再执行该状态逻辑 */
static z2_major_t s_step_pre_delay_major;
static uint32_t s_step_pre_delay_tick;

static void app_zone2_step_pre_delay_reset(void)
{
    s_step_pre_delay_major = (z2_major_t)255;
}

static void app_zone2_step_pre_delay_sync(void)
{
    if (s_step_pre_delay_major != s_major)
    {
        s_step_pre_delay_major = s_major;
        s_step_pre_delay_tick = HAL_GetTick();
    }
}

static uint8_t app_zone2_step_pre_delay_ready(void)
{
#if (APP_ZONE2_STEP_PRE_DELAY_MS == 0U)
    return 1U;
#else
    return (uint8_t)((HAL_GetTick() - s_step_pre_delay_tick) >= (uint32_t)APP_ZONE2_STEP_PRE_DELAY_MS);
#endif
}

#if APP_ZONE2_DBG_FAKE_MISSION
/** 调试专用假数据：mission_clear 后置 1，下一轮无任务时 poll 内自动 apply */
static uint8_t s_dbg_fake_rearm = 1U;
#endif
#if !APP_ZONE2_DBG_FAKE_MISSION
static uint8_t s_no_mission_tail_armed;
#endif

static void z2_step_reset(void)
{
    s_step_kind = Z2_STEP_NONE;
    s_step_seq = 0U;
    s_step_from_pile = 0U;
    s_step_to_pile = 0U;
    s_step_kfs_pile = 0U;
    s_step_kfs_idx = 0U;
    s_step_tier_delta = 0;
    s_step_face_dir = APP_ZONE2_FIELD_FACE_SKIP;
}

static void z2_step_set(z2_step_kind_t kind, uint8_t from_pile, uint8_t to_pile, uint8_t kfs_pile,
                        uint8_t kfs_idx, int16_t tier_delta, app_zone2_field_dir_t face_dir)
{
    if (s_step_kind != kind || s_step_from_pile != from_pile || s_step_to_pile != to_pile ||
        s_step_kfs_pile != kfs_pile || s_step_kfs_idx != kfs_idx ||
        s_step_tier_delta != tier_delta || s_step_face_dir != face_dir)
    {
        s_step_seq++;
    }

    s_step_kind = kind;
    s_step_from_pile = from_pile;
    s_step_to_pile = to_pile;
    s_step_kfs_pile = kfs_pile;
    s_step_kfs_idx = kfs_idx;
    s_step_tier_delta = tier_delta;
    s_step_face_dir = face_dir;
}

volatile app_zone2_debug_t g_app_zone2_debug = {
    0U,
    APP_ZONE2_DEBUG_NAV_POLL_RC_NONE,
};

static int16_t user_pile_tier_delta(uint8_t user_pile);

static void app_zone2_debug_snapshot_runtime(void)
{
    const odom_nav_goto_status_t *st = odom_nav_goto_peek_last_status();
    uint32_t busy_mask = 0U;

    if (Process_UpStairs_IsBusy() != 0U)
        busy_mask |= 1U;
    if (Process_DownStairs_IsBusy() != 0U)
        busy_mask |= 2U;
    if (Process_GetKFS_IsBusy() != 0U)
        busy_mask |= 4U;

    g_app_zone2_debug.nav_fail_rc = s_nav_leg_fail_rc;
    g_app_zone2_debug.nav_session = s_nav_leg_session;
    g_app_zone2_debug.odom_session = odom_nav_target.session_id;
    g_app_zone2_debug.nav_armed = (uint32_t)odom_nav_goto_is_armed();
    g_app_zone2_debug.nav_leg_running = (uint32_t)s_nav_leg_running;
    g_app_zone2_debug.override_axis_mask = (uint32_t)process_flow_chassis_override.axis_mask;
    g_app_zone2_debug.override_priority = (uint32_t)process_flow_chassis_override.priority;
    g_app_zone2_debug.override_priority_vx = (uint32_t)process_flow_chassis_override.priority_vx;
    g_app_zone2_debug.override_priority_vy = (uint32_t)process_flow_chassis_override.priority_vy;
    g_app_zone2_debug.override_priority_vw = (uint32_t)process_flow_chassis_override.priority_vw;
    g_app_zone2_debug.process_busy_mask = busy_mask;
    g_app_zone2_debug.step_kind = (uint32_t)s_step_kind;
    g_app_zone2_debug.step_seq = s_step_seq;
    g_app_zone2_debug.step_from_pile = (uint32_t)s_step_from_pile;
    g_app_zone2_debug.step_to_pile = (uint32_t)s_step_to_pile;
    g_app_zone2_debug.step_kfs_pile = (uint32_t)s_step_kfs_pile;
    g_app_zone2_debug.step_kfs_idx = (uint32_t)s_step_kfs_idx;
    g_app_zone2_debug.step_tier_delta = (int32_t)s_step_tier_delta;
    g_app_zone2_debug.step_face_dir = (uint32_t)s_step_face_dir;
    g_app_zone2_debug.nav_target_x_m = odom_nav_target.x_m;
    g_app_zone2_debug.nav_target_y_m = odom_nav_target.y_m;
    g_app_zone2_debug.override_vx = process_flow_chassis_override.vx;
    g_app_zone2_debug.override_vy = process_flow_chassis_override.vy;
    g_app_zone2_debug.override_vw = process_flow_chassis_override.vw;
    if (st != 0)
    {
        g_app_zone2_debug.nav_dist_m = st->distance_to_target_m;
        g_app_zone2_debug.nav_vy_cmd = st->vy_cmd;
        g_app_zone2_debug.nav_vw_cmd = st->vw_cmd;
    }
}

static void app_zone2_debug_set_poll_major(void)
{
    g_app_zone2_debug.poll_major = (uint32_t)s_major;
    app_zone2_debug_snapshot_runtime();
}

static void app_zone2_debug_record_nav_poll(app_zone2_nav_poll_result_t rc)
{
    g_app_zone2_debug.nav_poll_rc = (uint32_t)rc;
    app_zone2_debug_snapshot_runtime();
}

// 获取任务路径长度
static uint8_t mission_path_len(void)
{
    uint8_t i;
    if (s_mission.path_n > 0U && s_mission.path_n <= APP_ZONE2_MAX_PATH)
        return s_mission.path_n;
    for (i = 0U; i < APP_ZONE2_MAX_PATH; i++)
    {
        if (s_mission.path[i] == 0U)
            break;
    }
    return i;
}

static uint8_t mission_kfs_len(void)
{
    uint8_t j;
    if (s_mission.kfs_n > 0U && s_mission.kfs_n <= APP_ZONE2_MAX_KFS)
        return s_mission.kfs_n;
    for (j = 0U; j < APP_ZONE2_MAX_KFS; j++)
    {
        if (s_mission.kfs[j] == 0U)
            break;
    }
    return j;
}

/*
 * 方案 A：二区每段导航 = disarm → set_target → 等到 ARRIVED 或 TIMEOUT 再进入下一步（TIMEOUT 与到点同等）。
 * run 仍只由 chassis service_tick 驱动；状态机每拍 poll，未结束不进入下一步。
 */

static uint8_t z2_exec_process_motion_idle(void)
{
    uint8_t up_down_idle = (uint8_t)(Process_UpStairs_IsBusy() == 0U &&
                                      Process_DownStairs_IsBusy() == 0U &&
                                      Process_UpSlope_IsBusy() == 0U);

    /* 前进已结束但流程仍跑中：允许摆头/回中等继续推 Process_GetKFS 尾部 */
    if (Process_GetKFS_IsBusy() != 0U && Process_GetKFS_IsChassisForwardDone() != 0U)
        return up_down_idle;
    return (uint8_t)(up_down_idle && (Process_GetKFS_IsBusy() == 0U));
}

/** 导航前只释放 Vy/Vw，Vx 仍由流程控可兼顾摆头 */
static void z2_exec_release_chassis_for_nav(void)
{
    Process_Flow_ClearChassisOverrideAxes((uint8_t)(PROCESS_FLOW_CHASSIS_OVERRIDE_VY |
                                                    PROCESS_FLOW_CHASSIS_OVERRIDE_VW));
}

static void z2_exec_nav_abort(void)
{
    odom_nav_goto_disarm();
    s_nav_leg_running = 0U;
    s_nav_leg_session = 0U;
    s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
}

static app_zone2_nav_poll_result_t z2_exec_nav_peek(void)
{
    app_zone2_nav_poll_result_t nav_rc;

    nav_rc = odom_nav_goto_peek_last_run_result();
    app_zone2_debug_record_nav_poll(nav_rc);
    if (s_nav_leg_running == 0U || odom_nav_target.session_id != s_nav_leg_session)
        return ODOM_NAV_GOTO_ERR_DISARMED;
    return nav_rc;
}

/* 开始一段到地图坐标的导航；返回 0=未 arm（Process 仍忙等，下拍重试） */
static uint8_t z2_exec_nav_start_xy(float xm, float ym)
{
    if (z2_exec_process_motion_idle() == 0U)
        return 0U;

    z2_exec_release_chassis_for_nav();
    z2_exec_nav_abort();
    odom_nav_goto_set_target(xm, ym);
    s_nav_leg_session = odom_nav_target.session_id;
    s_nav_leg_running = 1U;
    s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
    app_zone2_debug_snapshot_runtime();
    return 1U;
}

/* 开始一段到桩心的导航；返回 0=未 arm（图无效、Process 仍忙等，下拍重试） */
static uint8_t z2_exec_nav_start_pile(uint8_t pile)
{
    float xm;
    float ym;

    if (!user_pile_center_map_m(pile, &xm, &ym))
        return 0U;
    return z2_exec_nav_start_xy(xm, ym);
}

/* 1=本段仍在进行；0=本段结束（ARRIVED 或 TIMEOUT，可进下一步）；ODOM/BAD_CONFIG 则结束任务 */
static uint8_t z2_exec_nav_poll_leg(void)
{
    app_zone2_nav_poll_result_t nav_rc;

    if (s_nav_leg_running == 0U)
        return 1U;

    nav_rc = z2_exec_nav_peek();
    if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
    {
        s_nav_leg_running = 0U;
        s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
        app_zone2_debug_snapshot_runtime();
        return 0U;
    }
    /* 兜底：已 disarm 且距离在容差内，视为本段到点（与 odom_nav_goto 正常 ARRIVED 一致） */
    if (nav_rc == ODOM_NAV_GOTO_ERR_DISARMED && odom_nav_goto_is_armed() == 0U)
    {
        const odom_nav_goto_status_t *st = odom_nav_goto_peek_last_status();

        if (st != 0 &&
            st->distance_to_target_m <= g_odom_nav_goto_tune.position_tolerance_m)
        {
            s_nav_leg_running = 0U;
            s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
            app_zone2_debug_snapshot_runtime();
            return 0U;
        }
    }
    if (nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT)
    {
        s_nav_leg_running = 0U;
        s_nav_leg_fail_rc = (uint32_t)nav_rc;
        odom_nav_goto_disarm();
        app_zone2_debug_snapshot_runtime();
        return 0U;
    }
    if ((nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
        (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
    {
        s_nav_leg_running = 0U;
        s_nav_leg_fail_rc = (uint32_t)nav_rc;
        odom_nav_goto_disarm();
        s_major = Z2_DONE;
        app_zone2_debug_snapshot_runtime();
        return 1U;
    }
    return 1U;
}


/* 目标示意图桩 user_pile 的层档 ? 当前机器人 s_robot_tier；>0 要上桩，<0 要下桩 */
static int16_t user_pile_tier_delta(uint8_t user_pile)
{
    uint8_t want = pile_height_mm_to_tier(user_pile_height_mm(user_pile));
    return (int16_t)want - (int16_t)s_robot_tier;
}


static void z2_exec_reset_act_flags(void)
{
    s_sent_mount = 0U;    // 上桩发送标志
    s_sent_dismount = 0U; // 下桩发送标志
    s_sent_turn = 0U;     // 转向发送标志
    s_sent_upslope = 0U;
}

/** 1=门控未过本步阻塞；0=摆头命令已发出，可进导航等后续 */
static uint8_t z2_exec_face_substep(app_zone2_field_dir_t fd, uint8_t *done)
{
    if (*done != 0U)
        return 0U;
    if (!z2_exec_motion_gate_ok())
        return 1U;

    s_last_face_dir_cmd = fd;
    YawHeadingCtrl_RunFieldDir(fd);
    s_sent_turn = 0U;
    *done = 1U;
    return 0U;
}

static uint8_t z2_exec_nav_recenter_substep(uint8_t pile, uint8_t *done)
{
    if (*done != 0U)
        return 0U;

    if (s_nav_leg_running == 0U)
    {
        if (!z2_exec_motion_gate_ok())
            return 1U;
        if (z2_exec_nav_start_pile(pile) == 0U)
            return 1U;
        return 1U;
    }

    if (z2_exec_nav_poll_leg() != 0U)
        return 1U;

    *done = 1U;
    return 0U;
}


static z2_exec_result_t z2_exec_one_stair_step(int16_t cha)
{
    uint8_t up = (uint8_t)(cha > 0);
    uint8_t *sent = up ? &s_sent_mount : &s_sent_dismount;

    if (*sent == 0U)
    {
        if (z2_exec_motion_gate_ok())
        {
            z2_exec_nav_abort();
            if (up) {
                Process_UpStairs();
                *sent = 1U;
            } else {
                Process_DownStairs();
                *sent = 1U;
            }
        }
    }
    else if (z2_exec_motion_gate_ok())
    {
        if (up)
        {
            Process_UpStairs();
            if (Process_UpStairs_IsBusy() != 0U)
                return Z2_EXEC_BUSY;
            s_robot_tier++;
        }
        else
        {
            Process_DownStairs();
            if (Process_DownStairs_IsBusy() != 0U)
                return Z2_EXEC_BUSY;
            s_robot_tier--;
        }
        z2_exec_nav_abort();
        *sent = 0U;
        return Z2_EXEC_BUSY;
    }
    return Z2_EXEC_BUSY;
}

static z2_exec_result_t z2_exec_ground_dismount(void)
{
    if (s_sent_dismount == 0U)
    {
        if (z2_exec_motion_gate_ok())
        {
            z2_exec_nav_abort();
            Process_DownStairs();
            s_sent_dismount = 1U;
        }
    }
    else if (z2_exec_motion_gate_ok())
    {
        Process_DownStairs();
        if (Process_DownStairs_IsBusy() != 0U)
            return Z2_EXEC_BUSY;
        s_sent_dismount = 0U;
        return Z2_EXEC_DONE;
    }
    return Z2_EXEC_BUSY;
}

static z2_exec_result_t z2_exec_upslope(void)
{
    if (!z2_exec_motion_gate_ok())
        return Z2_EXEC_BUSY;

    if (s_sent_upslope == 0U)
    {
        z2_exec_nav_abort();
        s_sent_upslope = 1U;
    }

    Process_UpSlope();
    if (Process_UpSlope_IsBusy() != 0U)
        return Z2_EXEC_BUSY;

    s_sent_upslope = 0U;
    z2_exec_nav_abort();
    return Z2_EXEC_DONE;
}

/** @param allow_early_after_chassis 1=梅花：前进段结束可早返 DONE，尾部由 z2_get_kfs_tail_service 推进 */
static z2_exec_result_t z2_exec_get_kfs(uint8_t station_pile, uint8_t kfs_j, uint8_t allow_early_after_chassis,
                                        app_zone2_get_kfs_rel_t rel)
{

    if (s_sent_getkfs == 0U)
    {
        if (!z2_exec_motion_gate_ok())
            return Z2_EXEC_BUSY;
        if (Process_GetKFS_IsBusy() != 0U)
            return Z2_EXEC_BUSY;
        if (YawHeadingCtrl_IsBusy() != 0U)
            return Z2_EXEC_BUSY;

        Process_GetKFS(rel);
        s_kfs_active_rel = rel;
        s_kfs_active_j = kfs_j;
        s_sent_getkfs = 1U;
        return Z2_EXEC_BUSY;
    }
    if (!z2_exec_motion_gate_ok())
        return Z2_EXEC_BUSY;

    Process_GetKFS(rel);

    if (allow_early_after_chassis != 0U && Process_GetKFS_IsChassisForwardDone() != 0U)
    {
        s_sent_getkfs = 0U;
        return Z2_EXEC_DONE;
    }
    if (Process_GetKFS_IsBusy() != 0U)
        return Z2_EXEC_BUSY;

    s_kfs_done_mask |= (uint16_t)(1U << kfs_j);
    s_kfs_active_j = Z2_KFS_ACTIVE_J_NONE;
    s_sent_getkfs = 0U;
    return Z2_EXEC_DONE;
}

/** 前进段尾段：每拍推进 Process_GetKFS，本段结束后置 kfs_done_mask */
static void z2_get_kfs_tail_service(void)
{
    if (s_kfs_active_j == Z2_KFS_ACTIVE_J_NONE)
        return;
    /* KFS_RUN 早退后由 z2_exec_get_kfs 推进，仍用同一 rel 调 Process_GetKFS */
    if (s_sent_getkfs != 0U)
        return;
    /* 急停/ResetAll 后流程回 idle 且 active_j 未清：不置 done 退出 */
    if (Process_GetKFS_IsBusy() == 0U && get_kfs_step == get_kfs_step_idle)
    {
        s_kfs_active_j = Z2_KFS_ACTIVE_J_NONE;
        return;
    }

    if (Process_GetKFS_IsBusy() == 0U)
    {
        s_kfs_done_mask |= (uint16_t)(1U << s_kfs_active_j);
        s_kfs_active_j = Z2_KFS_ACTIVE_J_NONE;
        return;
    }

    Process_GetKFS(s_kfs_active_rel);
    if (Process_GetKFS_IsBusy() == 0U)
    {
        s_kfs_done_mask |= (uint16_t)(1U << s_kfs_active_j);
        s_kfs_active_j = Z2_KFS_ACTIVE_J_NONE;
    }
}

static z2_exec_result_t z2_exec_enter_mount(void)
{
    if (s_sent_mount == 0U)
    {
        if (z2_exec_motion_gate_ok())
        {
            z2_exec_nav_abort();
            Process_UpStairs();
            s_sent_mount = 1U;
        }
        return Z2_EXEC_BUSY;
    }
    if (!z2_exec_motion_gate_ok())
        return Z2_EXEC_BUSY;

    Process_UpStairs();
    if (Process_UpStairs_IsBusy() != 0U)
        return Z2_EXEC_BUSY;

    s_sent_mount = 0U;
    s_enter_up_mount_enabled = 0U;
    s_face_dir_step_done = 0U;
    s_robot_tier = pile_height_mm_to_tier(user_pile_height_mm(s_mission.path[s_path_idx]));
    return Z2_EXEC_DONE;
}

static z2_exec_result_t z2_exec_face_beat(app_zone2_field_dir_t fd)
{
    if (s_sent_turn == 0U)
    {
        if (z2_exec_motion_gate_ok())
        {
            s_last_face_dir_cmd = fd;
            YawHeadingCtrl_RunFieldDir(fd);
            s_sent_turn = 1U;
        }
        return Z2_EXEC_BUSY;
    }
    if (!z2_exec_motion_gate_ok())
        return Z2_EXEC_BUSY;

    if (YawHeadingCtrl_IsBusy() != 0U)
        return Z2_EXEC_BUSY;

    s_sent_turn = 0U;
    return Z2_EXEC_DONE;
}

/* ==================== 调度层：每拍推进一步状态机执行意图 ==================== */

static int8_t z2_sched_pick_kfs_on_pile(uint8_t pile, uint8_t *out_j);

static float z2_ground_prep_x_m(uint8_t pile)
{
    if (pile == 1U)
        return APP_ZONE2_GROUND_PREP_PILE1_X_M;
    if (pile == 3U)
        return APP_ZONE2_GROUND_PREP_PILE3_X_M;
    return APP_ZONE2_ENTRY_NAV_X_M;
}

/* 第一次优化：预查预备桩号，入口直接导航到目标桩前，避免先去桩2再折返 */
static uint8_t z2_ground_prep_first_pile(void)
{
    uint8_t j;
    if (z2_sched_pick_kfs_on_pile(2U, &j) == 0)
        return 2U;
    if (z2_sched_pick_kfs_on_pile(1U, &j) == 0)
        return 1U;
    if (z2_sched_pick_kfs_on_pile(3U, &j) == 0)
        return 3U;
    return 2U; /* 都没有，默认桩2 */
}

static void z2_ground_prep_reset(void)
{
    s_prep_phase = Z2_PREP_IDLE;
    s_prep_pick_pile = 0U;
    s_prep_pick_j = 0U;
    s_prep_deferred_kfs_j = Z2_KFS_ACTIVE_J_NONE;
}

/** 预备取件优先级：桩2 > 桩1 > 桩3；桩2+桩3 时只取2并延后 j3 */
static uint8_t z2_ground_prep_select(uint8_t *out_pile, uint8_t *out_j)
{
    uint8_t j1;
    uint8_t j2;
    uint8_t j3;

    s_prep_deferred_kfs_j = Z2_KFS_ACTIVE_J_NONE;

    if (z2_sched_pick_kfs_on_pile(2U, &j2) == 0)
    {
        *out_pile = 2U;
        *out_j = j2;
        if (z2_sched_pick_kfs_on_pile(3U, &j3) == 0)
            s_prep_deferred_kfs_j = j3;
        return 1U;
    }
    if (z2_sched_pick_kfs_on_pile(1U, &j1) == 0)
    {
        *out_pile = 1U;
        *out_j = j1;
        return 1U;
    }
    if (z2_sched_pick_kfs_on_pile(3U, &j3) == 0)
    {
        *out_pile = 3U;
        *out_j = j3;
        return 1U;
    }
    return 0U;
}

/** 预备结束：导航桩2预备位 → 上桩 → path[0] 梅花主循环 */
static void z2_sched_begin_main_flow(void)
{
    main_lift_position = main_lift_p3; /* 上桩2前置位，与底盘导航并发 */
    s_path_idx = 0U;
    s_enter_up_mount_enabled = 1U;
    z2_exec_reset_act_flags();
    s_major = Z2_MAIN_PREP_NAV;
}

static void z2_sched_entry_nav(void)
{
    uint8_t prep_pile = z2_ground_prep_first_pile();
    float x_m = z2_ground_prep_x_m(prep_pile);
    z2_step_set(Z2_STEP_ENTRY_NAV, 0U, prep_pile, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    main_lift_position = main_lift_p3;
    if (z2_exec_nav_start_xy(x_m, APP_ZONE2_ENTRY_NAV_Y_M) == 0U)
        return;
    s_major = Z2_ENTRY_WAIT_NAV;
}

static void z2_sched_entry_wait_nav(void)
{
    z2_step_set(Z2_STEP_ENTRY_NAV, 0U, 0U, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_nav_poll_leg() != 0U)
        return;
    z2_ground_prep_reset();
    s_major = Z2_GROUND_PREP;
}

static void z2_sched_ground_kfs_prep(void)
{
    switch (s_prep_phase)
    {
        case Z2_PREP_IDLE:
            if (z2_ground_prep_select(&s_prep_pick_pile, &s_prep_pick_j) == 0U)
            {
                z2_sched_begin_main_flow();
                return;
            }
            s_prep_phase = Z2_PREP_NAV;
            /* fall through */
        case Z2_PREP_NAV:
            z2_step_set(Z2_STEP_GROUND_PREP_NAV, 0U, s_prep_pick_pile, s_mission.kfs[s_prep_pick_j],
                        s_prep_pick_j, 0, APP_ZONE2_FIELD_FACE_SKIP);
            if (z2_exec_nav_start_xy(z2_ground_prep_x_m(s_prep_pick_pile), APP_ZONE2_GROUND_PREP_Y_M) == 0U)
                return;
            s_prep_phase = Z2_PREP_WAIT_NAV;
            break;

        case Z2_PREP_WAIT_NAV:
            z2_step_set(Z2_STEP_GROUND_PREP_NAV, 0U, s_prep_pick_pile, s_mission.kfs[s_prep_pick_j],
                        s_prep_pick_j, 0, APP_ZONE2_FIELD_FACE_SKIP);
            if (z2_exec_nav_poll_leg() != 0U)
                return;
            s_sent_getkfs = 0U;
            s_prep_phase = Z2_PREP_PICK;
            break;

        case Z2_PREP_PICK:
            z2_step_set(Z2_STEP_GROUND_PREP_GET, 0U, s_prep_pick_pile, s_mission.kfs[s_prep_pick_j],
                        s_prep_pick_j, 0, APP_ZONE2_FIELD_FACE_SKIP);
            if (z2_exec_get_kfs(s_prep_pick_pile, s_prep_pick_j, 0U,
                                app_zone2_ground_prep_get_kfs_rel(s_prep_pick_pile)) == Z2_EXEC_BUSY)
                return;
            z2_sched_begin_main_flow();
            break;

        default:
            z2_sched_begin_main_flow();
            break;
    }
}

static void z2_sched_main_prep_nav(void)
{
    z2_step_set(Z2_STEP_ENTRY_NAV, 0U, 2U, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_nav_start_xy(APP_ZONE2_ENTRY_NAV_X_M, APP_ZONE2_ENTRY_NAV_Y_M) == 0U)
        return;
    s_major = Z2_MAIN_PREP_WAIT_NAV;
}

static void z2_sched_main_prep_wait_nav(void)
{
    z2_step_set(Z2_STEP_ENTRY_NAV, 0U, 2U, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_nav_poll_leg() != 0U)
        return;
    z2_exec_reset_act_flags();
    s_major = Z2_ENTER_UP;
}

static int8_t z2_sched_pick_kfs_on_pile(uint8_t pile, uint8_t *out_j)
{
    uint8_t j; // 取件索引
    for (j = 0U; j < mission_kfs_len(); j++) // 遍历取件列表
    {
        if (((s_kfs_done_mask >> j) & 1U) != 0U) // 该取件任务已经完成，跳过
            continue;
        if (s_mission.kfs[j] != pile) // 该取件任务不在当前桩号，跳过
            continue;
        *out_j = j; // 返回取件索引
        return 0;   // 返回 0 表示成功
    }
    return -1; // 返回 -1 表示失败
}

static int8_t z2_sched_pick_kfs_adjacent(uint8_t *out_j)
{
    uint8_t st = s_mission.path[s_path_idx];
    uint8_t j; // 取件索引
    for (j = 0U; j < mission_kfs_len(); j++)
    {
        if (((s_kfs_done_mask >> j) & 1U) != 0U) // 该取件任务已经完成，跳过
            continue;
        if (s_kfs_active_j == j && Process_GetKFS_IsBusy() != 0U)
            continue;
        if (!piles_adjacent(st, s_mission.kfs[j])) // 若当前路径桩号和取件桩号不相邻，跳过
            continue;
        *out_j = j; // 返回取件索引
        return 0;   // 返回 0 表示成功
    }
    return -1;
}

/** 当前 path 桩邻格矿粉取完：path_idx++，或进入换桩/末桩下地面/结束 */
static void z2_sched_after_station_kfs_done(void)
{
    uint8_t const plen = mission_path_len();
    uint8_t const cur_pile = s_mission.path[s_path_idx];
    uint8_t const path_last_pile = (plen > 0U) ? s_mission.path[plen - 1U] : 0U;

    s_path_idx++;
    if (s_path_idx < plen)
    {
        s_major = Z2_PATH_NEXT_PILE;
        z2_exec_reset_act_flags();
        s_face_dir_step_done = 0U;
        s_path_next_recenter_done = 0U;
        s_kfs_face_step_done = 0U;
        s_kfs_recenter_done = 0U;
        z2_exec_nav_abort();
        z2_step_set(Z2_STEP_FACE_NEXT, cur_pile, s_mission.path[s_path_idx], 0U, 0U,
                    user_pile_tier_delta(s_mission.path[s_path_idx]), APP_ZONE2_FIELD_FACE_SKIP);
        return;
    }

    if (plen > 0U && cur_pile == path_last_pile &&
        user_pile_height_mm(cur_pile) == 200U &&
        (cur_pile == 10U || cur_pile == 12U || cur_pile == 6U))
    {
        s_last_exit_pile = cur_pile;
        z2_exec_reset_act_flags();
        s_face_dir_step_done = 0U;
        s_last_down_recenter_done = 0U;
        z2_exec_nav_abort();
        s_major = Z2_LAST_DOWN_TURN;
        return;
    }

    z2_step_set(Z2_STEP_DONE, cur_pile, cur_pile, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    s_major = Z2_DONE;
}

static void z2_sched_deferred_kfs_get(void)
{
    uint8_t kfs_pile = s_mission.kfs[s_kfs_j];

    z2_step_set(Z2_STEP_GROUND_PREP_GET, 2U, 2U, kfs_pile, s_kfs_j, 0, APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_get_kfs(2U, s_kfs_j, 0U, app_zone2_get_kfs_rel(2U, kfs_pile)) == Z2_EXEC_BUSY)
        return;
    z2_step_set(Z2_STEP_NAV_TO_PILE, 0U, s_mission.path[s_path_idx], 0U, 0U, 0,
                APP_ZONE2_FIELD_FACE_SKIP);
    s_major = Z2_ENTER_NAV;
}

static void z2_sched_enter_up(void)
{
    if (s_enter_up_mount_enabled == 0U)
    {
        z2_step_set(Z2_STEP_NAV_TO_PILE, 0U, s_mission.path[s_path_idx], 0U, 0U, 0,
                    APP_ZONE2_FIELD_FACE_SKIP);
        s_major = Z2_ENTER_NAV;
        return;
    }
    z2_step_set(Z2_STEP_ENTER_MOUNT, 0U, s_mission.path[s_path_idx], 0U, 0U,
                user_pile_tier_delta(s_mission.path[s_path_idx]), APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_enter_mount() == Z2_EXEC_BUSY)
        return;

    if (s_prep_deferred_kfs_j != Z2_KFS_ACTIVE_J_NONE)
    {
        s_kfs_j = s_prep_deferred_kfs_j;
        s_prep_deferred_kfs_j = Z2_KFS_ACTIVE_J_NONE;
        s_sent_getkfs = 0U;
        s_major = Z2_DEFERRED_KFS_GET;
        return;
    }

    z2_step_set(Z2_STEP_NAV_TO_PILE, 0U, s_mission.path[s_path_idx], 0U, 0U, 0,
                APP_ZONE2_FIELD_FACE_SKIP);
    s_major = Z2_ENTER_NAV;
}

static void z2_sched_enter_nav(void)
{
    z2_step_set(Z2_STEP_NAV_TO_PILE, 0U, s_mission.path[s_path_idx], 0U, 0U,
                user_pile_tier_delta(s_mission.path[s_path_idx]), APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_nav_start_pile(s_mission.path[s_path_idx]) == 0U)
        return;
    s_major = Z2_ENTER_WAIT_NAV;
}

static void z2_sched_enter_wait_nav(void)
{
    z2_step_set(Z2_STEP_NAV_TO_PILE, 0U, s_mission.path[s_path_idx], 0U, 0U,
                user_pile_tier_delta(s_mission.path[s_path_idx]), APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_nav_poll_leg() != 0U)
        return;
    s_major = Z2_KFS_TURN;
}

static void z2_sched_kfs_turn(void)
{
    uint8_t j;
    uint8_t station = s_mission.path[s_path_idx];
    app_zone2_field_dir_t fd;

    if (z2_sched_pick_kfs_adjacent(&j) != 0)
    {
        if (s_kfs_active_j != Z2_KFS_ACTIVE_J_NONE)
            return;
        z2_sched_after_station_kfs_done();
        return;
    }

    if (s_kfs_j != j)
    {
        s_kfs_face_step_done = 0U;
        s_kfs_recenter_done = 0U;
    }
    s_kfs_j = j;

    fd = field_dir_between_user_piles(station, s_mission.kfs[j]);

    if (s_kfs_face_step_done == 0U)
    {
        z2_step_set(Z2_STEP_FACE_KFS, station, station, s_mission.kfs[j], j, 0, fd);
        if (z2_exec_face_substep(fd, &s_kfs_face_step_done) != 0U)
            return;
    }

    if (s_kfs_face_step_done != 0U && s_kfs_recenter_done == 0U)
    {
        z2_step_set(Z2_STEP_RECENTER, station, station, s_mission.kfs[j], j, 0, fd);
        if (z2_exec_nav_recenter_substep(station, &s_kfs_recenter_done) != 0U)
            return;
    }

    if (s_kfs_face_step_done == 0U || s_kfs_recenter_done == 0U)
        return;

    if (YawHeadingCtrl_IsBusy() != 0U)
        return;

    s_kfs_face_step_done = 0U;
    s_kfs_recenter_done = 0U;
    z2_step_set(Z2_STEP_GET_KFS, station, station, s_mission.kfs[j], j, 0, fd);
    s_major = Z2_KFS_RUN;
    s_sent_getkfs = 0U;
}

static void z2_sched_kfs_run(void)
{
    z2_step_set(Z2_STEP_GET_KFS, s_mission.path[s_path_idx], s_mission.path[s_path_idx],
                s_mission.kfs[s_kfs_j], s_kfs_j, 0,
                field_dir_between_user_piles(s_mission.path[s_path_idx], s_mission.kfs[s_kfs_j]));
    /* 摆头(Vx)结束后再按回中(Vy)推进，进入 Process_GetKFS chassis_forward 段 */
    if (YawHeadingCtrl_IsBusy() != 0U)
        return;
    if (z2_exec_get_kfs(s_mission.path[s_path_idx], s_kfs_j, 1U,
                        app_zone2_get_kfs_rel(s_mission.path[s_path_idx], s_mission.kfs[s_kfs_j])) == Z2_EXEC_BUSY)
        return;
    s_major = Z2_KFS_TURN;
}

static void z2_sched_path_next_pile(void)
{
    uint8_t from_u = s_mission.path[s_path_idx - 1U];
    uint8_t to_u = s_mission.path[s_path_idx];
    int16_t cha = user_pile_tier_delta(to_u);
    app_zone2_field_dir_t fd_step;

    if (cha < 0)
    {
        main_lift_position = main_lift_p1; /* 主流程下桩前置位，与 face/recenter 并发 */
        kfs_spin_position  = kfs_spin_p1;
    }

    if (s_face_dir_step_done == 0U)
    {
        fd_step = APP_ZONE2_FIELD_BACK;
        if (piles_adjacent(from_u, to_u))
            fd_step = field_dir_between_user_piles(from_u, to_u);
        if (cha < 0)
            fd_step = field_dir_opposite(fd_step);
        z2_step_set(Z2_STEP_FACE_NEXT, from_u, to_u, 0U, 0U, cha, fd_step);
        if (z2_exec_face_substep(fd_step, &s_face_dir_step_done) != 0U)
            return;
    }

    if (s_face_dir_step_done != 0U && s_path_next_recenter_done == 0U)
    {
        z2_step_set(Z2_STEP_RECENTER, from_u, from_u, 0U, 0U, cha, s_last_face_dir_cmd);
        if (z2_exec_nav_recenter_substep(from_u, &s_path_next_recenter_done) != 0U)
            return;
    }

    if (s_face_dir_step_done != 0U && s_path_next_recenter_done != 0U && cha == 0)
    {
        s_face_dir_step_done = 0U;
        s_path_next_recenter_done = 0U;
        z2_step_set(Z2_STEP_NAV_TO_PILE, from_u, to_u, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
        s_major = Z2_ENTER_NAV;
        return;
    }

    if (s_face_dir_step_done == 0U || s_path_next_recenter_done == 0U)
        return;
    if (YawHeadingCtrl_IsBusy() != 0U)
        return;

    z2_step_set(Z2_STEP_STAIR, from_u, to_u, 0U, 0U, cha, s_last_face_dir_cmd);
    if (z2_exec_one_stair_step(cha) == Z2_EXEC_BUSY)
        return;

    if (user_pile_tier_delta(to_u) == 0)
    {
        s_face_dir_step_done = 0U;
        s_path_next_recenter_done = 0U;
        z2_step_set(Z2_STEP_NAV_TO_PILE, from_u, to_u, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
        s_major = Z2_ENTER_NAV;
    }
}

static void z2_sched_last_down_turn(void)
{
    app_zone2_field_dir_t fd = APP_ZONE2_FIELD_BACK;

    main_lift_position = main_lift_p1; /* 下桩前置位，与底盘 face/recenter 并发 */
    kfs_spin_position  = kfs_spin_p1;

    if (s_last_exit_pile == 6U)
        fd = field_dir_opposite(field_dir_between_user_piles(5U, 6U));

    if (s_face_dir_step_done == 0U)
    {
        z2_step_set(Z2_STEP_LAST_FACE, s_last_exit_pile, s_last_exit_pile, 0U, 0U, 0, fd);
        if (z2_exec_face_substep(fd, &s_face_dir_step_done) != 0U)
            return;
        return;
    }

    if (s_last_down_recenter_done == 0U)
    {
        z2_step_set(Z2_STEP_LAST_RECENTER, s_last_exit_pile, s_last_exit_pile, 0U, 0U, 0, fd);
        if (z2_exec_nav_recenter_substep(s_last_exit_pile, &s_last_down_recenter_done) != 0U)
            return;
    }

    if (YawHeadingCtrl_IsBusy() != 0U)
        return;

    s_face_dir_step_done = 0U;
    s_last_down_recenter_done = 0U;
    z2_exec_reset_act_flags();
    z2_step_set(Z2_STEP_GROUND_DISMOUNT, s_last_exit_pile, 0U, 0U, 0U, -1, fd);
    s_major = Z2_LAST_DOWN_DISMOUNT;
}

static void z2_sched_last_down_dismount(void)
{
    z2_step_set(Z2_STEP_GROUND_DISMOUNT, s_last_exit_pile, 0U, 0U, 0U, -1, s_last_face_dir_cmd);
    if (z2_exec_ground_dismount() == Z2_EXEC_BUSY)
        return;
    z2_exec_nav_abort();
    s_sent_upslope = 0U;
    g_process_upslope_tune.p1_x_m = PROCESS_UPSLOPE_P1_X_M;
    g_process_upslope_tune.p1_y_m = PROCESS_UPSLOPE_P1_Y_M;
    Process_UpSlope_Reset();
    z2_step_set(Z2_STEP_UPSLOPE, s_last_exit_pile, 0U, 0U, 0U, 0, APP_ZONE2_FIELD_FRONT);
    s_major = Z2_LAST_UPSLOPE;
}

static void z2_sched_last_upslope(void)
{
    app_zone2_field_dir_t exit_dir;
    z2_step_set(Z2_STEP_UPSLOPE, s_last_exit_pile, 0U, 0U, 0U, 0, APP_ZONE2_FIELD_FRONT);
    if (z2_exec_upslope() == Z2_EXEC_BUSY)
        return;
#if APP_ZONE2_RED_SIDE
    exit_dir = APP_ZONE2_FIELD_LEFT;
#else
    exit_dir = APP_ZONE2_FIELD_RIGHT;
#endif
    YawHeadingCtrl_RunFieldDir(exit_dir);
    z2_step_set(Z2_STEP_EXIT_NAV, s_last_exit_pile, 0U, 0U, 0U, 0, exit_dir);
    s_major = Z2_LAST_EXIT_NAV;
}

static void z2_sched_last_exit_nav(void)
{
    z2_step_set(Z2_STEP_EXIT_NAV, s_last_exit_pile, 0U, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_nav_start_xy(APP_ZONE2_EXIT_NAV_X_M, APP_ZONE2_EXIT_NAV_Y_M) == 0U)
        return;
    s_major = Z2_LAST_EXIT_WAIT_NAV;
}

static void z2_sched_last_exit_wait_nav(void)
{
    z2_step_set(Z2_STEP_EXIT_NAV, s_last_exit_pile, 0U, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_nav_poll_leg() != 0U)
        return;
    z2_step_set(Z2_STEP_DONE, s_last_exit_pile, 0U, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    s_major = Z2_DONE;
}

void app_zone2_set_robot_tier(uint8_t tier012) // 设置机器人层档
{
    s_robot_tier = tier012;
}

void app_zone2_mission_clear(void) // 清除任务
{
    memset(&s_mission, 0, sizeof(s_mission));
    s_has_mission = 0U;              // 没有任务
    s_major = Z2_IDLE;               // 空闲
    s_path_idx = 0U;                 // 路径索引
    s_kfs_done_mask = 0U;            // 取件完成掩码
    z2_exec_reset_act_flags();       // 清除上桩等发送标志
    s_sent_getkfs = 0U;              // 清除取件标志
    s_face_dir_step_done = 0U;       // 摆头完成标志
    s_path_next_recenter_done = 0U;
    s_kfs_face_step_done = 0U;
    s_kfs_recenter_done = 0U;
    s_kfs_active_j = Z2_KFS_ACTIVE_J_NONE;
    s_last_down_recenter_done = 0U;
    s_nav_leg_session = 0U;
    s_nav_leg_running = 0U;
    s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
    s_enter_up_mount_enabled = 0U;   // 进入上桩标志
    s_last_exit_pile = 0U;           // 最后一次桩桩号
    z2_ground_prep_reset();
    z2_step_reset();
#if APP_ZONE2_DBG_FAKE_MISSION
    s_dbg_fake_rearm = 1U;
#else
    s_no_mission_tail_armed = 1U;
#endif
    app_zone2_step_pre_delay_reset();
    z2_exec_nav_abort();
    app_zone2_debug_set_poll_major();
}

#if APP_ZONE2_DBG_FAKE_MISSION
static const uint8_t s_dbg_fake_path[] = { APP_ZONE2_DBG_FAKE_PATH_LIST };
static const uint8_t s_dbg_fake_kfs[] = { APP_ZONE2_DBG_FAKE_KFS_LIST };

void app_zone2_debug_fake_mission_get(app_zone2_mission_t *m)
{
    uint8_t i;
    uint8_t pn;
    uint8_t kn;

    if (m == NULL)
        return;
    memset(m, 0, sizeof(*m));
    pn = (uint8_t)APP_ZONE2_DBG_FAKE_PATH_N;
    kn = (uint8_t)APP_ZONE2_DBG_FAKE_KFS_N;
    if (pn > (uint8_t)(sizeof(s_dbg_fake_path) / sizeof(s_dbg_fake_path[0])))
        pn = (uint8_t)(sizeof(s_dbg_fake_path) / sizeof(s_dbg_fake_path[0]));
    if (kn > (uint8_t)(sizeof(s_dbg_fake_kfs) / sizeof(s_dbg_fake_kfs[0])))
        kn = (uint8_t)(sizeof(s_dbg_fake_kfs) / sizeof(s_dbg_fake_kfs[0]));
    m->path_n = pn;
    m->kfs_n = kn;
    for (i = 0U; i < pn; i++)
        m->path[i] = s_dbg_fake_path[i];
    for (i = 0U; i < kn; i++)
        m->kfs[i] = s_dbg_fake_kfs[i];
}
#endif

void app_zone2_mission_apply(const app_zone2_mission_t *m) // 应用任务
{
#if APP_ZONE2_DBG_FAKE_MISSION
    s_dbg_fake_rearm = 0U; /* 正式 apply 后关 R1 假数据，poll 不再自动重灌 */
#endif
    memcpy(&s_mission, m, sizeof(s_mission));
    s_has_mission = 1U;              // 有任务
    s_path_idx = 0U;                 // 路径索引
    s_kfs_done_mask = 0U;            // 取件完成掩码
    z2_exec_reset_act_flags();       // 清除上桩等发送标志
    s_sent_getkfs = 0U;              // 清除取件标志
    s_face_dir_step_done = 0U;
    s_path_next_recenter_done = 0U;
    s_kfs_face_step_done = 0U;
    s_kfs_recenter_done = 0U;
    s_kfs_active_j = Z2_KFS_ACTIVE_J_NONE;
    s_last_down_recenter_done = 0U;
    s_nav_leg_session = 0U;
    s_nav_leg_running = 0U;
    s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
    s_last_exit_pile = 0U;
    z2_ground_prep_reset();
    z2_step_reset();
    z2_exec_nav_abort();

    s_major = Z2_ENTRY_NAV;
    app_zone2_debug_set_poll_major();
}

uint8_t app_zone2_is_busy(void) // 判断是否忙
{
    return (uint8_t)(s_has_mission != 0U && s_major != Z2_IDLE && s_major != Z2_DONE);
}

uint8_t app_zone2_is_done(void) // 判断是否完成
{
    return (uint8_t)(s_has_mission != 0U && s_major == Z2_DONE);
}

static void z2_sched_poll(void)
{
    switch (s_major)
    {
        case Z2_IDLE:
        case Z2_DONE:
            return;

        case Z2_ENTRY_NAV:
            z2_sched_entry_nav();
            break;

        case Z2_ENTRY_WAIT_NAV:
            z2_sched_entry_wait_nav();
            break;

        case Z2_GROUND_PREP:
            z2_sched_ground_kfs_prep();
            break;

        case Z2_MAIN_PREP_NAV:
            z2_sched_main_prep_nav();
            break;

        case Z2_MAIN_PREP_WAIT_NAV:
            z2_sched_main_prep_wait_nav();
            break;

        case Z2_DEFERRED_KFS_GET:
            z2_sched_deferred_kfs_get();
            break;

        case Z2_ENTER_UP:
            z2_sched_enter_up();
            break;

        case Z2_ENTER_NAV:
            z2_sched_enter_nav();
            break;

        case Z2_ENTER_WAIT_NAV:
            z2_sched_enter_wait_nav();
            break;

        case Z2_KFS_TURN:
            z2_sched_kfs_turn();
            break;

        case Z2_KFS_RUN:
            z2_sched_kfs_run();
            break;

        case Z2_PATH_NEXT_PILE:
            z2_sched_path_next_pile();
            break;

        case Z2_LAST_DOWN_TURN:
            z2_sched_last_down_turn();
            break;

        case Z2_LAST_DOWN_DISMOUNT:
            z2_sched_last_down_dismount();
            break;

        case Z2_LAST_UPSLOPE:
            z2_sched_last_upslope();
            break;

        case Z2_LAST_EXIT_NAV:
            z2_sched_last_exit_nav();
            break;

        case Z2_LAST_EXIT_WAIT_NAV:
            z2_sched_last_exit_wait_nav();
            break;

        default:
            break;
    }
}

static void app_zone2_poll_core(void)
{
#if APP_ZONE2_DBG_FAKE_MISSION
    if (s_has_mission == 0U && s_dbg_fake_rearm != 0U)
    {
        app_zone2_mission_t mf;
        app_zone2_debug_fake_mission_get(&mf);
        app_zone2_mission_apply(&mf);
        s_dbg_fake_rearm = 0U;
    }
#else
    if (s_has_mission == 0U && s_no_mission_tail_armed != 0U)
    {
        s_has_mission = 1U;
        s_robot_tier = 0U;
        s_path_idx = 0U;
        s_kfs_done_mask = 0U;
        z2_exec_reset_act_flags();
        s_sent_getkfs = 0U;
        s_face_dir_step_done = 0U;
        s_path_next_recenter_done = 0U;
        s_kfs_face_step_done = 0U;
        s_kfs_recenter_done = 0U;
        s_kfs_active_j = Z2_KFS_ACTIVE_J_NONE;
        s_last_down_recenter_done = 0U;
        s_nav_leg_session = 0U;
        s_nav_leg_running = 0U;
        s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
        s_enter_up_mount_enabled = 0U;
        s_last_exit_pile = 0U;
        z2_ground_prep_reset();
        z2_step_reset();
        app_zone2_step_pre_delay_reset();
        z2_exec_nav_abort();
        s_sent_upslope = 0U;
        s_no_mission_tail_armed = 0U;
        g_process_upslope_tune.p1_x_m = PROCESS_UPSLOPE_P1_X_M;
        g_process_upslope_tune.p1_y_m = PROCESS_UPSLOPE_P1_Y_M;
        Process_UpSlope_Reset();
        z2_step_set(Z2_STEP_UPSLOPE, 0U, 0U, 0U, 0U, 0, APP_ZONE2_FIELD_FRONT);
        s_major = Z2_LAST_UPSLOPE;
    }
#endif
    g_app_zone2_debug.nav_poll_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
    app_zone2_debug_set_poll_major();

    if (!s_has_mission)
        return;

    app_zone2_step_pre_delay_sync();
    if (app_zone2_step_pre_delay_ready() == 0U)
        return;

    z2_get_kfs_tail_service();
    z2_sched_poll();
    app_zone2_debug_set_poll_major();
}

void app_zone2_poll(void)
{
    app_zone2_poll_core();
}
