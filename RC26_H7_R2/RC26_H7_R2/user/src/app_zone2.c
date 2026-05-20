/**
 * @file app_zone2.c
 * @brief 二区梅花桩：上台面、走路径、邻格取秘籍、换桩对齐车头与层高（与 app_zone2.h 一致）。
 *
 * 分层：z2_exec_* 执行层（直接调用 nav/Process/摆头）；z2_sched_* 调度层（主状态与任务决策）；z2_step_* 记录当前脚本步骤。
 */
#include "app_zone2.h"
#include "app_yaw_heading_ctrl.h"
#include "map.h"
#include "Motion_Task.h"
#include "Process_Flow.h"
#include "main.h"

#include <string.h>

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

/* 站立桩与邻格秘籍桩的桩顶档比较（200/400/600 → tier 0/1/2）；仅区分高→低 / 低→高 */
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

static uint8_t piles_adjacent(uint8_t pile_a, uint8_t pile_b)//判断两个桩是否相邻
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
 * 邻格场向：本区格心 dx/dy。红区 +x 向右，蓝区 +x 向左，故蓝区判左右须对 dx 取反。
 * LEFT=+90°，RIGHT=-90°（见 app_yaw_heading_ctrl）。
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
    Z2_STEP_ZONE1_KFS_FACE = APP_ZONE2_DEBUG_STEP_ZONE1_KFS_FACE,
    Z2_STEP_ZONE1_KFS_GET = APP_ZONE2_DEBUG_STEP_ZONE1_KFS_GET,
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
    Z2_STEP_DONE = APP_ZONE2_DEBUG_STEP_DONE,
} z2_step_kind_t;

static app_zone2_mission_t s_mission;//任务
static uint8_t s_has_mission;//是否有任务
static uint8_t s_robot_tier;//机器人层高

typedef enum {//状态机
    Z2_IDLE = 0,//空闲
    Z2_DONE,//完成
    Z2_ZONE1_KFS_TURN,//一区取件转弯
    Z2_ZONE1_KFS_RUN,//一区取件运行
    Z2_ENTER_UP,//进入上桩
    Z2_ENTER_NAV,//进入导航
    Z2_ENTER_WAIT_NAV,//进入导航等待
    Z2_KFS_TURN,//取件转弯
    Z2_KFS_RUN,//取件运行
    Z2_PATH_NEXT_PILE,/* path 上一桩 → 下一桩：摆头 + 按层高上/下桩（无“台阶”语义） */
    Z2_LAST_DOWN_TURN,   /* path end on 200 pile 10/12/6: face then one ground dismount */
    Z2_LAST_DOWN_DISMOUNT,
} z2_major_t;

static z2_major_t s_major;//状态机
static uint8_t s_path_idx;//路径索引
static uint16_t s_kfs_done_mask;//取件完成掩码

static uint8_t s_sent_mount;//发送上桩
static uint8_t s_sent_dismount;//发送下桩
static uint8_t s_sent_turn;//发送转弯
static uint8_t s_sent_getkfs;//发送取件

static uint8_t s_face_dir_step_done;/* 「request_face_field_dir」子步是否跑完；PATH_NEXT_PILE 换桩前摆头用，0=未做完 */
static uint8_t s_path_next_recenter_done;/* PATH_NEXT：摆头后回 from 桩心 */
static uint8_t s_last_down_recenter_done;/* LAST_DOWN_TURN：摆头后回末桩桩心 */
static uint8_t s_nav_leg_running;/* 1=本段导航已 arm，未 ARRIVED/失败前不下一段 */
static uint32_t s_nav_leg_session;/* 本段 session；peek 须一致，等同单独调试一段 nav */
static uint32_t s_nav_leg_fail_rc;/* 最近一次导航段失败码；NONE 表示无失败 */

static uint8_t s_kfs_j;//取件索引
static uint8_t s_enter_up_mount_enabled;/* 1=进 Z2_ENTER_UP 要上桩再导航；0=在 ENTER_UP 里直接转导航（见 case） */
static uint8_t s_last_exit_pile;/* Z2_LAST_DOWN_*：path 末桩示意图号 */
static app_zone2_field_dir_t s_last_face_dir_cmd;/* 最近一次 request_face_field_dir，供 turn_dir 与实发一致 */
static z2_step_kind_t s_step_kind;/* 当前脚本步骤，供调度/Watch 对齐 */
static uint32_t s_step_seq;/* 脚本步骤切换序号 */
static uint8_t s_step_from_pile;
static uint8_t s_step_to_pile;
static uint8_t s_step_kfs_pile;
static uint8_t s_step_kfs_idx;
static int16_t s_step_tier_delta;
static app_zone2_field_dir_t s_step_face_dir;

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

//获取任务路径长度
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
 * 方案 A：二区每段导航 = disarm → set_target → 等到 ARRIVED 或 TIMEOUT 后进入下一步（TIMEOUT 与到点同等）。
 * run 仍只由 chassis service_tick 驱动；状态机每拍 poll，未结束不进入下一步。
 */

static uint8_t z2_exec_process_motion_idle(void)
{
    return (uint8_t)(Process_UpStairs_IsBusy() == 0U && Process_DownStairs_IsBusy() == 0U &&
                     Process_GetKFS_IsBusy() == 0U);
}

/** 导航段只释放 Vy/Vw；Vx 航向控制可继续并行摆头 */
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

/* 开始一段到桩心导航；返回 0=未 arm（图号无效、Process 仍忙等，下拍重试） */
static uint8_t z2_exec_nav_start_pile(uint8_t pile)
{
    float xm;
    float ym;

    if (!user_pile_center_map_m(pile, &xm, &ym))
        return 0U;
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

/* 1=本段仍在进行；0=本段结束（ARRIVED 或 TIMEOUT，调度进下一步）；ODOM/BAD_CONFIG 仍结束整局 */
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
    /* 兜底：已 disarm 且距离在容差内，视为本段到点（与 odom_nav_goto 保持 ARRIVED 一致） */
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


/* 目标示意图桩 user_pile 的层档 ? 当前车层高 s_robot_tier；>0 要上桩档数，<0 要下桩档数 */
static int16_t user_pile_tier_delta(uint8_t user_pile)
{
    uint8_t want = pile_height_mm_to_tier(user_pile_height_mm(user_pile));
    return (int16_t)want - (int16_t)s_robot_tier;
}


static void z2_exec_reset_act_flags(void)
{
    s_sent_mount = 0U;//发送上桩标志
    s_sent_dismount = 0U;//发送下桩标志
    s_sent_turn = 0U;//发送转弯标志
}

/** 1=门控未允许发令；0=摆头命令已发，可与导航并行 */
static uint8_t z2_exec_face_substep(app_zone2_field_dir_t fd, uint8_t *done)
{
    if (*done != 0U)
        return 0U;
    if (!z2_exec_motion_gate_ok())
        return 1U;

    s_last_face_dir_cmd = fd;
    AppYawHeadingCtrl_RunFieldDir(fd);
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

static z2_exec_result_t z2_exec_get_kfs(uint8_t station_pile, uint8_t kfs_j)
{
    app_zone2_get_kfs_rel_t rel = app_zone2_get_kfs_rel(station_pile, s_mission.kfs[kfs_j]);

    if (s_sent_getkfs == 0U)
    {
        if (z2_exec_motion_gate_ok())
        {
            Process_GetKFS(rel);
            s_sent_getkfs = 1U;
        }
        return Z2_EXEC_BUSY;
    }
    if (!z2_exec_motion_gate_ok())
        return Z2_EXEC_BUSY;

    Process_GetKFS(rel);
    if (Process_GetKFS_IsBusy() != 0U)
        return Z2_EXEC_BUSY;

    s_kfs_done_mask |= (uint16_t)(1U << kfs_j);
    s_sent_getkfs = 0U;
    return Z2_EXEC_DONE;
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
            AppYawHeadingCtrl_RunFieldDir(fd);
            s_sent_turn = 1U;
        }
        return Z2_EXEC_BUSY;
    }
    if (!z2_exec_motion_gate_ok())
        return Z2_EXEC_BUSY;

    if (AppYawHeadingCtrl_IsBusy() != 0U)
        return Z2_EXEC_BUSY;

    s_sent_turn = 0U;
    return Z2_EXEC_DONE;
}

/* ==================== 调度层：决定下一步主状态与执行意图 ==================== */

static int8_t z2_sched_pick_kfs_on_pile(uint8_t pile, uint8_t *out_j)
{
    uint8_t j;//取件索引
    for (j = 0U; j < mission_kfs_len(); j++)//遍历取件索引
    {
        if (((s_kfs_done_mask >> j) & 1U) != 0U)//如果取件索引已经完成，则跳过
            continue;
        if (s_mission.kfs[j] != pile)//如果取件索引不等于当前桩号，则跳过
            continue;
        *out_j = j;//设置取件索引
        return 0;//返回0表示成功
    }
    return -1;//返回-1表示失败
}

static int8_t z2_sched_pick_kfs_adjacent(uint8_t *out_j)
{
    uint8_t st = s_mission.path[s_path_idx];
    uint8_t j;//取件索引
    for (j = 0U; j < mission_kfs_len(); j++)
    {
        if (((s_kfs_done_mask >> j) & 1U) != 0U)//如果取件索引已经完成，则跳过
            continue;
        if (!piles_adjacent(st, s_mission.kfs[j]))//如果当前路径桩号和取件桩号不相邻，则跳过
            continue;
        *out_j = j;//设置取件索引
        return 0;//返回0表示成功
    }
    return -1;
}

/** 当前 path 桩邻格秘籍已取完：path_idx++，或进入换桩/末桩下地面/结束 */
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

static void z2_sched_zone1_kfs_turn(void)
{
    uint8_t j;

    if (z2_sched_pick_kfs_on_pile(s_mission.path[0], &j) != 0)
    {
        s_enter_up_mount_enabled = 1U;
        z2_exec_reset_act_flags();
        z2_step_set(Z2_STEP_ENTER_MOUNT, 0U, s_mission.path[0], 0U, 0U,
                    user_pile_tier_delta(s_mission.path[0]), APP_ZONE2_FIELD_FACE_SKIP);
        s_major = Z2_ENTER_UP;
        return;
    }

    s_kfs_j = j;
    z2_step_set(Z2_STEP_ZONE1_KFS_FACE, s_mission.path[0], s_mission.path[0], s_mission.kfs[j], j,
                0, APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_face_beat(APP_ZONE2_FIELD_FACE_SKIP) == Z2_EXEC_BUSY)
        return;

    z2_step_set(Z2_STEP_ZONE1_KFS_GET, s_mission.path[0], s_mission.path[0], s_mission.kfs[j], j,
                0, APP_ZONE2_FIELD_FACE_SKIP);
    s_major = Z2_ZONE1_KFS_RUN;
    s_sent_getkfs = 0U;
}

static void z2_sched_zone1_kfs_run(void)
{
    z2_step_set(Z2_STEP_ZONE1_KFS_GET, s_mission.path[0], s_mission.path[0], s_mission.kfs[s_kfs_j],
                s_kfs_j, 0, APP_ZONE2_FIELD_FACE_SKIP);
    if (z2_exec_get_kfs(s_mission.path[0], s_kfs_j) == Z2_EXEC_BUSY)
        return;
    s_major = Z2_ZONE1_KFS_TURN;
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
    app_zone2_field_dir_t fd;

    if (z2_sched_pick_kfs_adjacent(&j) != 0)
    {
        z2_sched_after_station_kfs_done();
        return;
    }

    s_kfs_j = j;
    fd = field_dir_between_user_piles(s_mission.path[s_path_idx], s_mission.kfs[j]);
    z2_step_set(Z2_STEP_FACE_KFS, s_mission.path[s_path_idx], s_mission.path[s_path_idx],
                s_mission.kfs[j], j, 0, fd);
    if (z2_exec_face_beat(fd) == Z2_EXEC_BUSY)
        return;

    z2_step_set(Z2_STEP_GET_KFS, s_mission.path[s_path_idx], s_mission.path[s_path_idx], s_mission.kfs[j],
                j, 0, fd);
    s_major = Z2_KFS_RUN;
    s_sent_getkfs = 0U;
}

static void z2_sched_kfs_run(void)
{
    z2_step_set(Z2_STEP_GET_KFS, s_mission.path[s_path_idx], s_mission.path[s_path_idx],
                s_mission.kfs[s_kfs_j], s_kfs_j, 0,
                field_dir_between_user_piles(s_mission.path[s_path_idx], s_mission.kfs[s_kfs_j]));
    if (z2_exec_get_kfs(s_mission.path[s_path_idx], s_kfs_j) == Z2_EXEC_BUSY)
        return;
    s_major = Z2_KFS_TURN;
}

static void z2_sched_path_next_pile(void)
{
    uint8_t from_u = s_mission.path[s_path_idx - 1U];
    uint8_t to_u = s_mission.path[s_path_idx];
    int16_t cha = user_pile_tier_delta(to_u);
    app_zone2_field_dir_t fd_step;

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
    if (AppYawHeadingCtrl_IsBusy() != 0U)
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

    if (AppYawHeadingCtrl_IsBusy() != 0U)
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
    z2_step_set(Z2_STEP_DONE, s_last_exit_pile, 0U, 0U, 0U, 0, APP_ZONE2_FIELD_FACE_SKIP);
    s_major = Z2_DONE;
}

void app_zone2_set_robot_tier(uint8_t tier012)//设置机器人层高
{
    s_robot_tier = tier012;
}

void app_zone2_mission_clear(void)//清除任务
{
    memset(&s_mission, 0, sizeof(s_mission));
    s_has_mission = 0U;//没有任务
    s_major = Z2_IDLE;//空闲
    s_path_idx = 0U;//路径索引
    s_kfs_done_mask = 0U;//取件完成掩码
    z2_exec_reset_act_flags();//重置上桩动作标志
    s_sent_getkfs = 0U;//发送取件标志
    s_face_dir_step_done = 0U;//摆头完成标志
    s_path_next_recenter_done = 0U;
    s_last_down_recenter_done = 0U;
    s_nav_leg_session = 0U;
    s_nav_leg_running = 0U;
    s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
    s_enter_up_mount_enabled = 0U;//进入上桩标志
    s_last_exit_pile = 0U;//最后一出桩桩号
    z2_step_reset();
#if APP_ZONE2_DBG_FAKE_MISSION
    s_dbg_fake_rearm = 1U;
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

void app_zone2_mission_apply(const app_zone2_mission_t *m)//应用任务
{
    uint8_t j0;//取件索引

#if APP_ZONE2_DBG_FAKE_MISSION
    s_dbg_fake_rearm = 0U;/* 显式 apply（含 R1）优先于 poll 内自动假数据 */
#endif
    memcpy(&s_mission, m, sizeof(s_mission));
    s_has_mission = 1U;//有任务
    s_path_idx = 0U;//路径索引
    s_kfs_done_mask = 0U;//取件完成掩码
    z2_exec_reset_act_flags();//重置上桩动作标志
    s_sent_getkfs = 0U;//发送取件标志
    s_face_dir_step_done = 0U;
    s_path_next_recenter_done = 0U;
    s_last_down_recenter_done = 0U;
    s_nav_leg_session = 0U;
    s_nav_leg_running = 0U;
    s_nav_leg_fail_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
    s_last_exit_pile = 0U;
    z2_step_reset();
    z2_exec_nav_abort();

    if (z2_sched_pick_kfs_on_pile(s_mission.path[0], &j0) == 0)
    {
        /* path[0] 上还有待取秘籍：先走一区台面（上台前会再把 s_enter_up_mount_enabled 置 1） */
        s_enter_up_mount_enabled = 0U;
        s_major = Z2_ZONE1_KFS_TURN;
    }
    else
    {
        /* path[0] 无待取秘籍：进 ENTER_UP 上桩再上梅林 */
        s_enter_up_mount_enabled = 1U;
        s_major = Z2_ENTER_UP;
    }
    app_zone2_debug_set_poll_major();
}

uint8_t app_zone2_is_busy(void)//判断是否繁忙
{
    return (uint8_t)(s_has_mission != 0U && s_major != Z2_IDLE && s_major != Z2_DONE);
}

uint8_t app_zone2_is_done(void)//判断是否完成
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

        case Z2_ZONE1_KFS_TURN:
            z2_sched_zone1_kfs_turn();
            break;

        case Z2_ZONE1_KFS_RUN:
            z2_sched_zone1_kfs_run();
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
#endif
    g_app_zone2_debug.nav_poll_rc = APP_ZONE2_DEBUG_NAV_POLL_RC_NONE;
    app_zone2_debug_set_poll_major();

    if (!s_has_mission)
        return;

    app_zone2_step_pre_delay_sync();
    if (app_zone2_step_pre_delay_ready() == 0U)
        return;

    z2_sched_poll();
    app_zone2_debug_set_poll_major();
}

void app_zone2_poll(void)
{
    app_zone2_poll_core();
}
