/**
 * @file app_zone2.c
 * @brief 二区梅花桩：上台面、走路径、邻格取秘籍、换桩对齐车头与层高（与 app_zone2.h 一致）。
 */
#include "app_zone2.h"
#include "map.h"
#include "Motion_Task.h"
#include "main.h"

#include <string.h>

/* 门控：全自动档 + 二区钩子时 app_flow_zone2 或无自动模式（无 CH5/CH7 全自动流程） */
static uint8_t app_zone2_motion_gate_ok(void)
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
/*----------------------------------------------------------------------*/

static void (*app_zone2_hook_nav_set_target)(float x_m, float y_m);
static app_zone2_nav_poll_result_t (*app_zone2_hook_nav_poll)(void);
static void (*app_zone2_hook_request_mount_pile)(void);
static void (*app_zone2_hook_request_dismount_pile)(void);
static void (*app_zone2_hook_request_face_field_dir)(app_zone2_field_dir_t dir);
/** 取秘籍：rel 为站立 path 桩与邻格秘籍桩的 tier 高低关系 */
static void (*app_zone2_hook_request_get_kfs)(app_zone2_get_kfs_rel_t rel);
/** 摆头后查询航向是否仍跟踪（与 AppYawHeadingCtrl_IsBusy 解耦，由 app_init 绑定） */
static uint8_t (*app_zone2_hook_face_yaw_is_busy)(void);
/** 上/下桩、取 KFS 流程忙；NULL 在等待完成阶段视为一直忙（须与 request_* 在 app_init 成对注册，防一拍误过） */
static uint8_t (*app_zone2_hook_mount_pile_is_busy)(void);
static uint8_t (*app_zone2_hook_dismount_pile_is_busy)(void);
static uint8_t (*app_zone2_hook_get_kfs_is_busy)(void);

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
static uint8_t s_recenter_nav_active;/* poll_recenter_to_pile 已下发 nav 目标 */
static uint32_t s_nav_leg_session;/* 本段 set_target 后的 session_id，peek 须匹配才认 ARRIVED */

static uint8_t s_kfs_j;//取件索引
static uint8_t s_enter_up_mount_enabled;/* 1=进 Z2_ENTER_UP 要上桩再导航；0=在 ENTER_UP 里直接转导航（见 case） */
static uint8_t s_last_exit_pile;/* Z2_LAST_DOWN_*：path 末桩示意图号 */
static app_zone2_field_dir_t s_last_face_dir_cmd;/* 最近一次 request_face_field_dir，供 turn_dir 与实发一致 */

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

volatile app_zone2_debug_t g_app_zone2_debug = {
    0U,
    APP_ZONE2_DEBUG_NAV_POLL_RC_NONE,
};

static int16_t user_pile_tier_delta(uint8_t user_pile);

static void app_zone2_debug_set_poll_major(void)
{
    g_app_zone2_debug.poll_major = (uint32_t)s_major;
}

static void app_zone2_debug_record_nav_poll(app_zone2_nav_poll_result_t rc)
{
    g_app_zone2_debug.nav_poll_rc = (uint32_t)rc;
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

//设置导航桩中心
static void nav_set_pile_center_m(uint8_t pile)
{
    float xm;
    float ym;

    if (!user_pile_center_map_m(pile, &xm, &ym))
        return;
    if (app_zone2_hook_nav_set_target != NULL)
    {
        app_zone2_hook_nav_set_target(xm, ym);
        s_nav_leg_session = odom_nav_target.session_id;
    }
}

/* 本段导航 peek：须 session 一致，避免陈旧 ARRIVED/DISARMED */
static app_zone2_nav_poll_result_t nav_poll_leg_peek(void)
{
    app_zone2_nav_poll_result_t nav_rc;

    if (app_zone2_hook_nav_poll == NULL)
        return ODOM_NAV_GOTO_ERR_DISARMED;

    nav_rc = app_zone2_hook_nav_poll();
    app_zone2_debug_record_nav_poll(nav_rc);
    if (odom_nav_target.session_id != s_nav_leg_session)
        return ODOM_NAV_GOTO_ERR_DISARMED;
    return nav_rc;
}

static uint8_t nav_poll_leg_arrived(void)
{
    return (uint8_t)(nav_poll_leg_peek() == ODOM_NAV_GOTO_ERR_OK_ARRIVED);
}

/* 到点或超时：与 Z2_ENTER_WAIT_NAV 一致，供 recenter 等不能无限等 ARRIVED 的步骤 */
static uint8_t nav_poll_leg_finished(void)
{
    app_zone2_nav_poll_result_t nav_rc = nav_poll_leg_peek();

    return (uint8_t)((nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT));
}


/* 目标示意图桩 user_pile 的层档 ? 当前车层高 s_robot_tier；>0 要上桩档数，<0 要下桩档数 */
static int16_t user_pile_tier_delta(uint8_t user_pile)
{
    uint8_t want = pile_height_mm_to_tier(user_pile_height_mm(user_pile));
    return (int16_t)want - (int16_t)s_robot_tier;
}


//重置上桩动作标志
static void reset_stair_act_flags(void)
{
    s_sent_mount = 0U;//发送上桩标志
    s_sent_dismount = 0U;//发送下桩标志
    s_sent_turn = 0U;//发送转弯标志
}

//判断车头朝向准备好
static uint8_t poll_face_dir_done(app_zone2_field_dir_t fd, uint8_t *done)
{
    if (*done != 0U) /* 这一轮「摆头」已经标记完成，不再发 */
        return 0U;//返回0表示成功

    if (s_sent_turn == 0U) /* 还没发出转弯命令 */
    {
        if (app_zone2_motion_gate_ok()) /* 半自动空闲，可以发下一条 */
        {
            if (app_zone2_hook_request_face_field_dir != NULL)
            {
                s_last_face_dir_cmd = fd;
                app_zone2_hook_request_face_field_dir(fd);
                s_sent_turn = 1U; /* 已发，等 face_yaw_is_busy==0（周期 Run 在 manual_chassis_function） */
            }
        }
    }
    else
    {
        /* 已发摆头：RunFieldDir 只设场向；此处只查忙闲，周期 PD 在 chassis manual_chassis_function */
        if (app_zone2_motion_gate_ok())
        {
            if (app_zone2_hook_face_yaw_is_busy != NULL && app_zone2_hook_face_yaw_is_busy() == 0U)
            {
                s_sent_turn = 0U;
                *done = 1U;
                return 1U; /* 本拍刚完成「等转弯结束」，返回 1 与末尾一致：本拍占住、下拍再进后续逻辑 */
            }
        }
    }
    return 1U; /* 还在等门控 / 航向未对准 */
}

/* 摆头后回指定桩心；到点以 odom_nav_goto_run 内 arrival_confirm_cycles 为准 */
static uint8_t poll_recenter_to_pile(uint8_t pile, uint8_t *done)
{
    if (*done != 0U)
        return 0U;

    if (s_recenter_nav_active == 0U)
    {
        if (app_zone2_motion_gate_ok())
        {
            nav_set_pile_center_m(pile);
            s_recenter_nav_active = 1U;
        }
        return 1U;
    }

    if (nav_poll_leg_finished() != 0U)
    {
        s_recenter_nav_active = 0U;
        *done = 1U;
        return 0U;
    }
    return 1U;
}


//判断上桩或下桩准备好
static uint8_t poll_one_stair_step(int16_t cha)
{
    uint8_t up = (uint8_t)(cha > 0);
    uint8_t *sent = up ? &s_sent_mount : &s_sent_dismount;

    if (*sent == 0U)
    {
        if (app_zone2_motion_gate_ok())
        {
            odom_nav_goto_disarm();
            if (up) {
                if (app_zone2_hook_request_mount_pile != NULL)
                {
                    app_zone2_hook_request_mount_pile();
                    *sent = 1U;
                }
            } else {
                if (app_zone2_hook_request_dismount_pile != NULL)
                {
                    app_zone2_hook_request_dismount_pile();
                    *sent = 1U;
                }
            }
        }
    }
    else if (app_zone2_motion_gate_ok())
    {
        if (up)
        {
            if (app_zone2_hook_request_mount_pile != NULL)
                app_zone2_hook_request_mount_pile();
            if (app_zone2_hook_mount_pile_is_busy == NULL ||
                app_zone2_hook_mount_pile_is_busy() != 0U)
                return 1U;
            s_robot_tier++; /* 层高加1 */
        }
        else
        {
            if (app_zone2_hook_request_dismount_pile != NULL)
                app_zone2_hook_request_dismount_pile();
            if (app_zone2_hook_dismount_pile_is_busy == NULL ||
                app_zone2_hook_dismount_pile_is_busy() != 0U)
                return 1U;
            s_robot_tier--; /* 层高减1 */
        }
        odom_nav_goto_disarm();
        *sent = 0U;
        return 1U;
    }
    return 1U;
}


static uint8_t poll_last_ground_dismount_busy(void)
{
    if (s_sent_dismount == 0U)
    {
        if (app_zone2_motion_gate_ok())
        {
            odom_nav_goto_disarm();
            if (app_zone2_hook_request_dismount_pile != NULL)
            {
                app_zone2_hook_request_dismount_pile();
                s_sent_dismount = 1U;
            }
        }
    }
    else if (app_zone2_motion_gate_ok())
    {
        if (app_zone2_hook_request_dismount_pile != NULL)
            app_zone2_hook_request_dismount_pile();
        if (app_zone2_hook_dismount_pile_is_busy == NULL ||
            app_zone2_hook_dismount_pile_is_busy() != 0U)
            return 1U;
        s_sent_dismount = 0U;
        return 0U;
    }
    return 1U;
}

//一区清 path[0] 台面取件桩，未有取kfs动作
static int8_t pick_next_kfs_on_pile(uint8_t pile, uint8_t *out_j)
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

//已上桩、在 当前所在桩的邻格上看有无取件桩，未有取kfs动作
static int8_t pick_next_kfs_for_station(uint8_t *out_j)
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
    return -1;//返回-1表示失败
}

void app_zone2_init_hooks(
    void (*nav_set_target)(float x_m, float y_m),
    app_zone2_nav_poll_result_t (*nav_poll)(void),
    void (*request_mount_pile)(void),
    void (*request_dismount_pile)(void),
    void (*request_face_field_dir)(app_zone2_field_dir_t dir),
    void (*request_get_kfs)(app_zone2_get_kfs_rel_t rel),
    uint8_t (*face_yaw_is_busy)(void),
    uint8_t (*mount_pile_is_busy)(void),
    uint8_t (*dismount_pile_is_busy)(void),
    uint8_t (*get_kfs_is_busy)(void))
{
    app_zone2_hook_nav_set_target = nav_set_target;
    app_zone2_hook_nav_poll = nav_poll;
    app_zone2_hook_request_mount_pile = request_mount_pile;
    app_zone2_hook_request_dismount_pile = request_dismount_pile;
    app_zone2_hook_request_face_field_dir = request_face_field_dir;
    app_zone2_hook_request_get_kfs = request_get_kfs;
    app_zone2_hook_face_yaw_is_busy = face_yaw_is_busy;
    app_zone2_hook_mount_pile_is_busy = mount_pile_is_busy;
    app_zone2_hook_dismount_pile_is_busy = dismount_pile_is_busy;
    app_zone2_hook_get_kfs_is_busy = get_kfs_is_busy;
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
    reset_stair_act_flags();//重置上桩动作标志
    s_sent_getkfs = 0U;//发送取件标志
    s_face_dir_step_done = 0U;//摆头完成标志
    s_path_next_recenter_done = 0U;
    s_last_down_recenter_done = 0U;
    s_recenter_nav_active = 0U;
    s_nav_leg_session = 0U;
    s_enter_up_mount_enabled = 0U;//进入上桩标志
    s_last_exit_pile = 0U;//最后一出桩桩号
#if APP_ZONE2_DBG_FAKE_MISSION
    s_dbg_fake_rearm = 1U;
#endif
    app_zone2_step_pre_delay_reset();
    odom_nav_goto_disarm();
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
    reset_stair_act_flags();//重置上桩动作标志
    s_sent_getkfs = 0U;//发送取件标志
    s_face_dir_step_done = 0U;
    s_path_next_recenter_done = 0U;
    s_last_down_recenter_done = 0U;
    s_recenter_nav_active = 0U;
    s_nav_leg_session = 0U;
    s_last_exit_pile = 0U;
    odom_nav_goto_disarm();

    if (pick_next_kfs_on_pile(s_mission.path[0], &j0) == 0)//一区清 path[0] 台面取件桩，未有取kfs动作
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

    switch (s_major)
    {
        case Z2_IDLE:
        case Z2_DONE:
            return;

        case Z2_ZONE1_KFS_TURN:
        {
            uint8_t j;

            if (pick_next_kfs_on_pile(s_mission.path[0], &j) != 0)
            {
                s_enter_up_mount_enabled = 1U;
                reset_stair_act_flags();
                s_major = Z2_ENTER_UP;
                break;
            }

            s_kfs_j = j;
            if (s_sent_turn == 0U)
            {
                if (app_zone2_motion_gate_ok())
                {
                    /* SKIP：车头由一区取件钩子自己管，这里只凑「发转向 → 再等空闲」节拍 */
                    if (app_zone2_hook_request_face_field_dir != NULL)
                    {
                        app_zone2_hook_request_face_field_dir(APP_ZONE2_FIELD_FACE_SKIP);
                        s_sent_turn = 1U;
                    }
                }
            }
            else if (app_zone2_motion_gate_ok())
            {
                s_sent_turn = 0U;
                s_major = Z2_ZONE1_KFS_RUN;
                s_sent_getkfs = 0U;
            }
            break;
        }

        case Z2_ZONE1_KFS_RUN:
            if (s_sent_getkfs == 0U)
            {
                if (app_zone2_motion_gate_ok())
                {
                    if (app_zone2_hook_request_get_kfs != NULL)
                    {
                        app_zone2_hook_request_get_kfs(
                            app_zone2_get_kfs_rel(s_mission.path[0], s_mission.kfs[s_kfs_j]));
                        s_sent_getkfs = 1U;
                    }
                }
            }
            else if (app_zone2_motion_gate_ok())
            {
                if (app_zone2_hook_request_get_kfs != NULL)
                {
                    app_zone2_hook_request_get_kfs(
                        app_zone2_get_kfs_rel(s_mission.path[0], s_mission.kfs[s_kfs_j]));
                }
                if (app_zone2_hook_get_kfs_is_busy == NULL ||
                    app_zone2_hook_get_kfs_is_busy() != 0U)
                    break;
                s_kfs_done_mask |= (uint16_t)(1U << s_kfs_j);
                s_sent_getkfs = 0U;
                s_major = Z2_ZONE1_KFS_TURN;
            }
            break;

        case Z2_ENTER_UP:
        {
            if (s_enter_up_mount_enabled == 0U)
            {
                s_major = Z2_ENTER_NAV;
                break;
            }

            if (s_sent_mount == 0U)
            {
                if (app_zone2_motion_gate_ok())
                {
                    odom_nav_goto_disarm();
                    if (app_zone2_hook_request_mount_pile != NULL)
                    {
                        app_zone2_hook_request_mount_pile();
                        s_sent_mount = 1U;
                    }
                }
            }
            else if (app_zone2_motion_gate_ok())
            {
                if (app_zone2_hook_request_mount_pile != NULL)
                    app_zone2_hook_request_mount_pile();
                if (app_zone2_hook_mount_pile_is_busy == NULL ||
                    app_zone2_hook_mount_pile_is_busy() != 0U)
                    break;
                s_sent_mount = 0U;
                s_enter_up_mount_enabled = 0U;
                s_face_dir_step_done = 0U;
                s_robot_tier = pile_height_mm_to_tier(user_pile_height_mm(s_mission.path[s_path_idx]));
                s_major = Z2_ENTER_NAV;
            }
            break;
        }

        case Z2_ENTER_NAV:
            nav_set_pile_center_m(s_mission.path[s_path_idx]);
            s_major = Z2_ENTER_WAIT_NAV;
            break;

        case Z2_ENTER_WAIT_NAV:
            if (nav_poll_leg_finished() != 0U)
                s_major = Z2_KFS_TURN;
            break;

        case Z2_KFS_TURN:
        {
            uint8_t j;
            if (pick_next_kfs_for_station(&j) != 0)
            {
                uint8_t const plen = mission_path_len();
                uint8_t const cur_pile = s_mission.path[s_path_idx];
                uint8_t const path_last_pile = (plen > 0U) ? s_mission.path[plen - 1U] : 0U;

                s_path_idx++;
                /* 未到 path 末尾：只是换下一 path 桩，与末格 10/12/6 无关 */
                if (s_path_idx < plen)
                {
                    s_major = Z2_PATH_NEXT_PILE;
                    reset_stair_act_flags();
                    s_face_dir_step_done = 0U;
                    s_path_next_recenter_done = 0U;
                    s_recenter_nav_active = 0U;
                    break;
                }

                /* 此处 path 已走完，cur_pile 必为 path 最后一格（plen-1），不会在中间桩误触发 */
                if (plen > 0U && cur_pile == path_last_pile &&
                    user_pile_height_mm(cur_pile) == 200U &&
                    (cur_pile == 10U || cur_pile == 12U || cur_pile == 6U))
                {
                    s_last_exit_pile = cur_pile;
                    reset_stair_act_flags();
                    s_face_dir_step_done = 0U;
                    s_last_down_recenter_done = 0U;
                    s_recenter_nav_active = 0U;
                    s_major = Z2_LAST_DOWN_TURN;
                }
                else
                {
                    s_major = Z2_DONE;
                }
                break;
            }

            s_kfs_j = j;
            if (s_sent_turn == 0U)
            {
                if (app_zone2_motion_gate_ok())
                {
                    if (app_zone2_hook_request_face_field_dir != NULL)
                    {
                        app_zone2_hook_request_face_field_dir(
                            field_dir_between_user_piles(s_mission.path[s_path_idx], s_mission.kfs[j]));
                        s_sent_turn = 1U;
                    }
                }
            }
            else if (app_zone2_motion_gate_ok())
            {
                s_sent_turn = 0U;
                s_major = Z2_KFS_RUN;
                s_sent_getkfs = 0U;
            }
            break;
        }

        case Z2_KFS_RUN:
            if (s_sent_getkfs == 0U)
            {
                if (app_zone2_motion_gate_ok())
                {
                    if (app_zone2_hook_request_get_kfs != NULL)
                    {
                        app_zone2_hook_request_get_kfs(
                            app_zone2_get_kfs_rel(s_mission.path[s_path_idx], s_mission.kfs[s_kfs_j]));
                        s_sent_getkfs = 1U;
                    }
                }
            }
            else if (app_zone2_motion_gate_ok())
            {
                if (app_zone2_hook_request_get_kfs != NULL)
                {
                    app_zone2_hook_request_get_kfs(
                        app_zone2_get_kfs_rel(s_mission.path[s_path_idx], s_mission.kfs[s_kfs_j]));
                }
                if (app_zone2_hook_get_kfs_is_busy == NULL ||
                    app_zone2_hook_get_kfs_is_busy() != 0U)
                    break;
                s_kfs_done_mask |= (uint16_t)(1U << s_kfs_j);
                s_sent_getkfs = 0U;
                s_major = Z2_KFS_TURN;
            }
            break;

        case Z2_PATH_NEXT_PILE:
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
                /* 低→高：车头朝邻格；高→低：取反，车尾朝目标桩（下桩） */
                if (cha < 0)
                    fd_step = field_dir_opposite(fd_step);
                if (poll_face_dir_done(fd_step, &s_face_dir_step_done))//转向执行
                    break;
            }

            if (s_face_dir_step_done != 0U && s_path_next_recenter_done == 0U)
            {
                if (poll_recenter_to_pile(from_u, &s_path_next_recenter_done) != 0U)
                    break;
            }

            /* 摆头 → 回 from 桩心 → 按 cha 上/下桩；层高对齐后 → 去 to 桩心 */
            if (s_face_dir_step_done != 0U && s_path_next_recenter_done != 0U && cha == 0)
            {
                s_face_dir_step_done = 0U;
                s_path_next_recenter_done = 0U;
                s_major = Z2_ENTER_NAV;
                break;
            }

            if (s_face_dir_step_done == 0U || s_path_next_recenter_done == 0U)
                break;

            poll_one_stair_step(cha);//上/下桩执行
            if (user_pile_tier_delta(to_u) == 0)
            {
                s_face_dir_step_done = 0U;
                s_path_next_recenter_done = 0U;
                s_recenter_nav_active = 0U;
                nav_set_pile_center_m(to_u);
                s_major = Z2_ENTER_WAIT_NAV;
            }
            break;
        }

        case Z2_LAST_DOWN_TURN:
        {
            app_zone2_field_dir_t fd = APP_ZONE2_FIELD_BACK;

            if (s_last_exit_pile == 6U)
            {
                /* 与换桩 5→6、cha<0 时摆头一致（不读当前 tier，避免到站后 cha==0 判错） */
                fd = field_dir_opposite(field_dir_between_user_piles(5U, 6U));
            }

            if (s_face_dir_step_done == 0U)
            {
                if (poll_face_dir_done(fd, &s_face_dir_step_done) != 0U)
                    break;
                break;
            }

            if (s_last_down_recenter_done == 0U)
            {
                if (poll_recenter_to_pile(s_last_exit_pile, &s_last_down_recenter_done) != 0U)
                    break;
            }

            if (app_zone2_hook_face_yaw_is_busy != NULL &&
                app_zone2_hook_face_yaw_is_busy() != 0U)
            {
                break;
            }
            s_face_dir_step_done = 0U;
            s_last_down_recenter_done = 0U;
            reset_stair_act_flags();
            s_major = Z2_LAST_DOWN_DISMOUNT;
            break;
        }

        case Z2_LAST_DOWN_DISMOUNT:
            if (poll_last_ground_dismount_busy() != 0U)
                break;
            s_major = Z2_DONE;
            break;

        default:
            break;
    }
    app_zone2_debug_set_poll_major();
}

void app_zone2_poll(void)
{
    app_zone2_poll_core();
}
