/**
 * @file app_zone2.c
 * @brief 二区梅花桩：上台面、走路径、邻格取秘籍、换桩对齐车头与层高（与 app_zone2.h 一致）。
 */
#include "app_zone2.h"
#include "Motion_Task.h"

#include <string.h>

/* 下标 1..12 = MF_Block_1..12 的矩形中心（map.c 里 MF_BLOCK_* 的 mm/1000）；改 map 矩形时请同步改这里 */
static const float s_mf_cx_m[13] = {
    0.f, 
    1.8f, 3.0f, 4.2f,
    1.8f, 3.0f, 4.2f, 
    1.8f, 3.0f, 4.2f,
};
static const float s_mf_cy_m[13] = {
    0.f,
    3.8f, 3.8f, 3.8f, 5.0f, 
    5.0f, 5.0f, 6.2f, 6.2f, 
    6.2f, 7.4f, 7.4f,
};

static uint8_t user_pile_to_mf(uint8_t user_pile)//桩号转换为梅花桩编号
{
    static const uint8_t dinghang[4] = {0, 3U, 2U, 1U};
    if (user_pile >= 1U && user_pile <= 3U)
        return dinghang[user_pile];
    return user_pile;
}

/*
 * 桩顶 mm：下标 = 物理 MF 格 1..12（与 map 梅林块自上而下、每行左→右）。
 * 附图第二行左→右为 200、400、600 → 即 mf4=200、mf5=400、mf6=600；600 在下标 6，不是下标 4。
 * 若口头「四号」指图上另一套桩号，应对照物理格改 path/kfs，勿只改 [4]。
 */
static const uint16_t s_pile_height_mm[13] = {
    0,
    400, 200, 400,
    200, 400, 600,
    400, 600, 400,
    200, 400, 200,
};
/* 桩顶 mm：下标 = 物理 MF 格 1..12（与 map 梅林块自上而下、每行左→右）。 */
static uint16_t pile_height_mm_for_schematic(uint8_t schematic_pile)
{
    return s_pile_height_mm[user_pile_to_mf(schematic_pile)];
}

//高度转换为层高
static uint8_t tier_from_mm(uint16_t mm)
{
    if (mm == 200U)
        return 0U;
    if (mm == 400U)
        return 1U;
    if (mm == 600U)
        return 2U;
    return 0U;
}

static uint8_t piles_adjacent(uint8_t pile_a, uint8_t pile_b)//判断两个桩是否相邻
{
    uint8_t mf_a = user_pile_to_mf(pile_a);
    uint8_t mf_b = user_pile_to_mf(pile_b);
    uint8_t ra = (uint8_t)((mf_a - 1U) / 3U);
    uint8_t ca = (uint8_t)((mf_a - 1U) % 3U);
    uint8_t rb = (uint8_t)((mf_b - 1U) / 3U);
    uint8_t cb = (uint8_t)((mf_b - 1U) % 3U);
    uint8_t dr = (uint8_t)((ra > rb) ? (ra - rb) : (rb - ra));
    uint8_t dc = (uint8_t)((ca > cb) ? (ca - cb) : (cb - ca));
    return (uint8_t)((dr + dc) == 1U);
}

static app_zone2_field_dir_t field_dir_mf_step(uint8_t mf_a, uint8_t mf_b)
{
    uint8_t ra = (uint8_t)((mf_a - 1U) / 3U);
    uint8_t ca = (uint8_t)((mf_a - 1U) % 3U);
    uint8_t rb = (uint8_t)((mf_b - 1U) / 3U);
    uint8_t cb = (uint8_t)((mf_b - 1U) % 3U);

    if (rb > ra)
        return APP_ZONE2_FIELD_FRONT;
    if (rb < ra)
        return APP_ZONE2_FIELD_BACK;
    if (cb > ca)
        return APP_ZONE2_FIELD_RIGHT;
    if (cb < ca)
        return APP_ZONE2_FIELD_LEFT;
    return APP_ZONE2_FIELD_FACE_SKIP;
}

static app_zone2_field_dir_t field_dir_neighbor_step(uint8_t pile_a, uint8_t pile_b)
{
    return field_dir_mf_step(user_pile_to_mf(pile_a), user_pile_to_mf(pile_b));
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
//根据桩号选择最合适的梅花桩
static uint8_t ref_mf_for_zone1_approach(uint8_t target_user_pile)
{
    int i;
    uint8_t mf = user_pile_to_mf(target_user_pile);
    uint8_t r = (uint8_t)((mf - 1U) / 3U);
    uint8_t c = (uint8_t)((mf - 1U) % 3U);
    uint8_t best_mf = 0U;
    float best_cy = 999.f;
    static const int8_t dr[4] = {-1, 1, 0, 0};
    static const int8_t dc[4] = {0, 0, -1, 1};

    for (i = 0; i < 4; i++)
    {
        int32_t nr = (int32_t)r + (int32_t)dr[i];
        int32_t nc = (int32_t)c + (int32_t)dc[i];
        if (nr < 0 || nr > 3 || nc < 0 || nc > 2)
            continue;
        uint8_t nm = (uint8_t)(nr * 3 + nc + 1);
        float cy = s_mf_cy_m[nm];
        if (cy < best_cy)
        {
            best_cy = cy;
            best_mf = nm;
        }
    }
    return best_mf;
}
//根据桩号和层高计算离开梅花桩时的朝向
static app_zone2_field_dir_t leave_stair_face_dir(uint8_t from_u, uint8_t to_u, int16_t cha)
{
    app_zone2_field_dir_t tw = APP_ZONE2_FIELD_BACK;
    if (piles_adjacent(from_u, to_u))
        tw = field_dir_neighbor_step(from_u, to_u);
    if (cha < 0)
        return field_dir_opposite(tw);
    return tw;
}
/*----------------------------------------------------------------------*/

static app_zone2_hooks_t s_hooks;//钩子函数
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
    Z2_LEAVE_STAIR,//离开桩
} z2_major_t;

static z2_major_t s_major;//状态机
static uint8_t s_path_idx;//路径索引
static uint16_t s_kfs_done_mask;//取件完成掩码

static uint8_t s_sent_mount;//发送上桩
static uint8_t s_sent_dismount;//发送下桩
static uint8_t s_sent_turn;//发送转弯
static uint8_t s_sent_getkfs;//发送取件

static uint8_t s_pre_stair_heading_ok;//上桩前朝向准备好
static uint8_t s_kfs_j;//取件索引
static uint8_t s_first_mount_stairs;/* 1=进 Z2_ENTER_UP 要走摆头+上桩；0=在 ENTER_UP 里直接转导航（见 case） */

//获取任务路径长度
static uint8_t mission_path_len(void)
{
    uint8_t i;
    for (i = 0U; i < APP_ZONE2_MAX_PATH; i++)
    {
        if (s_mission.path[i] == 0U)
            break;
    }
    return i;
}

//获取当前路径桩号
static uint8_t cur_path_pile(void)
{
    return s_mission.path[s_path_idx];
}

//设置导航桩中心
static void nav_set_pile_center_m(uint8_t pile)
{
    float xm;//x坐标
    uint8_t mf = user_pile_to_mf(pile);//桩号转换为梅花桩编号

    xm = s_mf_cx_m[mf];//梅花桩x坐标
#if !APP_ZONE2_RED_SIDE
    /* 蓝侧：半幅内沿 x 镜像到本机 map（若你们 odom 已统一成全图坐标，可改宏为 1 并在此关掉镜像） */
    xm = APP_ZONE2_MIRROR_X_M - xm;//镜像到本机map
#endif
    s_hooks.nav_set_target(xm, s_mf_cy_m[mf]);
}


//层高差转换为桩号差
static int16_t tier_delta_to_pile(uint8_t user_pile)
{
    uint8_t want = tier_from_mm(pile_height_mm_for_schematic(user_pile));
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
        if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none)) /* 半自动空闲，可以发下一条 */
        {
            s_hooks.request_face_field_dir(fd);
            s_sent_turn = 1U; /* 已发，等动作跑完 */
        }
    }
    else if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none)) /* 转弯已发过，且半自动又回到空闲 → 本步结束 */
    {
        s_sent_turn = 0U;
        *done = 1U;
        return 1U; /* 本拍刚完成「等转弯结束」，返回 1 与末尾一致：本拍占住、下拍再进后续逻辑 */
    }
    return 1U; /* 还在等半自动空闲或转弯未结束 */
}


//判断上桩或下桩准备好
static uint8_t poll_one_stair_step(int16_t cha)
{
    uint8_t up = (uint8_t)(cha > 0);
    uint8_t *sent = up ? &s_sent_mount : &s_sent_dismount;

    if (*sent == 0U)
    {
        if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
        {
            if (up)//上桩
                s_hooks.request_mount_pile(s_hooks.user);//发送上桩命令
            else//下桩
                s_hooks.request_dismount_pile(s_hooks.user);//发送下桩命令
            *sent = 1U;
        }
    }
    else if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
    {
        if (up)
            s_robot_tier++;//层高加1
        else
            s_robot_tier--;//层高减1
        *sent = 0U;
        return 1U;
    }
    return 1U;
}

//一区清 path[0] 台面取件桩，未有取kfs动作
static int8_t pick_next_kfs_on_pile(uint8_t pile, uint8_t *out_j)
{
    uint8_t j;//取件索引
    for (j = 0U; j < APP_ZONE2_MAX_KFS && s_mission.kfs[j] != 0U; j++)//遍历取件索引
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
    uint8_t st = cur_path_pile();//获取当前路径桩号
    uint8_t j;//取件索引
    for (j = 0U; j < APP_ZONE2_MAX_KFS && s_mission.kfs[j] != 0U; j++)
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

void app_zone2_set_hooks(const app_zone2_hooks_t *hooks)//设置钩子函数
{
    if (hooks != NULL)
        s_hooks = *hooks;
}
//当前未用上
void app_zone2_set_robot_tier(uint8_t tier012)//设置机器人层高
{
    s_robot_tier = tier012;
}

void app_zone2_mission_clear(void)//清除任务
{
    s_has_mission = 0U;//没有任务
    s_major = Z2_IDLE;//空闲
    s_path_idx = 0U;//路径索引
    s_kfs_done_mask = 0U;//取件完成掩码
    reset_stair_act_flags();//重置上桩动作标志
    s_sent_getkfs = 0U;//发送取件标志
    s_pre_stair_heading_ok = 0U;//上桩前朝向准备好
    s_first_mount_stairs = 0U;
}

void app_zone2_mission_apply(const app_zone2_mission_t *m)//应用任务
{
    uint8_t j0;//取件索引

    memcpy(&s_mission, m, sizeof(s_mission));
    s_has_mission = 1U;//有任务
    s_path_idx = 0U;//路径索引
    s_kfs_done_mask = 0U;//取件完成掩码
    reset_stair_act_flags();//重置上桩动作标志
    s_sent_getkfs = 0U;//发送取件标志
    s_pre_stair_heading_ok = 0U;//上桩前朝向准备好

    if (pick_next_kfs_on_pile(s_mission.path[0], &j0) == 0)//一区清 path[0] 台面取件桩，未有取kfs动作
    {
        /* path[0] 上还有待取秘籍：先走一区台面转向/取件，不先进「上台」大流程 */
        s_first_mount_stairs = 0U;
        s_major = Z2_ZONE1_KFS_TURN;
    }
    else
    {
        /* path[0] 无待取秘籍：从上台进梅林（ENTER_UP 里摆头+上桩） */
        s_first_mount_stairs = 1U;
        s_major = Z2_ENTER_UP;
    }
}

uint8_t app_zone2_is_busy(void)//判断是否繁忙
{
    return (uint8_t)(s_has_mission != 0U && s_major != Z2_IDLE && s_major != Z2_DONE);
}

uint8_t app_zone2_is_done(void)//判断是否完成
{
    return (uint8_t)(s_has_mission != 0U && s_major == Z2_DONE);
}

void app_zone2_poll(void)
{
    if (!s_has_mission)
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
                s_first_mount_stairs = 1U;
                s_pre_stair_heading_ok = 0U;
                reset_stair_act_flags();
                s_major = Z2_ENTER_UP;
                break;
            }

            s_kfs_j = j;
            if (s_sent_turn == 0U)
            {
                if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
                {
                    /* SKIP：车头由一区取件钩子自己管，这里只凑「发转向 → 再等空闲」节拍 */
                    s_hooks.request_face_field_dir(APP_ZONE2_FIELD_FACE_SKIP);
                    s_sent_turn = 1U;
                }
            }
            else if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
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
                if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
                {
                    s_hooks.request_get_kfs(s_mission.path[0], s_mission.kfs[s_kfs_j], s_kfs_j, s_hooks.user);
                    s_sent_getkfs = 1U;
                }
            }
            else if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
            {
                s_kfs_done_mask |= (uint16_t)(1U << s_kfs_j);
                s_sent_getkfs = 0U;
                s_major = Z2_ZONE1_KFS_TURN;
            }
            break;

        case Z2_ENTER_UP:
        {
            uint8_t ref_mf;
            app_zone2_field_dir_t fd;

            if (s_first_mount_stairs == 0U)
            {
                s_major = Z2_ENTER_NAV;
                break;
            }

            if (s_pre_stair_heading_ok == 0U)
            {
                fd = APP_ZONE2_FIELD_FRONT;
                ref_mf = ref_mf_for_zone1_approach(cur_path_pile());
                if (ref_mf != 0U)
                    fd = field_dir_mf_step(ref_mf, user_pile_to_mf(cur_path_pile()));
                if (poll_face_dir_done(fd, &s_pre_stair_heading_ok))
                    break;
            }

            if (s_sent_mount == 0U)
            {
                if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
                {
                    s_hooks.request_mount_pile(s_hooks.user);
                    s_sent_mount = 1U;
                }
            }
            else if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
            {
                s_sent_mount = 0U;
                s_first_mount_stairs = 0U;
                s_pre_stair_heading_ok = 0U;
                s_robot_tier = tier_from_mm(pile_height_mm_for_schematic(cur_path_pile()));
                s_major = Z2_ENTER_NAV;
            }
            break;
        }

        case Z2_ENTER_NAV:
            nav_set_pile_center_m(cur_path_pile());
            s_major = Z2_ENTER_WAIT_NAV;
            break;

        case Z2_ENTER_WAIT_NAV:
            if (s_hooks.nav_poll() == APP_ZONE2_NAV_ARRIVED)
                s_major = Z2_KFS_TURN;
            break;

        case Z2_KFS_TURN:
        {
            uint8_t j;
            if (pick_next_kfs_for_station(&j) != 0)
            {
                s_path_idx++;
                if (s_path_idx >= mission_path_len())
                {
                    s_major = Z2_DONE;
                    break;
                }
                s_major = Z2_LEAVE_STAIR;
                reset_stair_act_flags();
                s_pre_stair_heading_ok = 0U;
                break;
            }

            s_kfs_j = j;
            if (s_sent_turn == 0U)
            {
                if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
                {
                    s_hooks.request_face_field_dir(
                        field_dir_neighbor_step(cur_path_pile(), s_mission.kfs[j]));
                    s_sent_turn = 1U;
                }
            }
            else if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
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
                if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
                {
                    s_hooks.request_get_kfs(cur_path_pile(), s_mission.kfs[s_kfs_j], s_kfs_j, s_hooks.user);
                    s_sent_getkfs = 1U;
                }
            }
            else if ((control_mode == semi_auto_control) && (semi_auto_mode == semi_auto_none))
            {
                s_kfs_done_mask |= (uint16_t)(1U << s_kfs_j);
                s_sent_getkfs = 0U;
                s_major = Z2_KFS_TURN;
            }
            break;

        case Z2_LEAVE_STAIR:
        {
            uint8_t from_u = s_mission.path[s_path_idx - 1U];
            uint8_t to_u = cur_path_pile();
            int16_t cha = tier_delta_to_pile(to_u);

            if (s_pre_stair_heading_ok == 0U)
            {
                if (poll_face_dir_done(leave_stair_face_dir(from_u, to_u, cha), &s_pre_stair_heading_ok))
                    break;
            }

            if (cha == 0)
            {
                s_pre_stair_heading_ok = 0U;
                s_major = Z2_ENTER_NAV;
                break;
            }

            poll_one_stair_step(cha);
            break;
        }

        default:
            break;
    }
}
