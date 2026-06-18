/**
 * @file r1_zone3_parse.c
 * @brief 三区指令解析（统一入口）
 *
 * === 业务调用链 ===
 * 3个IR源->3条路径，汇聚到 AppZone3_PostR1Cmd：
 *
 * (1) USART1 EE..FF(1~7): r1_zone3_parse_from_link_z3_cmd(cmd_id, put_sub)
 *    -> r1_zone3_link_z3_cmd_wire_to_z3(cmd_id, &id)      // wire->zone3
 *    -> r1_zone3_parse_post(id, raw_cmd, put_sub)
 *    -> AppZone3_PostR1Cmd(&z3)
 *
 * (2) USART10 55..AA Put: r1_zone3_parse_from_link_z3_put(cmd_id,raw)
 *    -> 仅PUT_L3 -> r1_zone3_parse_post(APP_Z3_CMD_PUT_KFS_ON_R1)
 *    -> AppZone3_PostR1Cmd(&z3)
 *
 * (3) USART10 EE..FF STOP: r1_zone3_parse_from_usart10_stop()
 *    -> r1_zone3_parse_post(APP_Z3_CMD_STOP_ACTION)
 *    -> AppZone3_PostR1Cmd(&z3)
 *
 * (4) USART10 EE..FF GET_KFS/放料: r1_zone3_parse_from_link_z3_cmd(cmd_id, put_sub)
 *    -> 同路径(1)
 */
/*---------------------------------------------------------------------
 * 【3个来源入口】
 * 1. USART1/r1_link_z3_cmd：r1_zone3_parse_from_link_z3_cmd()  EE..FF wire 1~7
 * 2. USART10/r1_link 放三层：r1_zone3_parse_from_link_z3_put()  55..AA
 * 3. USART10/r1_link STOP：r1_zone3_parse_from_usart10_stop()  EE 04 00 EA FF
 *---------------------------------------------------------------------*/
#include "r1_zone3_parse.h"

#include "app_zone3.h"
#include "r1_link_z3_cmd.h"
#include "r1_link_z3_put.h"

#include <stddef.h>

static void r1_zone3_parse_post(app_zone3_cmd_id_t id, uint8_t raw_cmd, uint8_t put_sub)
{
    app_zone3_r1_cmd_t z3;

    if (id == APP_Z3_CMD_NONE)
    {
        return;
    }

    z3.id = id;
    z3.seq = 0U;
    z3.raw_cmd = raw_cmd;
    z3.put_sub = put_sub;
    AppZone3_PostR1Cmd(&z3);
}

static uint8_t r1_zone3_link_z3_cmd_wire_to_z3(uint8_t wire_id, app_zone3_cmd_id_t *out_id)
{
    if (out_id == NULL || wire_id == 0U || wire_id > R1_LINK_Z3_CMD_WIRE_CMD_ID_MAX)
    {
        return 0U;
    }

    if (wire_id <= (uint8_t)APP_Z3_CMD_UP_R1)
    {
        *out_id = (app_zone3_cmd_id_t)wire_id;
        return 1U;
    }
    if (wire_id == R1_LINK_Z3_CMD_WIRE_GET_KFS_G1)
    {
        *out_id = APP_Z3_CMD_GET_KFS_G1;
        return 1U;
    }
    if (wire_id == R1_LINK_Z3_CMD_WIRE_GET_KFS_G2)
    {
        *out_id = APP_Z3_CMD_GET_KFS_G2;
        return 1U;
    }

    return 0U;
}

void r1_zone3_parse_from_link_z3_put(uint8_t wire_cmd_id, uint8_t raw_cmd)
{
    if (wire_cmd_id == R1_LINK_Z3_PUT_WIRE_CMD_ID_PUT_L3)
    {
        r1_zone3_parse_post(APP_Z3_CMD_PUT_KFS_ON_R1, raw_cmd, R1_LINK_Z3_CMD_PUT_SUB_NONE);
    }
}

void r1_zone3_parse_from_link_z3_cmd(uint8_t cmd_id, uint8_t put_sub)
{
    app_zone3_cmd_id_t id = APP_Z3_CMD_NONE;

    if (r1_zone3_link_z3_cmd_wire_to_z3(cmd_id, &id) == 0U)
    {
        return;
    }

    r1_zone3_parse_post(id, cmd_id, put_sub);
}

void r1_zone3_parse_from_usart10_stop(void)
{
    r1_zone3_parse_post(APP_Z3_CMD_STOP_ACTION,
                        (uint8_t)APP_Z3_CMD_STOP_ACTION,
                        R1_LINK_Z3_CMD_PUT_SUB_NONE);
}
