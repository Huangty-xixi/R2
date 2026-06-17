/**
 * @file r1_zone3_parse.c
 * @brief 全区指令解析（统一入口）
 *
 * === 业务调用链 ===
 * 3个IR源→3条路径，汇聚到 AppZone3_PostR1Cmd：
 * 
 * ① USART1 EE..FF(1~7): r1_zone3_parse_from_link_z3_cmd(data)
 *    → r1_zone3_link_z3_cmd_wire_to_z3(data, &id)      // wire→zone3
 *    → r1_zone3_parse_post(id, data)
 *    → AppZone3_PostR1Cmd(&z3)
 * 
 * ② USART10 55..AA Put: r1_zone3_parse_from_link_z3_put(cmd_id,raw)
 *    → 仅PUT_L3 → r1_zone3_parse_post(APP_Z3_CMD_PUT_KFS_ON_R1)
 *    → AppZone3_PostR1Cmd(&z3)
 * 
 * ③ USART10 EE..FF STOP: r1_zone3_parse_from_usart10_stop()
 *    → r1_zone3_parse_post(APP_Z3_CMD_STOP_ACTION)
 *    → AppZone3_PostR1Cmd(&z3)
 * 
 * ④ USART10 EE..FF GET_KFS: r1_zone3_parse_from_link_z3_cmd(cmd_id)
 *    → 同路径①
 */
/*---------------------------------------------------------------------
 * 【3个来源入口】
 * 1. USART1/r1_link_z3_cmd：r1_zone3_parse_from_link_z3_cmd()  EE..FF cmd 1~5
 * 2. USART10/r1_link 放三层：r1_zone3_parse_from_link_z3_put()  55..AA
 * 3. USART10/r1_link STOP：r1_zone3_parse_from_usart10_stop()  EE 04 EA FF
 *---------------------------------------------------------------------*/
#include "r1_zone3_parse.h"

#include "app_zone3.h"
#include "r1_link_z3_cmd.h"
#include "r1_link_z3_put.h"

#include <stddef.h>

static void r1_zone3_parse_post(app_zone3_cmd_id_t id, uint8_t raw)
{
    app_zone3_r1_cmd_t z3;

    if (id == APP_Z3_CMD_NONE)
    {
        return;
    /* TODO: G1/G2 wire_id -> cmd_id 映射(帧格式待定) */
    /* if (wire_id == TBD_GET_KFS_G1) { *out_id = APP_Z3_CMD_GET_KFS_G1; return 1U; } */
    /* if (wire_id == TBD_GET_KFS_G2) { *out_id = APP_Z3_CMD_GET_KFS_G2; return 1U; } */
    }

    z3.id = id;
    z3.seq = 0U;
    z3.raw_cmd = raw;
    AppZone3_PostR1Cmd(&z3);
}

static uint8_t r1_zone3_link_z3_cmd_wire_to_z3(uint8_t wire_id, app_zone3_cmd_id_t *out_id)
{
    if (out_id == NULL || wire_id == 0U || wire_id > R1_LINK_Z3_CMD_WIRE_CMD_ID_MAX)
    {
        return 0U;
    }

    if (wire_id == (uint8_t)APP_Z3_CMD_PUT_KFS_ON_R1)
    {
        return 0U;
    }

    *out_id = (app_zone3_cmd_id_t)wire_id;
    return 1U;
}

void r1_zone3_parse_from_link_z3_put(uint8_t wire_cmd_id, uint8_t raw_cmd)
{
    if (wire_cmd_id == R1_LINK_Z3_PUT_WIRE_CMD_ID_PUT_L3)
    {
        r1_zone3_parse_post(APP_Z3_CMD_PUT_KFS_ON_R1, raw_cmd);
    }
}

void r1_zone3_parse_from_link_z3_cmd(uint8_t data)
{
    app_zone3_cmd_id_t id = APP_Z3_CMD_NONE;

    if (r1_zone3_link_z3_cmd_wire_to_z3(data, &id) == 0U)
    {
        return;
    }

    r1_zone3_parse_post(id, data);
}

void r1_zone3_parse_from_usart10_stop(void)
{
    r1_zone3_parse_post(APP_Z3_CMD_STOP_ACTION, (uint8_t)APP_Z3_CMD_STOP_ACTION);
}
