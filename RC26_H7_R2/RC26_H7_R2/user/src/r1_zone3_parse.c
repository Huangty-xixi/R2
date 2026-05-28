/**
 * @file r1_zone3_parse.c
 */
/*---------------------------------------------------------------------
 * r1_zone3_parse.c 全区指令解析（统一入口）
 * 功能：接收 3 个串口的 R1 指令，统一翻译后交给 Zone3 执行
 *
 * 【3个来源入口】
 * 1. USART3  ：r1_zone3_parse_from_usart3()
 * 2. USART1  ：r1_zone3_parse_from_usart1()
 * 3. USART10 ：r1_zone3_parse_from_usart10_stop()  EE 04 EA FF（STOP，cmd_id=4）
 *
 * 【内部流程】
 * 1. 各入口接收原始命令
 * 2. 翻译：原始值 → 内部命令ID
 * 3. 提交：r1_zone3_parse_post()
 * 4. 执行：AppZone3_PostR1Cmd()
 *
 * 作用：Zone3 所有外部指令统一调度中心
 *---------------------------------------------------------------------*/
#include "r1_zone3_parse.h"

#include "app_zone3.h"
#include "r1_usart1_proto.h"
#include "r1_usart3_proto.h"

#include <stddef.h>

static void r1_zone3_parse_post(app_zone3_cmd_id_t id, uint8_t raw)
{
    app_zone3_r1_cmd_t z3;

    if (id == APP_Z3_CMD_NONE)
    {
        return;
    }

    z3.id = id;
    z3.seq = 0U;
    z3.raw_cmd = raw;
    AppZone3_PostR1Cmd(&z3);
}

static uint8_t r1_zone3_usart3_wire_to_z3(uint8_t wire_id, app_zone3_cmd_id_t *out_id)
{
    if (out_id == NULL || wire_id == 0U || wire_id > R1_USART3_WIRE_CMD_ID_MAX)
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

void r1_zone3_parse_from_usart1(uint8_t wire_cmd_id, uint8_t raw_cmd)
{
    if (wire_cmd_id == R1_USART1_WIRE_CMD_ID_PUT_L3)
    {
        r1_zone3_parse_post(APP_Z3_CMD_PUT_KFS_ON_R1, raw_cmd);
    }
}

void r1_zone3_parse_from_usart3(uint8_t data)
{
    app_zone3_cmd_id_t id = APP_Z3_CMD_NONE;

    if (r1_zone3_usart3_wire_to_z3(data, &id) == 0U)
    {
        return;
    }

    r1_zone3_parse_post(id, data);
}

void r1_zone3_parse_from_usart10_stop(void)
{
    r1_zone3_parse_post(APP_Z3_CMD_STOP_ACTION, (uint8_t)APP_Z3_CMD_STOP_ACTION);
}
