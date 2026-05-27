/**
 * @file r1_zone3_parse.c
 */

#include "r1_zone3_parse.h"

#include "app_zone3.h"
#include "r1_usart1_proto.h"
#include "r1_usart3_proto.h"
#include "r1_link_z3_stop.h"

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
    r1_zone3_parse_post(APP_Z3_CMD_STOP_ACTION, R1_LINK_Z3_STOP_DATA);
}
