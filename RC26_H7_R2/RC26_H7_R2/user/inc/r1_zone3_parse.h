/**
 * @file r1_zone3_parse.h
 * @brief R1 线协议 -> app_zone3_r1_cmd_t，成功后 PostR1Cmd
 *
 * USART1/r1_link_z3_cmd：EE..FF 5B，wire 1~5=P2/P3/P4/STOP/上R1，6/7=取kfs位1/2
 * USART10/r1_link：55..AA 放三层；EE..FF STOP(4) / GET_KFS(6/7) / 放料(1~3)
 */
#ifndef R1_ZONE3_PARSE_H
#define R1_ZONE3_PARSE_H

#include <stdint.h>

void r1_zone3_parse_from_link_z3_cmd(uint8_t cmd_id, uint8_t put_sub);

void r1_zone3_parse_from_link_z3_put(uint8_t wire_cmd_id, uint8_t raw_cmd);

/** USART10 EE 04 00 EA FF -> STOP_ACTION */
void r1_zone3_parse_from_usart10_stop(void);

#endif /* R1_ZONE3_PARSE_H */
