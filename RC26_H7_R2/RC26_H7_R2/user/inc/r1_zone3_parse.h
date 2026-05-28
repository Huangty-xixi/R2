/**
 * @file r1_zone3_parse.h
 * @brief R1 线协议 -> app_zone3_r1_cmd_t，成功后 PostR1Cmd
 *
 * USART1：wire cmd_id 1=放三层；2/3 预留
 * USART3：wire cmd_id 1~5 = P2/P3/P4/STOP/上R1（无 6）
 * USART10：EE 04 EA FF（与 USART3 STOP 同帧，仅收 cmd_id=4）-> STOP_ACTION
 */
#ifndef R1_ZONE3_PARSE_H
#define R1_ZONE3_PARSE_H

#include <stdint.h>

void r1_zone3_parse_from_usart1(uint8_t wire_cmd_id, uint8_t raw_cmd);

void r1_zone3_parse_from_usart3(uint8_t data);

/** USART10 EE 04 EA FF -> STOP_ACTION */
void r1_zone3_parse_from_usart10_stop(void);

#endif /* R1_ZONE3_PARSE_H */
