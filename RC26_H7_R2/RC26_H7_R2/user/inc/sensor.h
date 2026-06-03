#ifndef __LASER_UART_H
#define __LASER_UART_H

#include "main.h"

#define DISTANCE_MIN    20
#define DISTANCE_MAX    4000
#define CONFIDENCE_MAX  62

#ifndef LASER_SUDDEN_JUMP_MM_DEFAULT
#define LASER_SUDDEN_JUMP_MM_DEFAULT  100U
#endif

/* TinyF UART text frame: [0x20][dist][0x2C 0x20][conf][0x0A]  e.g. " 327, 61\n" */
#define LASER_FRAME_HEAD_BYTE       0x20U
#define LASER_FRAME_COMMA_BYTE      0x2CU
#define LASER_FRAME_TAIL_BYTE       0x0AU
#define LASER_FRAME_CR_BYTE         0x0DU
#define LASER_DIST_ASCII_MAX        5U
#define LASER_CONF_ASCII_MAX        2U

typedef struct {
    volatile uint16_t distance;
    volatile uint8_t  confidence;
    volatile uint8_t  ready;
    volatile uint8_t  sudden_increase;
    volatile uint8_t  sudden_decrease;
} Laser_t;

/** Keil Watch: laser1_diag */
typedef struct {
    volatile uint32_t irq_cnt;
    volatile uint32_t rx_byte_cnt;
    volatile uint32_t frame_ok_cnt;
    volatile uint32_t parse_fail_cnt;
    volatile uint32_t ore_cnt;
    volatile uint32_t fe_cnt;
    volatile uint32_t rxneie_restore_cnt;
    volatile uint8_t  last_rx_byte;
    volatile uint32_t isr_last;
} Laser_Diag_t;

extern Laser_t laser1;
extern Laser_Diag_t laser1_diag;

void Laser_Init(UART_HandleTypeDef *huart7);
void Laser_UART7_OnRxByte(uint8_t rx_byte);
void Laser_UART7_RxIrqSanityCheck(void);

uint8_t Laser_GetSuddenIncrease(const Laser_t *laser);
void Laser_ClearSuddenIncrease(Laser_t *laser);
uint8_t Laser_GetSuddenDecrease(const Laser_t *laser);
void Laser_ClearSuddenDecrease(Laser_t *laser);

#endif
