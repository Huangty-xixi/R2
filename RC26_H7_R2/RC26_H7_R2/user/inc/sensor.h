#ifndef __LASER_UART_H
#define __LASER_UART_H

#include "main.h"

#define DISTANCE_MIN    20
#define DISTANCE_MAX    4000
#define CONFIDENCE_MAX  62

#ifndef LASER_SUDDEN_JUMP_MM_DEFAULT
#define LASER_SUDDEN_JUMP_MM_DEFAULT  100U
#endif

#define SENSOR_TINYF_RECV_BUF_CAP     16U
#define SENSOR_TINYF_HEAD_BYTE        0x20U
#define SENSOR_TINYF_COMMA_BYTE       0x2CU
#define SENSOR_TINYF_LF_BYTE          0x0AU
#define SENSOR_TINYF_CR_BYTE          0x0DU
#define SENSOR_TINYF_DIST_STR_MAX     5U
#define SENSOR_TINYF_CONF_STR_MAX     2U

typedef struct {
    volatile uint16_t distance;
    volatile uint8_t  confidence;
    volatile uint8_t  ready;
    volatile uint8_t  sudden_increase;
    volatile uint8_t  sudden_decrease;
} Laser_t;

extern Laser_t laser1;

void Laser_Init(UART_HandleTypeDef *huart7);
void Laser_UART7_OnRxByte(uint8_t rx_byte);
void Laser_UART7_ErrorRecover(void);

uint8_t Laser_GetSuddenIncrease(const Laser_t *laser);
void Laser_ClearSuddenIncrease(Laser_t *laser);
uint8_t Laser_GetSuddenDecrease(const Laser_t *laser);
void Laser_ClearSuddenDecrease(Laser_t *laser);

#endif
