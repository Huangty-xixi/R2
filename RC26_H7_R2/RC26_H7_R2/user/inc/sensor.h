#ifndef __LASER_UART_H
#define __LASER_UART_H

#include "main.h"

#define DISTANCE_MIN    20
#define DISTANCE_MAX    4000
#define CONFIDENCE_MAX  62

typedef struct {
    volatile uint16_t distance;
    volatile uint8_t  confidence;
    volatile uint8_t  ready;
} Laser_t;

extern Laser_t laser1;  // UART7
extern Laser_t laser2;  // UART10

uint8_t Read_PE0_State(void);
void Laser_Init(UART_HandleTypeDef *huart7, UART_HandleTypeDef *huart10);

#endif