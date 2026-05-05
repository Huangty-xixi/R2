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

extern Laser_t laser1;

void parse_laser_byte(uint8_t byte);
void Laser_Init(UART_HandleTypeDef *huart7);

#endif
