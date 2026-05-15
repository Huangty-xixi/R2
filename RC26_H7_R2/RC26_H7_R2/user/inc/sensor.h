#ifndef __LASER_UART_H
#define __LASER_UART_H

#include "main.h"

#define DISTANCE_MIN    20
#define DISTANCE_MAX    4000
#define CONFIDENCE_MAX  62

#ifndef LASER_SUDDEN_JUMP_MM_DEFAULT
#define LASER_SUDDEN_JUMP_MM_DEFAULT  200U
#endif

typedef struct {
    volatile uint16_t distance;
    volatile uint8_t  confidence;
    volatile uint8_t  ready;
    /** 解析到单帧突增时置 1，保持到 Laser_ClearSuddenIncrease，正常帧不会自动清 0 */
    volatile uint8_t  sudden_increase;
} Laser_t;

extern Laser_t laser1;  // UART7
extern Laser_t laser2;  // UART10

/** 调试用：Watch 里加 g_laser_debug，每帧合法测距后更新（mm / 置信度） */
typedef struct {
    volatile uint16_t dist_mm_1;   /* UART7 → laser1 */
    volatile uint16_t dist_mm_2;   /* UART10 → laser2 */
    volatile uint8_t  confidence_1;
    volatile uint8_t  confidence_2;
} laser_debug_watch_t;

extern laser_debug_watch_t g_laser_debug;

uint8_t Read_PE0_State(void);
void Laser_Init(UART_HandleTypeDef *huart7, UART_HandleTypeDef *huart10);
void Laser_ClearSuddenIncrease(Laser_t *laser);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
