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

extern Laser_t laser1;  /* UART7 激光测距 */

/** 调试用：Watch 里只加 g_laser_debug（测距 + UART7 IT 统计） */
typedef struct {
    volatile uint16_t dist_mm_1;     /* UART7 合法帧距离 mm */
    volatile uint8_t  confidence_1;
    volatile uint32_t rx_cplt_cnt;   /* UART7 每收满 1 字节进入 RxCplt 次数 */
    volatile uint32_t init_rx_ret;  /* Laser_Init 里 HAL_UART_Receive_IT，0=HAL_OK */
} laser_debug_watch_t;

extern laser_debug_watch_t g_laser_debug;

uint8_t Read_PE0_State(void);
void Laser_Init(UART_HandleTypeDef *huart7);
void Laser_ClearSuddenIncrease(Laser_t *laser);
/** 激光 IT 接收：在 sensor.c 中实现 HAL_UART_RxCpltCallback（仅 UART7） */

#endif
