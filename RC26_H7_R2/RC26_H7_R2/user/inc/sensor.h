#ifndef __LASER_UART_H
#define __LASER_UART_H

#include "main.h"

#define DISTANCE_MIN    20
#define DISTANCE_MAX    4000
#define CONFIDENCE_MAX  62

#ifndef LASER_SUDDEN_JUMP_MM_DEFAULT
#define LASER_SUDDEN_JUMP_MM_DEFAULT  100U
#endif

/** RX buffer capacity (datasheet / example uses <= 11 bytes typical) */
#define SENSOR_TINYF_RECV_BUF_CAP     16U

/** TinyF ASCII frame: ' ' + distance + ", " + confidence + '\n' */
#define SENSOR_TINYF_HEAD_BYTE        0x20U
#define SENSOR_TINYF_COMMA_BYTE       0x2CU
#define SENSOR_TINYF_LF_BYTE          0x0AU

#define SENSOR_TINYF_DIST_STR_MAX     5U
#define SENSOR_TINYF_CONF_STR_MAX     2U

typedef struct {
    volatile uint16_t distance;
    volatile uint8_t  confidence;
    /** 完整帧结束(0x0A)置 1，业务读完后自行清 0 */
    volatile uint8_t  ready;
    /** 合法帧间突增置 1，业务处理完后调用 Laser_ClearSuddenIncrease 清 0 */
    volatile uint8_t  sudden_increase;
    /** 合法帧间突减置 1，业务处理完后调用 Laser_ClearSuddenDecrease 清 0 */
    volatile uint8_t  sudden_decrease;
} Laser_t;


extern Laser_t laser1;  /* UART7 激光测距 */

void Laser_Init(UART_HandleTypeDef *huart7);
/** @return 1 表示突增标志有效，0 表示无 */
uint8_t Laser_GetSuddenIncrease(const Laser_t *laser);
void Laser_ClearSuddenIncrease(Laser_t *laser);
/** @return 1 表示突减标志有效，0 表示无 */
uint8_t Laser_GetSuddenDecrease(const Laser_t *laser);
void Laser_ClearSuddenDecrease(Laser_t *laser);
/** 激光 IT 接收：在 sensor.c 中实现 HAL_UART_RxCpltCallback（仅 UART7） */

#endif
