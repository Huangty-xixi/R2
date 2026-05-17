#include "sensor.h"
#include <string.h>
#include <stdlib.h>

Laser_t laser1 = {0};

static UART_HandleTypeDef *s_huart7 = NULL;
static uint8_t s_uart7_rx = 0;

static uint16_t s_prev_dist = 0U;
static uint8_t s_has_prev_dist = 0U;

static void sensor_uart7_tinyf_processing_data(uint8_t rx_data, Laser_t *laser)
{
    static uint8_t recv_buf[SENSOR_TINYF_RECV_BUF_CAP] = {0};
    static uint8_t index = 0;
    static uint8_t parsing = 0;
    static uint8_t comma_pos = 0;

    if (laser == NULL) {
        return;
    }

    if (index >= SENSOR_TINYF_RECV_BUF_CAP) {
        index = 0;
        parsing = 0;
        comma_pos = 0;
        return;
    }

    recv_buf[index++] = rx_data;

    switch (parsing) {
    case 0U:
        if (rx_data == SENSOR_TINYF_HEAD_BYTE) {
            parsing = 1U;
            index = 1U;
        } else {
            index = 0U;
        }
        break;

    case 1U:
        if (rx_data == SENSOR_TINYF_COMMA_BYTE) {
            parsing = 2U;
            comma_pos = (uint8_t)(index - 1U);
        }
        break;

    case 2U:
        if (rx_data == SENSOR_TINYF_HEAD_BYTE) {
            parsing = 3U;
        } else {
            parsing = 0U;
            index = 0U;
            comma_pos = 0U;
        }
        break;

    case 3U:
        if (rx_data == SENSOR_TINYF_LF_BYTE) {
            char dist_str[SENSOR_TINYF_DIST_STR_MAX + 1U] = {0};
            char conf_str[SENSOR_TINYF_CONF_STR_MAX + 1U] = {0};
            uint8_t dist_len = 0U;
            uint8_t conf_start;
            uint8_t conf_len;
            int dist_val;
            int conf_val;

            if (comma_pos >= 1U) {
                dist_len = (uint8_t)(comma_pos - 1U);
                if (dist_len > SENSOR_TINYF_DIST_STR_MAX) {
                    dist_len = SENSOR_TINYF_DIST_STR_MAX;
                }
            }

            conf_start = (uint8_t)(comma_pos + 2U);
            if (conf_start >= index) {
                conf_len = 0U;
            } else {
                conf_len = (uint8_t)(index - conf_start - 1U);
            }
            if (conf_len > SENSOR_TINYF_CONF_STR_MAX) {
                conf_len = SENSOR_TINYF_CONF_STR_MAX;
            }

            if ((dist_len > 0U) && (comma_pos >= 1U)) {
                (void)memcpy(dist_str, &recv_buf[1], dist_len);
                dist_str[dist_len] = '\0';
            }
            if (conf_len > 0U) {
                (void)memcpy(conf_str, &recv_buf[conf_start], conf_len);
                conf_str[conf_len] = '\0';
            }

            dist_val = atoi(dist_str);
            conf_val = atoi(conf_str);

            if (dist_val < 0) {
                dist_val = 0;
            }
            if (conf_val < 0) {
                conf_val = 0;
            }

            laser->distance = (uint16_t)dist_val;
            laser->confidence = (uint8_t)conf_val;

            if (laser->distance < (uint16_t)DISTANCE_MIN ||
                laser->distance > (uint16_t)DISTANCE_MAX ||
                laser->confidence > CONFIDENCE_MAX) {
                laser->distance = 0U;
                laser->confidence = 0U;
            } else {
                if (s_has_prev_dist != 0U) {
                    if (laser->distance > s_prev_dist &&
                        (laser->distance - s_prev_dist) >= LASER_SUDDEN_JUMP_MM_DEFAULT) {
                        laser->sudden_increase = 1U;
                    } else if (s_prev_dist > laser->distance &&
                               (s_prev_dist - laser->distance) >= LASER_SUDDEN_JUMP_MM_DEFAULT) {
                        laser->sudden_decrease = 1U;
                    }
                }
                s_has_prev_dist = 1U;
                s_prev_dist = laser->distance;
            }

            laser->ready = 1U;

            index = 0U;
            parsing = 0U;
            comma_pos = 0U;
        }
        break;

    default:
        parsing = 0U;
        index = 0U;
        comma_pos = 0U;
        break;
    }
}

uint8_t Laser_GetSuddenIncrease(const Laser_t *laser)
{
    if (laser == NULL) {
        return 0U;
    }
    return (laser->sudden_increase != 0U) ? 1U : 0U;
}

void Laser_ClearSuddenIncrease(Laser_t *laser)
{
    if (laser != NULL) {
        laser->sudden_increase = 0U;
    }
}

uint8_t Laser_GetSuddenDecrease(const Laser_t *laser)
{
    if (laser == NULL) {
        return 0U;
    }
    return (laser->sudden_decrease != 0U) ? 1U : 0U;
}

void Laser_ClearSuddenDecrease(Laser_t *laser)
{
    if (laser != NULL) {
        laser->sudden_decrease = 0U;
    }
}

void Laser_Init(UART_HandleTypeDef *huart7)
{
    s_huart7 = huart7;
    s_has_prev_dist = 0U;
    s_prev_dist = 0U;

    if (s_huart7 != NULL) {
        (void)HAL_UART_Receive_IT(s_huart7, &s_uart7_rx, 1U);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == s_huart7)) {
        (void)HAL_UART_Receive_IT(s_huart7, &s_uart7_rx, 1U);
        sensor_uart7_tinyf_processing_data(s_uart7_rx, &laser1);
    }
}
