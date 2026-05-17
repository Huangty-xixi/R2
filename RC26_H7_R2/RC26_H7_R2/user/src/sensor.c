#include "sensor.h"
#include <string.h>
#include <stdlib.h>

Laser_t laser1 = {0};

static UART_HandleTypeDef *s_huart7 = NULL;

static uint16_t s_prev_dist = 0U;
static uint8_t s_has_prev_dist = 0U;

static uint8_t s_recv_buf[SENSOR_TINYF_RECV_BUF_CAP] = {0};
static uint8_t s_parse_index = 0U;
static uint8_t s_parsing = 0U;
static uint8_t s_comma_pos = 0U;

static void sensor_uart7_parser_reset(void)
{
    s_parse_index = 0U;
    s_parsing = 0U;
    s_comma_pos = 0U;
}

static void sensor_uart7_rx_flush(UART_HandleTypeDef *huart)
{
    uint32_t guard = 50000U;

    if (huart == NULL) {
        return;
    }

    while ((__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET) && (guard > 0U)) {
        (void)(huart->Instance->RDR & 0xFFU);
        guard--;
    }

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
}

static void sensor_uart7_tinyf_processing_data(uint8_t rx_data, Laser_t *laser)
{
    if (laser == NULL) {
        return;
    }

    if (s_parse_index >= SENSOR_TINYF_RECV_BUF_CAP) {
        sensor_uart7_parser_reset();
        return;
    }

    s_recv_buf[s_parse_index++] = rx_data;

    switch (s_parsing) {
    case 0U:
        if (rx_data == SENSOR_TINYF_HEAD_BYTE) {
            s_parsing = 1U;
            s_parse_index = 1U;
        } else {
            s_parse_index = 0U;
        }
        break;

    case 1U:
        if (rx_data == SENSOR_TINYF_COMMA_BYTE) {
            s_parsing = 2U;
            s_comma_pos = (uint8_t)(s_parse_index - 1U);
        }
        break;

    case 2U:
        if (rx_data == SENSOR_TINYF_HEAD_BYTE) {
            s_parsing = 3U;
        } else {
            sensor_uart7_parser_reset();
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

            if (s_comma_pos >= 1U) {
                dist_len = (uint8_t)(s_comma_pos - 1U);
                if (dist_len > SENSOR_TINYF_DIST_STR_MAX) {
                    dist_len = SENSOR_TINYF_DIST_STR_MAX;
                }
            }

            conf_start = (uint8_t)(s_comma_pos + 2U);
            if (conf_start >= s_parse_index) {
                conf_len = 0U;
            } else {
                conf_len = (uint8_t)(s_parse_index - conf_start - 1U);
            }
            if (conf_len > SENSOR_TINYF_CONF_STR_MAX) {
                conf_len = SENSOR_TINYF_CONF_STR_MAX;
            }

            if ((dist_len > 0U) && (s_comma_pos >= 1U)) {
                (void)memcpy(dist_str, &s_recv_buf[1], dist_len);
                dist_str[dist_len] = '\0';
            }
            if (conf_len > 0U) {
                (void)memcpy(conf_str, &s_recv_buf[conf_start], conf_len);
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
            sensor_uart7_parser_reset();
        } else if (rx_data != SENSOR_TINYF_CR_BYTE) {
            sensor_uart7_parser_reset();
        }
        break;

    default:
        sensor_uart7_parser_reset();
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

static void laser_uart7_enable_rx_irq(void)
{
    if (s_huart7 == NULL) {
        return;
    }

    ATOMIC_SET_BIT(s_huart7->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
    ATOMIC_SET_BIT(s_huart7->Instance->CR3, USART_CR3_EIE);
}

void Laser_UART7_OnRxByte(uint8_t rx_byte)
{
    sensor_uart7_tinyf_processing_data(rx_byte, &laser1);
}

void Laser_UART7_ErrorRecover(void)
{
    if (s_huart7 == NULL) {
        return;
    }

    (void)HAL_UART_AbortReceive_IT(s_huart7);
    sensor_uart7_rx_flush(s_huart7);
    sensor_uart7_parser_reset();
    s_huart7->ErrorCode = HAL_UART_ERROR_NONE;
    s_huart7->RxState = HAL_UART_STATE_READY;
    laser_uart7_enable_rx_irq();
}

void Laser_Init(UART_HandleTypeDef *huart7)
{
    s_huart7 = huart7;
    s_has_prev_dist = 0U;
    s_prev_dist = 0U;
    laser1.ready = 0U;
    sensor_uart7_parser_reset();

    if (s_huart7 == NULL) {
        return;
    }

    (void)HAL_UART_AbortReceive_IT(s_huart7);
    sensor_uart7_rx_flush(s_huart7);
    s_huart7->ErrorCode = HAL_UART_ERROR_NONE;
    s_huart7->RxState = HAL_UART_STATE_READY;
    s_huart7->gState = HAL_UART_STATE_READY;
    laser_uart7_enable_rx_irq();
}
