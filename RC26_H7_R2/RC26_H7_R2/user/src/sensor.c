#include "sensor.h"
#include <string.h>
#include <stdlib.h>

Laser_t laser1 = {0};

static UART_HandleTypeDef *huart7_ptr;
static uint8_t u7_rx_byte;

void Laser_Init(UART_HandleTypeDef *huart7)
{
    huart7_ptr = huart7;
    HAL_UART_Receive_IT(huart7_ptr, &u7_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == huart7_ptr) {
        parse_laser_byte(u7_rx_byte);
        HAL_UART_Receive_IT(huart7_ptr, &u7_rx_byte, 1);
    }
    /* 其他UART现在不在这里处理 - new_remote_control用USB，USART10不再使用 */
}

void parse_laser_byte(uint8_t byte) {
    static uint8_t buf[16];
    static uint8_t idx = 0;
    static uint8_t state = 0;
    static uint8_t comma = 0;

    if (idx >= 16) {
        idx = state = comma = 0;
        return;
    }
    buf[idx++] = byte;

    switch (state) {
        case 0:
            if (byte == 0x20) {
                state = 1;
                idx = 1;
            } else idx = 0;
            break;

        case 1:
            if (byte == 0x2C) {
                state = 2;
                comma = idx - 1;
            }
            break;

        case 2:
            if (byte == 0x20) state = 3;
            else idx = state = comma = 0;
            break;

        case 3:
            if (byte == 0x0A) {
                uint8_t d_len = comma - 1;
                if (d_len > 5) d_len = 5;
                char d_str[6] = {0};
                memcpy(d_str, buf + 1, d_len);

                uint8_t c_start = comma + 2;
                uint8_t c_len = idx - c_start - 1;
                if (c_len > 2) c_len = 2;
                char c_str[3] = {0};
                memcpy(c_str, buf + c_start, c_len);

                uint16_t dist = atoi(d_str);
                uint8_t conf = atoi(c_str);

                if (dist >= DISTANCE_MIN && dist <= DISTANCE_MAX && conf <= CONFIDENCE_MAX) {
                    laser1.distance = dist;
                    laser1.confidence = conf;
                    laser1.ready = 1;
                } else {
                    laser1.distance = 0;
                    laser1.confidence = 0;
                }
                idx = state = comma = 0;
            }
            break;

        default:
            idx = state = comma = 0;
            break;
    }
}
