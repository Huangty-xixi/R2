#include "sensor.h"
#include <string.h>
#include <stdlib.h>

Laser_t laser1 = {0};
Laser_t laser2 = {0};

static UART_HandleTypeDef *huart7_ptr;
static UART_HandleTypeDef *huart10_ptr;

// 每个串口独立的接收字节
static uint8_t u7_rx;
static uint8_t u10_rx;
uint8_t state;

uint8_t Read_PE0_State(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
    return (state == GPIO_PIN_SET) ? 1 : 0;
}

// 解析函数
static void parse_byte(uint8_t byte, Laser_t *laser) {
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
        case 0: // 等待空格 0x20
            if (byte == 0x20) {
                state = 1;
                idx = 1;
            } else idx = 0;
            break;

        case 1: // 等待逗号 0x2C
            if (byte == 0x2C) {
                state = 2;
                comma = idx - 1;
            }
            break;

        case 2: // 等待分隔空格
            if (byte == 0x20) state = 3;
            else idx = state = comma = 0;
            break;

        case 3: // 等待换行 0x0A
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
                    laser->distance = dist;
                    laser->confidence = conf;
                    laser->ready = 1;
                } else {
                    laser->distance = 0;
                    laser->confidence = 0;
                }
                idx = state = comma = 0;
            }
            break;

        default:
            idx = state = comma = 0;
            break;
    }
}

// 初始化
void Laser_Init(UART_HandleTypeDef *h7, UART_HandleTypeDef *h10) {
    huart7_ptr = h7;
    huart10_ptr = h10;

    // 启动中断接收（正确写法）
    HAL_UART_Receive_IT(huart7_ptr, &u7_rx, 1);
    HAL_UART_Receive_IT(huart10_ptr, &u10_rx, 1);
}

// 接收完成回调（HAL自动调用）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == huart7_ptr) {
        parse_byte(u7_rx, &laser1);      // 解析数据
        HAL_UART_Receive_IT(huart7_ptr, &u7_rx, 1); // 重新开启中断
    }

    if (huart == huart10_ptr) {
        parse_byte(u10_rx, &laser2);
        HAL_UART_Receive_IT(huart10_ptr, &u10_rx, 1);
    }
}