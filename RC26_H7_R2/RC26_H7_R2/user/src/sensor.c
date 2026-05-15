#include "sensor.h"
#include <string.h>
#include <stdlib.h>

Laser_t laser1 = {0};
Laser_t laser2 = {0};

laser_debug_watch_t g_laser_debug = {0};

static UART_HandleTypeDef *huart7_ptr;
static UART_HandleTypeDef *huart10_ptr;

static uint8_t u7_rx;
static uint8_t u10_rx;
uint8_t state;

typedef struct {
    uint8_t  buf[16];
    uint8_t  idx;
    uint8_t  parse_state;
    uint8_t  comma;
    uint16_t prev_dist;
    uint8_t  has_prev;
} laser_parse_ctx_t;

static laser_parse_ctx_t s_parse_ctx1;
static laser_parse_ctx_t s_parse_ctx2;

static void laser_on_valid_frame(Laser_t *laser, laser_parse_ctx_t *ctx, uint16_t dist, uint8_t conf)
{
    if (ctx->has_prev != 0U &&
        dist > ctx->prev_dist &&
        (dist - ctx->prev_dist) >= LASER_SUDDEN_JUMP_MM_DEFAULT)
    {
        laser->sudden_increase = 1U;
    }

    ctx->has_prev = 1U;
    ctx->prev_dist = dist;
    laser->distance = dist;
    laser->confidence = conf;
    laser->ready = 1U;

    if (laser == &laser1)
    {
        g_laser_debug.dist_mm_1 = dist;
        g_laser_debug.confidence_1 = conf;
    }
    else if (laser == &laser2)
    {
        g_laser_debug.dist_mm_2 = dist;
        g_laser_debug.confidence_2 = conf;
    }
}

static void parse_byte(uint8_t byte, Laser_t *laser, laser_parse_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    if (ctx->idx >= 16U)
    {
        ctx->idx = 0U;
        ctx->parse_state = 0U;
        ctx->comma = 0U;
        return;
    }
    ctx->buf[ctx->idx++] = byte;

    switch (ctx->parse_state)
    {
    case 0:
        if (byte == 0x20U)
        {
            ctx->parse_state = 1U;
            ctx->idx = 1U;
        }
        else
        {
            ctx->idx = 0U;
        }
        break;

    case 1:
        if (byte == 0x2CU)
        {
            ctx->parse_state = 2U;
            ctx->comma = (uint8_t)(ctx->idx - 1U);
        }
        break;

    case 2:
        if (byte == 0x20U)
        {
            ctx->parse_state = 3U;
        }
        else
        {
            ctx->idx = 0U;
            ctx->parse_state = 0U;
            ctx->comma = 0U;
        }
        break;

    case 3:
        if (byte == 0x0AU)
        {
            uint8_t d_len = (uint8_t)(ctx->comma - 1U);
            if (d_len > 5U)
            {
                d_len = 5U;
            }
            char d_str[6] = {0};
            memcpy(d_str, ctx->buf + 1, d_len);

            uint8_t c_start = (uint8_t)(ctx->comma + 2U);
            uint8_t c_len = (uint8_t)(ctx->idx - c_start - 1U);
            if (c_len > 2U)
            {
                c_len = 2U;
            }
            char c_str[3] = {0};
            memcpy(c_str, ctx->buf + c_start, c_len);

            uint16_t dist = (uint16_t)atoi(d_str);
            uint8_t conf = (uint8_t)atoi(c_str);

            if (dist >= DISTANCE_MIN && dist <= DISTANCE_MAX && conf <= CONFIDENCE_MAX)
            {
                laser_on_valid_frame(laser, ctx, dist, conf);
            }
            else
            {
                laser->distance = 0U;
                laser->confidence = 0U;
            }

            ctx->idx = 0U;
            ctx->parse_state = 0U;
            ctx->comma = 0U;
        }
        break;

    default:
        ctx->idx = 0U;
        ctx->parse_state = 0U;
        ctx->comma = 0U;
        break;
    }
}

void Laser_ClearSuddenIncrease(Laser_t *laser)
{
    if (laser != NULL)
    {
        laser->sudden_increase = 0U;
    }
}

uint8_t Read_PE0_State(void)
{
    GPIO_PinState pin = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
    return (pin == GPIO_PIN_SET) ? 1U : 0U;
}

void Laser_Init(UART_HandleTypeDef *h7, UART_HandleTypeDef *h10)
{
    huart7_ptr = h7;
    huart10_ptr = h10;

    (void)memset(&s_parse_ctx1, 0, sizeof(s_parse_ctx1));
    (void)memset(&s_parse_ctx2, 0, sizeof(s_parse_ctx2));

    HAL_UART_Receive_IT(huart7_ptr, &u7_rx, 1);
    HAL_UART_Receive_IT(huart10_ptr, &u10_rx, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == huart7_ptr)
    {
        parse_byte(u7_rx, &laser1, &s_parse_ctx1);
        HAL_UART_Receive_IT(huart7_ptr, &u7_rx, 1);
    }

    if (huart == huart10_ptr)
    {
        parse_byte(u10_rx, &laser2, &s_parse_ctx2);
        HAL_UART_Receive_IT(huart10_ptr, &u10_rx, 1);
    }
}
