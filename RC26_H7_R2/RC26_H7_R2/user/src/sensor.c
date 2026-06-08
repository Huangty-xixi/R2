#include "sensor.h"

Laser_t laser1 = {0};
Laser_Diag_t laser1_diag = {0};

static UART_HandleTypeDef *s_huart7 = NULL;

/* TinyF frame state machine */
#define ST_WAIT_HEAD  0U
#define ST_DIST       1U
#define ST_SEP        2U
#define ST_CONF       3U

static uint8_t  s_state = ST_WAIT_HEAD;
static uint16_t s_dist_acc = 0U;
static uint8_t  s_dist_len = 0U;
static uint16_t s_conf_acc = 0U;
static uint8_t  s_conf_len = 0U;

static uint16_t s_prev_dist = 0U;
static uint8_t  s_has_prev_dist = 0U;

/*
 * TinyF ASCII frame: [0x20] + dist(1..5) + [0x2C 0x20] + conf(1..2) + [0x0A]
 * Example: 20 32 38 38 32 2C 20 35 38 0A  -> " 2882, 58\n"
 */
static void sensor_uart7_on_rx_byte(uint8_t rx_byte, Laser_t *laser)
{
    if (laser == NULL) {
        return;
    }

    if (rx_byte == LASER_FRAME_CR_BYTE) {
        return;
    }

    switch (s_state) {

    case ST_WAIT_HEAD:
        if (rx_byte == LASER_FRAME_HEAD_BYTE) {
            s_state = ST_DIST;
            s_dist_acc = 0U;
            s_dist_len = 0U;
        }
        break;

    case ST_DIST:
        if (rx_byte == LASER_FRAME_COMMA_BYTE && s_dist_len >= 1U) {
            s_state = ST_SEP;
        } else if (rx_byte >= (uint8_t)'0' && rx_byte <= (uint8_t)'9'
                   && s_dist_len < LASER_DIST_ASCII_MAX) {
            s_dist_acc = (uint16_t)(s_dist_acc * 10U + (uint16_t)(rx_byte - (uint8_t)'0'));
            s_dist_len++;
        } else {
            s_state = ST_WAIT_HEAD;
            laser1_diag.parse_fail_cnt++;
        }
        break;

    case ST_SEP:
        if (rx_byte == LASER_FRAME_HEAD_BYTE) {
            s_state = ST_CONF;
            s_conf_acc = 0U;
            s_conf_len = 0U;
        } else {
            s_state = ST_WAIT_HEAD;
            laser1_diag.parse_fail_cnt++;
        }
        break;

    case ST_CONF:
        if (rx_byte == LASER_FRAME_TAIL_BYTE && s_conf_len >= 1U) {
            /* Frame complete -- validate */
            if (s_dist_acc >= (uint16_t)DISTANCE_MIN
                && s_dist_acc <= (uint16_t)DISTANCE_MAX
                && s_conf_acc >= 1U
                && s_conf_acc <= (uint16_t)CONFIDENCE_MAX) {

                laser->distance = s_dist_acc;
                laser->confidence = (uint8_t)s_conf_acc;

                if (s_has_prev_dist != 0U) {
                    if (laser->distance > s_prev_dist
                        && (uint16_t)(laser->distance - s_prev_dist) >= LASER_SUDDEN_JUMP_MM_DEFAULT) {
                        laser->sudden_increase = 1U;
                    } else if (s_prev_dist > laser->distance
                               && (uint16_t)(s_prev_dist - laser->distance) >= LASER_SUDDEN_JUMP_MM_DEFAULT) {
                        laser->sudden_decrease = 1U;
                    }
                }

                s_prev_dist = laser->distance;
                s_has_prev_dist = 1U;
                laser->ready = 1U;
                laser1_diag.frame_ok_cnt++;
            } else {
                laser1_diag.parse_fail_cnt++;
            }
            s_state = ST_WAIT_HEAD;
        } else if (rx_byte >= (uint8_t)'0' && rx_byte <= (uint8_t)'9'
                   && s_conf_len < LASER_CONF_ASCII_MAX) {
            s_conf_acc = (uint16_t)(s_conf_acc * 10U + (uint16_t)(rx_byte - (uint8_t)'0'));
            s_conf_len++;
        } else {
            s_state = ST_WAIT_HEAD;
            laser1_diag.parse_fail_cnt++;
        }
        break;

    default:
        s_state = ST_WAIT_HEAD;
        break;
    }
}

/* ---------- Public API ---------- */

uint8_t Laser_GetSuddenIncrease(const Laser_t *laser)
{
    if (laser == NULL) return 0U;
    return (laser->sudden_increase != 0U) ? 1U : 0U;
}

void Laser_ClearSuddenIncrease(Laser_t *laser)
{
    if (laser != NULL) laser->sudden_increase = 0U;
}

uint8_t Laser_GetSuddenDecrease(const Laser_t *laser)
{
    if (laser == NULL) return 0U;
    return (laser->sudden_decrease != 0U) ? 1U : 0U;
}

void Laser_ClearSuddenDecrease(Laser_t *laser)
{
    if (laser != NULL) laser->sudden_decrease = 0U;
}

/* ---------- ISR entry per byte ---------- */

void Laser_UART7_OnRxByte(uint8_t rx_byte)
{
    laser1_diag.last_rx_byte = rx_byte;
    laser1_diag.rx_byte_cnt++;
    sensor_uart7_on_rx_byte(rx_byte, &laser1);
}

/* ---------- Periodic sanity check (~2ms) ---------- */

void Laser_UART7_RxIrqSanityCheck(void)
{
    uint32_t guard;

    if (s_huart7 == NULL) return;

    laser1_diag.isr_last = READ_REG(s_huart7->Instance->ISR);

    if ((s_huart7->Instance->CR1 & USART_CR1_RXNEIE_RXFNEIE) == 0U) {
        laser1_diag.rxneie_restore_cnt++;

        guard = 50000U;
        while ((__HAL_UART_GET_FLAG(s_huart7, UART_FLAG_RXNE) != RESET) && (guard > 0U)) {
            (void)(s_huart7->Instance->RDR & 0xFFU);
            guard--;
        }

        __HAL_UART_CLEAR_FLAG(s_huart7,
            UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
        ATOMIC_SET_BIT(s_huart7->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
    }
}

/* ---------- Init ---------- */

void Laser_Init(UART_HandleTypeDef *huart7)
{
    uint32_t guard;

    s_huart7 = huart7;
    s_has_prev_dist = 0U;
    s_prev_dist = 0U;
    laser1.ready = 0U;
    s_state = ST_WAIT_HEAD;

    if (s_huart7 == NULL) return;

    /* Flush RX */
    guard = 50000U;
    while ((__HAL_UART_GET_FLAG(s_huart7, UART_FLAG_RXNE) != RESET) && (guard > 0U)) {
        (void)(s_huart7->Instance->RDR & 0xFFU);
        guard--;
    }

    /* Clear all error flags */
    __HAL_UART_CLEAR_FLAG(s_huart7,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    /* Ensure UE, RE, RXNEIE on; keep EIE off */
    ATOMIC_SET_BIT(s_huart7->Instance->CR1,
        USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE);
    ATOMIC_CLEAR_BIT(s_huart7->Instance->CR3, USART_CR3_EIE);
}