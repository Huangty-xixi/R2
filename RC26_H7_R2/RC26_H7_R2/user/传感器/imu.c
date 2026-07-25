/**
 * @file    imu.c
 * @brief   HI14 (RS-485 Modbus) IMU driver + power-on init sequence
 */
#include "imu.h"
#include "Sensor_Task.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"

#include <string.h>

/* ---------------- Modbus CRC16 (RTU) ---------------- */
static uint16_t crc16_modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    if (buf == NULL) return crc;

    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)buf[i];
        for (uint8_t b = 0U; b < 8U; b++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

/* ---------------- RS-485 send helpers (USART2) ---------------- */
static void imu_rs485_send_bytes(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) return;

    BSP_USART2_DE(1U);
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 50U);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {;}
    BSP_USART2_DE(0U);
}

static uint8_t imu_rs485_send_and_wait_echo8(const uint8_t tx8[8], uint32_t timeout_ms)
{
    uint8_t rx8[8] = {0U};

    if (tx8 == NULL) return 0U;

    (void)HAL_UART_AbortReceive(&huart2);
    imu_rs485_send_bytes(tx8, 8U);

    /* 写单寄存器正常响应为 8 字节回显：Addr Func RegHi RegLo ValHi ValLo CRCL CRCH */
    if (HAL_UART_Receive(&huart2, rx8, 8U, (uint32_t)timeout_ms) != HAL_OK)
    {
        return 0U;
    }

    if (memcmp((const void *)rx8, (const void *)tx8, 8U) != 0)
    {
        return 0U;
    }
    return 1U;
}

/* ---------------- HI14 power-on init sequence ---------------- */
int32_t IMU_HI14_PowerOnInit(void)
{
    /* Step1: Write HEADING_MODE (0x0006) = 0x0000, CRC calculated */
    uint8_t frm1[8] = {0x50U, 0x06U, 0x00U, 0x06U, 0x00U, 0x00U, 0x00U, 0x00U};
    const uint16_t crc1 = crc16_modbus(frm1, 6U);
    frm1[6] = (uint8_t)(crc1 & 0xFFU);         /* CRC_L */
    frm1[7] = (uint8_t)((crc1 >> 8) & 0xFFU);  /* CRC_H */

    /* 至少对第 1 步做确认：等待 8 字节回显 */
    if (imu_rs485_send_and_wait_echo8(frm1, 80U) == 0U)
    {
        return -1;
    }

    /* Step2: CTRL reset (existing frame) */
    {
        static const uint8_t frm2[8] = {0x50U, 0x06U, 0x00U, 0x00U, 0x00U, 0xFFU, 0xC4U, 0x0BU};
        imu_rs485_send_bytes(frm2, 8U);
    }

    /* Step3: wait module reboot & keep still about 2s */
    HAL_Delay(1500U);

    /* Step4: Pitch/Roll reset (existing frame) */
    {
        static const uint8_t frm3[8] = {0x50U, 0x06U, 0x00U, 0xA5U, 0x00U, 0x02U, 0x15U, 0xA9U};
        imu_rs485_send_bytes(frm3, 8U);
    }

    return 0;
}

/* ---------------- periodic request + parse (moved from Sensor_Task.c) ---------------- */
void IMU_RequestAndStartRx(void)
{
    /* 0x50 0x03 ...：工程内原逻辑请求 */
    static const uint8_t req[8] = {0x50U, 0x03U, 0x00U, 0x34U, 0x00U, 0x18U, 0x09U, 0x8FU};

    imu_rs485_send_bytes(req, 8U);
    BSP_USART2_StartRxIT();
}

void IMU_ParseFrameIfReady(void)
{
    if ((g_imu_uart_ctx.rx_ready == 0U) || (g_imu_uart_ctx.rx_size != 53U))
    {
        return;
    }

    if ((g_imu_uart_ctx.rx_buf[0] != 0x50U) || (g_imu_uart_ctx.rx_buf[1] != 0x03U) || (g_imu_uart_ctx.rx_buf[2] != 0x30U))
    {
        g_imu_uart_ctx.rx_ready = 0U;
        return;
    }

    {
        int16_t accx, accy, accz;
        int16_t gyrx, gyry, gyrz;
        int16_t magx, magy, magz;
        int32_t roll, pitch, yaw;
        sensor_imu_t imu;
        const uint8_t *rx = (const uint8_t *)g_imu_uart_ctx.rx_buf;

        accx = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
        accy = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
        accz = (int16_t)(((uint16_t)rx[7] << 8) | rx[8]);

        gyrx = (int16_t)(((uint16_t)rx[9] << 8) | rx[10]);
        gyry = (int16_t)(((uint16_t)rx[11] << 8) | rx[12]);
        gyrz = (int16_t)(((uint16_t)rx[13] << 8) | rx[14]);

        magx = (int16_t)(((uint16_t)rx[15] << 8) | rx[16]);
        magy = (int16_t)(((uint16_t)rx[17] << 8) | rx[18]);
        magz = (int16_t)(((uint16_t)rx[19] << 8) | rx[20]);

        roll  = (int32_t)(((uint32_t)rx[21] << 24) | ((uint32_t)rx[22] << 16) | ((uint32_t)rx[23] << 8) | (uint32_t)rx[24]);
        pitch = (int32_t)(((uint32_t)rx[25] << 24) | ((uint32_t)rx[26] << 16) | ((uint32_t)rx[27] << 8) | (uint32_t)rx[28]);
        yaw   = (int32_t)(((uint32_t)rx[29] << 24) | ((uint32_t)rx[30] << 16) | ((uint32_t)rx[31] << 8) | (uint32_t)rx[32]);

        imu.acc_x_g = (float)accx * 0.00048828f;
        imu.acc_y_g = (float)accy * 0.00048828f;
        imu.acc_z_g = (float)accz * 0.00048828f;

        imu.gyr_x_dps = (float)gyrx * 0.061035f;
        imu.gyr_y_dps = (float)gyry * 0.061035f;
        imu.gyr_z_dps = (float)gyrz * 0.061035f;

        imu.mag_x_ut = (float)magx * 0.030517f;
        imu.mag_y_ut = (float)magy * 0.030517f;
        imu.mag_z_ut = (float)magz * 0.030517f;

        imu.roll_deg  = (float)roll * 0.001f;
        imu.pitch_deg = (float)pitch * 0.001f;
        imu.yaw_deg   = (float)yaw * 0.001f;

        g_sensor_task_data.imu = imu;
    }

    g_imu_uart_ctx.rx_ready = 0U;
}

