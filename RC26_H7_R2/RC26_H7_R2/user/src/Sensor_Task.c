#include "Sensor_Task.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"

#include <string.h>

volatile sensor_task_data_t g_sensor_task_data = {0};

static void IMU_RequestAndStartRx(void)
{
    /* 0x50 0x03 ...：你在 Can_Task 里用的同一帧请求 */
    static const uint8_t req[8] = {0x50U, 0x03U, 0x00U, 0x34U, 0x00U, 0x18U, 0x09U, 0x8FU};

    BSP_USART2_DE(1U);
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)req, 8U, 30U);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {;}
    BSP_USART2_DE(0U);

    BSP_USART2_StartRxIT();
}

static void IMU_ParseFrameIfReady(void)
{
    if ((g_imu_rx_ready == 0U) || (g_imu_rx_size != 53U))
    {
        return;
    }

    if ((g_imu_rx_buf[0] != 0x50U) || (g_imu_rx_buf[1] != 0x03U) || (g_imu_rx_buf[2] != 0x30U))
    {
        g_imu_rx_ready = 0U;
        return;
    }

    {
        int16_t accx, accy, accz;
        int16_t gyrx, gyry, gyrz;
        int16_t magx, magy, magz;
        int32_t roll, pitch, yaw;
        sensor_imu_t imu;

        accx = (int16_t)(((uint16_t)g_imu_rx_buf[3] << 8) | g_imu_rx_buf[4]);
        accy = (int16_t)(((uint16_t)g_imu_rx_buf[5] << 8) | g_imu_rx_buf[6]);
        accz = (int16_t)(((uint16_t)g_imu_rx_buf[7] << 8) | g_imu_rx_buf[8]);

        gyrx = (int16_t)(((uint16_t)g_imu_rx_buf[9] << 8) | g_imu_rx_buf[10]);
        gyry = (int16_t)(((uint16_t)g_imu_rx_buf[11] << 8) | g_imu_rx_buf[12]);
        gyrz = (int16_t)(((uint16_t)g_imu_rx_buf[13] << 8) | g_imu_rx_buf[14]);

        magx = (int16_t)(((uint16_t)g_imu_rx_buf[15] << 8) | g_imu_rx_buf[16]);
        magy = (int16_t)(((uint16_t)g_imu_rx_buf[17] << 8) | g_imu_rx_buf[18]);
        magz = (int16_t)(((uint16_t)g_imu_rx_buf[19] << 8) | g_imu_rx_buf[20]);

        roll  = (int32_t)(((uint32_t)g_imu_rx_buf[21] << 24) | ((uint32_t)g_imu_rx_buf[22] << 16) | ((uint32_t)g_imu_rx_buf[23] << 8) | (uint32_t)g_imu_rx_buf[24]);
        pitch = (int32_t)(((uint32_t)g_imu_rx_buf[25] << 24) | ((uint32_t)g_imu_rx_buf[26] << 16) | ((uint32_t)g_imu_rx_buf[27] << 8) | (uint32_t)g_imu_rx_buf[28]);
        yaw   = (int32_t)(((uint32_t)g_imu_rx_buf[29] << 24) | ((uint32_t)g_imu_rx_buf[30] << 16) | ((uint32_t)g_imu_rx_buf[31] << 8) | (uint32_t)g_imu_rx_buf[32]);

        imu.acc_x_g = (float)accx * 0.00048828f;
        imu.acc_y_g = (float)accy * 0.00048828f;
        imu.acc_z_g = (float)accz * 0.00048828f;

        imu.gyr_x_dps = (float)gyrx * 0.061035f;
        imu.gyr_y_dps = (float)gyry * 0.061035f;
        imu.gyr_z_dps = (float)gyrz * 0.061035f;

        imu.mag_x_ut = (float)magx * 0.030517f;
        imu.mag_y_ut = (float)magy * 0.030517f;
        imu.mag_z_ut = (float)magz * 0.030517f;

        imu.roll_deg = (float)roll * 0.001f;
        imu.pitch_deg = (float)pitch * 0.001f;
        imu.yaw_deg = (float)yaw * 0.001f;

        g_sensor_task_data.imu = imu;
    }

    g_imu_rx_ready = 0U;
}

void Sensor_Task(void *argument)
{
    (void)argument;

    uint32_t imu_last_tick = 0U;

    for (;;)
    {
        if ((HAL_GetTick() - imu_last_tick) >= 200U)
        {
            imu_last_tick = HAL_GetTick();
            IMU_RequestAndStartRx();
        }

        IMU_ParseFrameIfReady();

        {
            const rc_odom_t *p = rc_get_latest_odom();
            (void)memcpy((void *)&g_sensor_task_data.odom, (const void *)p, sizeof(rc_odom_t));
        }

        osDelay(2);
    }
}
