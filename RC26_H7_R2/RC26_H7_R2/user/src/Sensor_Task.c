#include "Sensor_Task.h"
#include "imu.h"
#include "main.h"
#include "sensor.h"

#include <string.h>

volatile sensor_task_data_t g_sensor_task_data = {0};

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
        Laser_UART7_RxIrqSanityCheck();

        {
            const rc_odom_t *p = rc_get_latest_odom();
            (void)memcpy((void *)&g_sensor_task_data.odom, (const void *)p, sizeof(rc_odom_t));
        }

#if !RC_USE_IMU_ATTITUDE
        g_sensor_task_data.imu.roll_deg  = g_sensor_task_data.odom.roll;
        g_sensor_task_data.imu.pitch_deg = g_sensor_task_data.odom.pitch;
        g_sensor_task_data.imu.yaw_deg   = g_sensor_task_data.odom.yaw;
#endif

        osDelay(2);
    }
}
