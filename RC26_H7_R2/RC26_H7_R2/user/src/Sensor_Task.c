#include "Sensor_Task.h"
#include "imu.h"
#include "main.h"

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

        {
            const rc_odom_t *p = rc_get_latest_odom();
            (void)memcpy((void *)&g_sensor_task_data.odom, (const void *)p, sizeof(rc_odom_t));
        }

        osDelay(2);
    }
}
