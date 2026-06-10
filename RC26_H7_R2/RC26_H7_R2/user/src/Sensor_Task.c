#include "Sensor_Task.h"
#include "imu.h"
#include "main.h"
#include "sensor.h"
#include "common.h"

#include <math.h>
#include <string.h>

volatile sensor_task_data_t g_sensor_task_data = {0};

#if !RC_USE_IMU_ATTITUDE
/** ODOM yaw 差分角速度一阶低通系数 */
#define SENSOR_ODOM_YAW_RATE_LPF_ALPHA  (0.3f)

static void sensor_update_gyr_z_from_odom_yaw(float yaw_deg, uint32_t now_ms)
{
    static float prev_yaw_deg = 0.0f;
    static uint32_t prev_ms = 0U;
    static uint8_t inited = 0U;
    const float dyaw = wrap_deg_180(yaw_deg - prev_yaw_deg);

    if (inited == 0U)
    {
        prev_yaw_deg = yaw_deg;
        prev_ms = now_ms;
        inited = 1U;
        g_sensor_task_data.imu.gyr_z_dps = 0.0f;
        return;
    }

    if (fabsf(dyaw) < 1e-4f)
    {
        g_sensor_task_data.imu.gyr_z_dps *= 0.95f;
        return;
    }

    {
        float dt_s = (float)(now_ms - prev_ms) * 0.001f;

        if (dt_s < 1e-3f)
        {
            return;
        }
        if (dt_s > 0.5f)
        {
            prev_yaw_deg = yaw_deg;
            prev_ms = now_ms;
            g_sensor_task_data.imu.gyr_z_dps = 0.0f;
            return;
        }

        {
            const float raw_rate = dyaw / dt_s;
            const float alpha = SENSOR_ODOM_YAW_RATE_LPF_ALPHA;
            const float prev_rate = g_sensor_task_data.imu.gyr_z_dps;

            g_sensor_task_data.imu.gyr_z_dps =
                alpha * raw_rate + (1.0f - alpha) * prev_rate;
        }
    }

    prev_yaw_deg = yaw_deg;
    prev_ms = now_ms;
}
#endif /* !RC_USE_IMU_ATTITUDE */

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
        sensor_update_gyr_z_from_odom_yaw(g_sensor_task_data.imu.yaw_deg, HAL_GetTick());
#endif

        osDelay(2);
    }
}
