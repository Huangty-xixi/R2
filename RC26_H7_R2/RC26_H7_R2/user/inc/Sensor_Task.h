#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

#include "cmsis_os.h"
#include <stdint.h>

/* IMU data (updated by Sensor_Task) */
extern volatile float g_imu_acc_x_g;
extern volatile float g_imu_acc_y_g;
extern volatile float g_imu_acc_z_g;
extern volatile float g_imu_gyr_x_dps;
extern volatile float g_imu_gyr_y_dps;
extern volatile float g_imu_gyr_z_dps;
extern volatile float g_imu_mag_x_ut;
extern volatile float g_imu_mag_y_ut;
extern volatile float g_imu_mag_z_ut;
extern volatile float g_imu_roll_deg;
extern volatile float g_imu_pitch_deg;
extern volatile float g_imu_yaw_deg;

void Sensor_Task(void *argument);

#endif
