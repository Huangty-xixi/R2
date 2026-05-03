#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

#include "cmsis_os.h"
#include "upper_pc_protocol.h"
#include <stdint.h>

/** IMU 物理量（Sensor_Task 解析 Modbus 帧后写入） */
typedef struct {
    float acc_x_g;     /* g */
    float acc_y_g;
    float acc_z_g;
    float gyr_x_dps;   /* deg/s */
    float gyr_y_dps;
    float gyr_z_dps;
    float mag_x_ut;    /* uT */
    float mag_y_ut;
    float mag_z_ut;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} sensor_imu_t;

/**
 * 传感器任务对外快照：IMU + 上位机里程计（里程计由 RcOdomSnap_Read 刷新）
 */
typedef struct {
    sensor_imu_t imu;
    rc_odom_t    odom;
} sensor_task_data_t;

extern volatile sensor_task_data_t g_sensor_task_data;

void Sensor_Task(void *argument);

#endif
