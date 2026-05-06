/**
 * @file    imu.h
 * @brief   HI14 (RS-485 Modbus) IMU driver + power-on init sequence
 *
 * 说明：
 * - 使用 USART2 (RS485) 与 HI14 通讯
 * - 支持上电初始化：切六轴(heading=0) + 复位 + 约2s静止 + PR置零
 * - Sensor_Task 周期调用：请求一帧 + 解析更新到 g_sensor_task_data.imu
 */
#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HI14 上电初始化（调度器启动前调用）
 * @return 0 成功；非0 表示第1步（写0x0006）未通过回显确认或通信异常
 */
int32_t IMU_HI14_PowerOnInit(void);

/** @brief 周期请求并启动接收（原 Sensor_Task 内部逻辑） */
void IMU_RequestAndStartRx(void);

/** @brief 若接收缓冲满足条件则解析并写入 g_sensor_task_data.imu */
void IMU_ParseFrameIfReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */

