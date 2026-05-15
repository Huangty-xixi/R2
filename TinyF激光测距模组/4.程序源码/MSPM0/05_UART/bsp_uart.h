#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "ti_msp_dl_config.h"
#include "ti/driverlib/m0p/dl_core.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// ==================== 硬件配置宏定义 ====================
#define DEBUG_UART         UART_0_INST        // 调试串口实例[3](@ref)
#define RADAR_UART         UART_1_INST        // 雷达数据串口
#define RX_BUF_SIZE        32                 // 接收缓冲区大小(防溢出)[1](@ref)

// ==================== 通信协议常量 ====================
#define FRAME_HEADER       0x20               // 帧头(空格)
#define FRAME_SEPARATOR    0x2C               // 分隔符(逗号)[3](@ref)
#define FRAME_TERMINATOR   0x0A               // 帧尾(换行符)
#define DISTANCE_MIN       20                 // 最小有效距离(mm)
#define DISTANCE_MAX       4000               // 最大有效距离(mm)
#define CONFIDENCE_MAX     62                 // 最大置信度



// 数据发送
void uart_send_char(char ch);                 // 单字节发送
void uart_send_string(const char* str);       // 字符串发送[1](@ref)

// 数据处理
void Radar_Data_Parser(uint8_t RXdata);       // 雷达数据解析uint8_t Verify_Checksum(const uint8_t* data); // CRC校验(可选)

// ==================== 外部变量声明 ====================
extern volatile uint8_t  rx_flag;            // 帧接收完成标志
extern uint8_t radar_rx_buffer[RX_BUF_SIZE];   // 接收缓冲区
extern volatile uint8_t  data_ready;        // 数据就绪标志
extern volatile uint16_t distance_value;           // 解析的距离值（单位mm）
extern volatile uint8_t  confidence_value;        // 解析的置信度（0-62）

// 函数声明
void UART_Radar_Init(uint32_t baudrate);
void Radar_Data_Parser(uint8_t RXdata);
void uart_send_string(const char* str);

#endif /* __BSP_USART_H */