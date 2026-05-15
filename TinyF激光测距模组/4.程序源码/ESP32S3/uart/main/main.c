#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "ring_buffer.h"
#include "uart0.h"
#include "uart1.h"


void parse_distance_data(char *line);

void app_main() {
   // 初始化UART  Init UART
    Uart0_Init();
    Uart1_Init();
    uint8_t data;
    char line_buf[16]; // 用于存储一行数据  Line buffer for storing one line of data
    int line_index = 0; // 当前行索引  Current line index
    while (1) {
        int len = 0;
     //   尝试读取一个字节  Try to read one byte
        len = uart_read_bytes(UART_NUM_1, &data, 1,8/115200);
        
        if (len > 0) {
        //    如果遇到换行符，处理数据  If a newline character is encountered, process data
            if (data == '\n') {
                if (line_index > 0) {
                    line_buf[line_index] = '\0'; // 添加字符串结束符  Add a string end character
                    // 解析数据  Parse data
                    parse_distance_data(line_buf);
                    
                    // 重置行索引  Reset line index
                    line_index = 0;
                }
            } 
            // 添加到行缓冲区  Add to line buffer
            else if (line_index < 15) {
                line_buf[line_index++] = data;
            }
            // 缓冲区溢出处理  Buffer overflow handling
            else {
                line_index = 0; // 重置缓冲区  Reset buffer
            }
        } 
        // 短暂延迟降低CPU使用率（非阻塞）  Short delay to reduce CPU usage (non-blocking)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}



// 解析距离数据
void parse_distance_data(char *line) {
    // 去除开始空白字符  Remove leading whitespace characters
    char *p = line;
    while (*p && (*p == ' ' )) p++;
    char *end = p + strlen(p) - 1;
    while (end > p && *end == '\n') end--;
    *(end+1) = '\0';

    // 查找逗号位置  Find comma position
    char *comma = strchr(p, ',');
    if (comma == NULL) {

        return;
    }

    // 提取距离部分  Extract distance part
    *comma = '\0'; // 分割字符串
    char *distance_str = p;
    char *confidence_str = comma + 1;
    
    // 去除置信度字符串前导空格  Remove leading whitespace characters from confidence string
    while (*confidence_str && (*confidence_str == ' ' )) confidence_str++;

    // 验证数据有效性  Validate data validity
    if (strlen(distance_str) == 0 || strlen(confidence_str) == 0) {

        return;
    }

    // 转换为整数  Convert to integer
    uint32_t distance = atoi(distance_str);
    uint32_t confidence = atoi(confidence_str);
   

    // 输出结果  Output results
        char result_buf[100];
        int len = snprintf(result_buf, sizeof(result_buf),
                           "Distance: %ldmm,Confidence: %ld\r\n",
                           distance, confidence);
        uart_write_bytes(UART_NUM_0, result_buf, len);

        // 短暂延迟降低CPU使用率（非阻塞）  Short delay to reduce CPU usage (non-blocking)
        vTaskDelay(pdMS_TO_TICKS(1));
}
