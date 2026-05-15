#include "bsp_uart.h"
#define DISTANCE_MIN     20      // 最小有效距离 (mm) / Minimum valid distance (mm)
#define DISTANCE_MAX     4000    // 最大有效距离 (mm) / Maximum valid distance (mm)
#define CONFIDENCE_MAX   62      // 最大置信度 / Maximum confidence value
        
volatile uint16_t distance_value = 0;       // 解析得到的距离值(20-4000)单位mm / Parsed distance value (20-4000) in mm
volatile uint8_t confidence_value = 0;      // 解析得到的置信度（0-62） / Parsed confidence value (0-62)
volatile uint8_t data_ready = 0;            // 接收完成标志 / Data ready flag
volatile uint8_t uart_data = 0;

int fputc(int ch, FILE *f) {
    while (DL_UART_isBusy(DEBUG_UART));  // 等待串口空闲
    DL_UART_Main_transmitData(DEBUG_UART, (uint8_t)ch);  // 发送字符
    return ch;
}
void uart_send_string(const char* str) {
    while (*str != '\0') {
        while (DL_UART_isBusy(DEBUG_UART));  // 等待空闲
        DL_UART_Main_transmitData(DEBUG_UART, *str++);
    }
}

void Processing_Data(uint8_t RXdata) {
    static uint8_t recv_buf[16] = {0}; // 接收缓冲区（最大11字节数据） / Receive buffer (max 11 bytes data)
    static uint8_t index = 0;           // 当前缓冲区位置 / Current buffer position
    static uint8_t parsing = 0;         // 解析状态 / Parsing state
    static uint8_t comma_pos = 0;       // 逗号位置（用于截取距离数据） / Comma position (for extracting distance data)

    // 防溢出：如果缓冲区满则重置状态机 / Overflow protection: reset state machine if buffer full
    if (index >= sizeof(recv_buf)) {
        index = 0;        
        parsing = 0;
        comma_pos = 0;  // 重置逗号位置 / Reset comma position
        return;
    }

    // 存储接收到的字节 / Store received byte
    recv_buf[index++] = RXdata;

    // 状态机解析 / State machine parsing
    switch (parsing) {
        case 0:  // 等待帧头0x20 (空格)  Wait for header 0x20 (space)
            if (RXdata == 0x20) {
                parsing = 1;  // 进入距离解析状态  Enter distance parsing state
                index = 1;     
            } else {
                index = 0;     // 非帧头字符，重置  Non-header character, reset
            }
            break;

        case 1:  // 解析距离值 / Parse distance value
            if (RXdata == 0x2C) {  // 遇到逗号  Encounter comma
                parsing = 2;       // 进入分隔符检查状态  Enter separator check state
                comma_pos = index - 1; // 记录逗号位置  Record comma position
            }
            break;

        case 2:  // 检查分隔符0x20 (空格)  Check separator 0x20 (space)
            if (RXdata == 0x20) {
                parsing = 3; // 进入置信度解析状态  Enter confidence parsing state
            } else {
                // 格式错误，重置状态机  Format error, reset state machine
                parsing = 0;
                index = 0;
                comma_pos = 0;  // 重置逗号位置 / Reset comma position
            }
            break;

        case 3:  // 解析置信度 / Parse confidence value
            if (RXdata == 0x0A) { 
            
                uint8_t dist_len = comma_pos - 1;  // 距离值长度  Distance value length
                if (dist_len > 5) dist_len = 5;   // 防止溢出  Prevent overflow
                char dist_str[6] = {0};            
                memcpy(dist_str, &recv_buf[1], dist_len); 
                dist_str[dist_len] = '\0';         // 添加结束符  Add null terminator

                // 提取置信度字符串 / Extract confidence string
                uint8_t conf_start = comma_pos + 2;  
                uint8_t conf_len = index - conf_start - 1;  
                if (conf_len > 2) conf_len = 2;      // 置信度最多2字符  Confidence max 2 characters
                char conf_str[3] = {0};              
                memcpy(conf_str, &recv_buf[conf_start], conf_len);
                conf_str[conf_len] = '\0';            // 同上

                // 转换为数值 / Convert to numerical values
                distance_value = atoi(dist_str);      // 距离值转换  Distance conversion
                confidence_value = atoi(conf_str);    // 置信度转换  Confidence conversion

                // 检查数据有效性  Check data validity
                if (distance_value < DISTANCE_MIN || 
                    distance_value > DISTANCE_MAX || 
                    confidence_value > CONFIDENCE_MAX) 
                {
                    // 无效数据，清零  Invalid data, clear values
                    distance_value = 0;
                    confidence_value = 0;
                }
                
                // 设置数据就绪标志  Set data ready flag
                data_ready = 1;

                // 重置接收状态机  Reset state machine
                index = 0;
                parsing = 0;
                comma_pos = 0;  
            }
            break;

        default:
            // 未知状态，重置状态机  Unknown state, reset state machine
            parsing = 0;
            index = 0;
            comma_pos = 0;
            break;
    }
}
// UART1中断服务函数（接收数据）

void UART_1_INST_IRQHandler(void)
{
    //如果产生了串口中断 If a serial port interrupt occurs
    switch( DL_UART_getPendingInterrupt(UART_1_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断 If it is a receive interrupt
            //接发送过来的数据保存在变量中 The data sent is saved in the variable
            uart_data = DL_UART_Main_receiveData(UART_1_INST);
            Processing_Data(uart_data);
//					uart_send_string(&uart_data);
            break;

        default://其他的串口中断 Other serial port interrupts
            break;
    }
}
