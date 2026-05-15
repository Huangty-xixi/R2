#include "bsp_usart.h"

#define DISTANCE_MIN     20      // 最小有效距离 (mm) / Minimum valid distance (mm)
#define DISTANCE_MAX     4000    // 最大有效距离 (mm) / Maximum valid distance (mm)
#define CONFIDENCE_MAX   62      // 最大置信度 / Maximum confidence value
        
volatile uint16_t distance_value = 0;       // 解析得到的距离值(20-4000)单位mm / Parsed distance value (20-4000) in mm
volatile uint8_t confidence_value = 0;      // 解析得到的置信度（0-62） / Parsed confidence value (0-62)
volatile uint8_t data_ready = 0;            // 接收完成标志 / Data ready flag

// USART1初始化 - 与PC通信，输出调试信息
// USART1 initialization - communicates with PC for debugging output
void USART1_init(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
    GPIO_Init(GPIOA, &GPIO_InitStructure);    

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
      
    USART_InitStructure.USART_BaudRate = baudrate;        
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
    USART_InitStructure.USART_StopBits = USART_StopBits_1;     
    USART_InitStructure.USART_Parity = USART_Parity_No;         
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure); 
    
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);   
    
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_Cmd(USART1, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                       // USART1中断通道 / USART1 interrupt channel
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;               // 抢占优先级 / Preemption priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;                // 子优先级 / Subpriority
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);
}

// 也可以只使用串口1来接收，但是串口1还具有烧录调试的作用，使用串口2避免冲突
// Alternatively, only USART1 could be used for reception, but USART1 also serves for programming and debugging. 
// Using USART2 avoids conflicts.
void USART2_init(u32 baudrate) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级 / Preemption priority
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;        // 子优先级 / Subpriority
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;            // 使能中断通道 / Enable interrupt channel
    NVIC_Init(&NVIC_InitStruct);
}


void USART1_Send_U8(uint8_t ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    
			USART_SendData(USART1, ch);
}


void USART1_Send_ArrayU8(uint8_t *BufferPtr)
{
    while (*BufferPtr)
    {
        USART1_Send_U8(*BufferPtr++);
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

// 串口中断服务函数  USART interrupt service routine
void USART2_IRQHandler(void)
{
    uint8_t Rx1_Temp = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        // 读取接收数据寄存器  Read receive data register
        Rx1_Temp = USART_ReceiveData(USART2);
        
        // 可选：回显接收到的数据（调试用）  Optional: echo received data (for debugging)
        // USART2_Send_U8(Rx1_Temp);
        
        // 处理数据  Process received data
        Processing_Data(Rx1_Temp);

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

// 重定向printf到串口 / Redirect printf to USART
int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return (ch);
}

