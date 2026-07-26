#ifndef __BUZZER_H__
#define __BUZZER_H__

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

/** 响指定毫秒后自动关（非阻塞，需 Buzzer_Service 每 tick 驱动） */
void Buzzer_Beep(uint32_t duration_ms);

/** 每 tick 调用，处理自动关 */
void Buzzer_Service(uint32_t now_ms);

#endif
