#ifndef __RETRY_H__
#define __RETRY_H__

#include <stdint.h>

void Retry_Init(void);

/** CH5 low->mid edge, only counted when CH7==min */
void Retry_OnCH5Edge(void);

/** CH7 level: 0=min 1=max 2=mid */
void Retry_OnCH7Level(uint8_t ch7_bit);

/** Every tick: state machine + buzzer rhythm */
void Retry_Service(uint32_t now_ms);

/** True while counting/feedback/executing ¡ª gate channel routing */
uint8_t Retry_IsActive(void);

/** Save R1 mission for zone2 retry */
void Retry_SaveMission(const void *data, uint8_t size);

#endif
