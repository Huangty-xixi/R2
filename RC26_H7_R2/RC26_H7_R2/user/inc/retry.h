#ifndef __RETRY_H__
#define __RETRY_H__

#include <stdint.h>

/** 重试模块可调参数 — Keil Watch 实时修改 */
typedef struct {
    volatile uint32_t count_window_ms;    /* CH5 计数窗口 (ms) */
    volatile uint32_t feedback_timeout_ms;/* 蜂鸣确认超时 (ms) */
    volatile uint32_t beep_on_ms;         /* 每次蜂鸣时长 (ms) */
    volatile uint32_t beep_gap_ms;        /* 蜂鸣间隔 (ms) */
    volatile uint32_t beep_cycle_ms;      /* 蜂鸣周期 (ms) */
} RetryTune;

extern volatile RetryTune g_retry_tune;

void Retry_Init(void);

/** CH5 low/high->mid edge: up=1 increment, up=0 decrement, only counted when CH7==min */
void Retry_OnCH5Pulse(uint8_t up);

/** CH7 level: 0=min 1=max 2=mid */
void Retry_OnCH7Level(uint8_t ch7_bit);

/** Every tick: state machine + buzzer rhythm */
void Retry_Service(uint32_t now_ms);

/** True while counting/feedback/executing — gate channel routing */
uint8_t Retry_IsActive(void);

/** Save R1 mission for zone2 retry */
void Retry_SaveMission(const void *data, uint8_t size);

#endif
