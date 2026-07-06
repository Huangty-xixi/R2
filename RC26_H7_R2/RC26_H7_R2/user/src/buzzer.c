#include "buzzer.h"
#include "tim.h"
#include "cmsis_os.h"

static uint32_t s_beep_start_ms = 0U;
static uint32_t s_beep_duration_ms = 0U;
static uint8_t  s_beep_active = 0U;

void Buzzer_Init(void)
{
    s_beep_active = 0U;
    s_beep_start_ms = 0U;
    s_beep_duration_ms = 0U;
    Buzzer_Off();
}

void Buzzer_On(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 125U);
}

void Buzzer_Off(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
}

void Buzzer_Beep(uint32_t duration_ms)
{
    s_beep_start_ms = osKernelGetTickCount();
    s_beep_duration_ms = duration_ms;
    s_beep_active = 1U;
    Buzzer_On();
}

void Buzzer_Service(uint32_t now_ms)
{
    if (s_beep_active == 0U) return;
    if ((now_ms - s_beep_start_ms) >= s_beep_duration_ms)
    {
        s_beep_active = 0U;
        Buzzer_Off();
    }
}
