#include "weapon.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"

// 状态标志
uint8_t servo_state = 1;    // 舵机状态
uint8_t pump_state = 0;     // 气泵状态

// 消抖锁（全局唯一）
uint8_t ch5_lock = 0;

/**
  * @brief 舵机控制
  */
void steering_use(void)
{
    // 统一用 CH5=192 触发
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        servo_state ^= 1; // 翻转
        ch5_lock = 1;
    }
    if (RCctrl.CH5 !=192)
    {
        ch5_lock = 0;
    }

    if (servo_state %2==0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1400);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2250);
    }
}

/**
  * @brief 气泵控制
  */
void pump_use(void)
{
    // 统一用 CH5=192 触发
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        pump_state ^= 1; // 翻转
        ch5_lock = 1;
    }
    if (RCctrl.CH5 !=192)
    {
        ch5_lock = 0;
    }

    if (pump_state %2== 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    }
}

/**
  * @brief 完全不复位，保持所有状态
  */
void weapon_reset_all(void)
{
    // 空函数 → 任何模式切换都不复位舵机、不关闭气泵
}