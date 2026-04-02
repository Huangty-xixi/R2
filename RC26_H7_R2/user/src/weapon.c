#include "weapon.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"

// 状态标志
uint8_t servo_state = 1;    // 舵机状态
uint8_t clamp_state = 0;     // 夹爪开合
uint8_t sucker1_state = 0;     // 吸盘1开合
uint8_t sucker2_state = 0;     // 吸盘2开合
uint8_t sucker3_state = 0;     // 吸盘3开合
uint8_t sucker4_state = 0;     // 吸盘4开合


// 消抖锁
uint8_t ch5_lock = 0;




/**
  * @brief 武器运行逻辑
  */
void manual_weapon_function(void)
{
	//空函数
}



/**
* @brief 舵机控制
  */
void servo_use(void)
{
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        servo_state ^= 1; // 反转
        ch5_lock = 1;
    }
    if (RCctrl.CH5 !=192)
    {
        ch5_lock = 0;
    }

    if (servo_state %2==0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1400); // 中间位置
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100); //直立位置
    }
}

/**
  * @brief 夹爪开合（PC10）
  */
void clamp_use(void)
{
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        clamp_state ^= 1; // 反转
        ch5_lock = 1;
    }
    if (RCctrl.CH5 !=192)
    {
        ch5_lock = 0;
    }

    if (clamp_state %2== 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    }
}

/**
  * @brief 吸盘1（PC11）
  */
void sucker1_use(void)
{
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        sucker1_state ^= 1; // 反转
        ch5_lock = 1;
    }
    if (RCctrl.CH5 !=192)
    {
        ch5_lock = 0;
    }

    if (sucker1_state %2== 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    }
}

/**
  * @brief 吸盘2（PC13）
  */
void sucker2_use(void)
{
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        sucker2_state ^= 1; // 反转
        ch5_lock = 1;
    }
    if (RCctrl.CH5 !=192)
    {
        ch5_lock = 0;
    }

    if (sucker2_state %2== 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

/**
  * @brief 吸盘3（PC14）
  */
void sucker3_use(void)
{
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        sucker3_state ^= 1; // 反转
        ch5_lock = 1;
    }
    if (RCctrl.CH5 !=192)
    {
        ch5_lock = 0;
    }

    if (sucker3_state %2== 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    }
}

/**
  * @brief 吸盘4（PC15）
  */
void sucker4_use(void)
{
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        sucker4_state ^= 1; // 反转
        ch5_lock = 1;
    }
    if (RCctrl.CH5 !=192)
    {
        ch5_lock = 0;
    }

    if (sucker4_state %2== 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

/**
* @brief 不复位，保持状态
  */
void weapon_reset_all(void)
{
  //空函数
}

