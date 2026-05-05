#include "weapon.h"
#include "remote_control.h"
#include "Motion_Task.h"
#include "main.h"
#include "tim.h"
#include "chassis.h"



// 状态标志
uint8_t servo_state = 1;    // 舵机状态
uint8_t clamp_state = 0;     // 夹爪开合（PC10）
uint8_t sucker1_state = 0;     // 吸盘1开合（PC11）
uint8_t sucker2_state = 0;     // 吸盘2开合（PC12）
uint8_t sucker3_state = 0;     // 吸盘3开合（PE14）
uint8_t sucker4_state = 0;     // 吸盘4开合（PE1）
// uint8_t pump1_state = 0;     // 泵1开合（PC12）
// uint8_t pump2_state = 0;     // 泵2开合（PE14）

// 消抖锁
uint8_t ch5_lock = 0;

/* master_weapon_action_bits（第二个字节）位定义 */
#define MASTER_WEAPON_SERVO_BIT   (1U << 0)
#define MASTER_WEAPON_CLAMP_BIT   (1U << 1)
#define MASTER_WEAPON_SUCKER1_BIT (1U << 2)
#define MASTER_WEAPON_SUCKER2_BIT (1U << 3)
#define MASTER_WEAPON_SUCKER3_BIT (1U << 4)
#define MASTER_WEAPON_SUCKER4_BIT (1U << 5)

static void weapon_master_drive_by_bits(uint8_t action_bits)
{
    /* bit0: 舵机，0->2100，1->1100 */
    servo_state = ((action_bits & MASTER_WEAPON_SERVO_BIT) != 0U) ? 1U : 0U;
    if (servo_state == 0U)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);
    }   
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100);
    }

    /* bit1: 夹爪，按你的要求 1->SET，0->RESET */
    clamp_state = ((action_bits & MASTER_WEAPON_CLAMP_BIT) != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, clamp_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* bit2~bit5: 吸盘1~4，状态进入现有联动函数 */
    sucker1_state = ((action_bits & MASTER_WEAPON_SUCKER1_BIT) != 0U) ? 1U : 0U;
    sucker2_state = ((action_bits & MASTER_WEAPON_SUCKER2_BIT) != 0U) ? 1U : 0U;
    sucker3_state = ((action_bits & MASTER_WEAPON_SUCKER3_BIT) != 0U) ? 1U : 0U;
    sucker4_state = ((action_bits & MASTER_WEAPON_SUCKER4_BIT) != 0U) ? 1U : 0U;

    // pump1_two_suckers_linkage_nominal_open((uint8_t)(sucker1_state & 0x01U), (uint8_t)(sucker2_state & 0x01U));
    // pump2_two_suckers_linkage_nominal_open((uint8_t)(sucker3_state & 0x01U), (uint8_t)(sucker4_state & 0x01U));
}




/**
  * @brief 武器运行逻辑
  */
void manual_weapon_function(void)
{
    /* master模式逻辑暂时注释保留（与 Motion_Task.h 一致） */
    // if (control_mode == master_control)
    // {
    //     weapon_master_drive_by_bits(master_weapon_action_bits);
    //     return;
    // }

    /* 遥控模式：保持原手动逻辑 */
    if(control_mode == remote_control)
    {
        if (RCctrl.CH3 >=1500)
        {
        servo_use();
        }
        if (RCctrl.CH3<=500)
        {
        clamp_use();
        }
        if (RCctrl.CH2>=1500)
        {
        sucker1_use();
        }
        if (RCctrl.CH1<=500)
        {
        sucker2_use();
        }
        if (RCctrl.CH1>=1500)
        {
        sucker3_use();
        }
        if (RCctrl.CH2<=500)
        {
        sucker4_use();
        }
    }
    else if(control_mode == semi_auto_control)
    {
        servo_use();
        clamp_use();
        sucker1_use();
        sucker2_use();
        sucker3_use();
        sucker4_use();
        /* 半自动模式下不读取武器手动通道，避免CH5流程触发串动舵机 */
    }
    else
    {
        servo_state = 0;
        clamp_state = 0;
        sucker1_state = 0;
        sucker2_state = 0;
        sucker3_state = 0;
        sucker4_state = 0;
    }

}



/**
* @brief 舵机控制
  */
void servo_use(void)
{
    if (control_mode == remote_control)
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
    }
    if (servo_state %2==0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,1100); // 中间位置1400
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000); //直立位置2100（稍偏）
    }
}

/**
  * @brief 夹爪开合（PC10）
  */
void clamp_use(void)
{
    if (control_mode == remote_control)
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
    if (control_mode == remote_control)
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
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, sucker1_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // if (sucker1_state == 1)
    // {
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    // }
    // else
    // {
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    // }
}

/**
  * @brief 吸盘2（PC12）
  */
void sucker2_use(void)
{
    if (control_mode == remote_control)
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
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, sucker2_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // if (sucker2_state == 1)
    // {
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    // }
    // else
    // {
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    // }
}

    /**
  * @brief 吸盘3（PE14）
  */
void sucker3_use(void)
{
    if (control_mode == remote_control)
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
    }
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, sucker3_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // if (sucker3_state == 1)
    // {
    //     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
    // }
    // else
    // {
    //     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
    // }
}

/**
  * @brief 吸盘4（PE1）
  */
void sucker4_use(void)
{
    if (control_mode == remote_control)
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
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, sucker4_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // if (sucker4_state == 1)
    // {
    //     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    // }
    // else
    // {
    //     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
    // }
}
