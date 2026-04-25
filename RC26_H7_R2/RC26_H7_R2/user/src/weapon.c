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

uint16_t switch_state;//光电开关（PE9）
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
    /* master模式：按数据包位控直接驱动机构 */
    if (control_mode == master_control)
    {
        weapon_master_drive_by_bits(master_weapon_action_bits);
        return;
    }

    /* 遥控模式：保持原手动逻辑 */
    else if(control_mode == remote_control)
    {
        if (RCctrl.CH3==1792)
        {
        servo_use();
        }
        if (RCctrl.CH3==192)
        {
        clamp_use();
        }
        if (RCctrl.CH2==1792)
        {
        sucker1_use();
        }
        if (RCctrl.CH1==192)
        {
        sucker2_use();
        }
        if (RCctrl.CH1==1792)
        {
        sucker3_use();
        }
        if (RCctrl.CH2==192)
        {
        sucker4_use();
        }
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
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,1100); // 中间位置1400
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100); //直立位置2100
    }
}

/**
  * @brief 夹爪开合（PC10）
  */
void clamp_use(void)
{
	switch_state=HAL_GPIO_ReadPin(GPIOE ,GPIO_PIN_9); 
	if(switch_state ==1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	}
	else 
	{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	}
//    if (RCctrl.CH5 ==192 && ch5_lock == 0)
//    {
//        clamp_state ^= 1; // 反转
//        ch5_lock = 1;
//    }
//    if (RCctrl.CH5 !=192)
//    {
//        ch5_lock = 0;
//    }

//    if (clamp_state %2== 0)
//    {
//        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
//    }
//    else
//    {
//        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
//    }
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

    if (sucker1_state == 1)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    }
}

/**
  * @brief 吸盘2（PC12）
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

    if (sucker2_state == 1)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    }
}

    /**
  * @brief 吸盘3（PE14）
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

    if (sucker3_state == 1)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
    }
}

/**
  * @brief 吸盘4（PE1）
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

    if (sucker4_state == 1)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
    }
}




/**
* @brief 不复位，保持状态
  */
void weapon_reset_all(void)
{
  //空函数
}


///**
//  * @brief 两个吸盘共用一个气泵的联动控制
//  * @param sucker1_on 吸盘1目标状态：1-打开，0-关闭
//  * @param sucker2_on 吸盘2目标状态：1-打开，0-关闭
//  * @note  任一吸盘打开则开启气泵；仅当两个吸盘都关闭时关闭气泵
//  */
//void pump1_two_suckers_linkage(uint8_t sucker1_on, uint8_t sucker2_on)
//{
//    /* 归一化输入，避免非0值造成歧义 */
//    sucker1_on = (sucker1_on != 0U) ? 1U : 0U;
//    sucker2_on = (sucker2_on != 0U) ? 1U : 0U;

//    /* 同步状态变量，保持与现有代码风格一致（1为打开，0为关闭） */
//    sucker1_state = sucker1_on;
//    sucker2_state = sucker2_on;

//    /* 吸盘/气泵为高电平有效：打开->SET，关闭->RESET */
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, sucker1_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, sucker2_on ? GPIO_PIN_RESET : GPIO_PIN_SET);

//    /* 联动逻辑：任一吸盘打开则气泵打开 */
//    pump1_state = (uint8_t)((sucker1_on || sucker2_on) ? 1U : 0U);
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, pump1_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
//}
///**
//  * @brief 两个吸盘共用一个气泵的联动控制
//  * @param sucker3_on 吸盘3目标状态：1-打开，0-关闭
//  * @param sucker4_on 吸盘4目标状态：1-打开，0-关闭
//  * @note  任一吸盘打开则开启气泵；仅当两个吸盘都关闭时关闭气泵
//  */
//static void pump2_two_suckers_linkage(uint8_t sucker3_on, uint8_t sucker4_on)
//{
//    sucker3_on = (sucker3_on != 0U) ? 1U : 0U;
//    sucker4_on = (sucker4_on != 0U) ? 1U : 0U;

//    sucker3_state = sucker3_on;
//    sucker4_state = sucker4_on;

//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, sucker3_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, sucker4_on ? GPIO_PIN_RESET : GPIO_PIN_SET);


//    pump2_state = (uint8_t)((sucker3_on || sucker4_on) ? 1U : 0U);
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, pump2_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
//}  

// /**
//   * @brief 泵1+吸盘1/2：电磁阀默认常开(低电平)，仅单路工作时给另一路高电平关阀
//   * @note  与 pump1_two_suckers_linkage 并存，需用时在 sucker1_use/sucker2_use 中改调本函数
//   */
// void pump1_two_suckers_linkage_nominal_open(uint8_t sucker1_on, uint8_t sucker2_on)
// {
//     sucker1_on = (sucker1_on != 0U) ? 1U : 0U;
//     sucker2_on = (sucker2_on != 0U) ? 1U : 0U;

//     sucker1_state = sucker1_on;
//     sucker2_state = sucker2_on;

//     /* 电磁阀：低电平=开(不通电)，高电平=关(通电) */
//     if ((sucker1_on == 0U) && (sucker2_on == 0U))
//     {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
//         pump1_state = 0U;
//     }
//     else if ((sucker1_on == 1U) && (sucker2_on == 0U))
//     {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
//         pump1_state = 1U;
//     }
//     else if ((sucker1_on == 0U) && (sucker2_on == 1U))
//     {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
//         pump1_state = 1U;
//     }
//     else
//     {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
//         pump1_state = 1U;
//     }

//     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, pump1_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
// }

// /**
//   * @brief 泵2+吸盘3/4：同上
//   */
// void pump2_two_suckers_linkage_nominal_open(uint8_t sucker3_on, uint8_t sucker4_on)
// {
//     sucker3_on = (sucker3_on != 0U) ? 1U : 0U;
//     sucker4_on = (sucker4_on != 0U) ? 1U : 0U;

//     sucker3_state = sucker3_on;
//     sucker4_state = sucker4_on;

//     if ((sucker3_on == 0U) && (sucker4_on == 0U))
//     {
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//         pump2_state = 0U;
//     }
//     else if ((sucker3_on == 1U) && (sucker4_on == 0U))
//     {
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//         pump2_state = 1U;
//     }
//     else if ((sucker3_on == 0U) && (sucker4_on == 1U))
//     {
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//         pump2_state = 1U;
//     }
//     else
//     {
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//         pump2_state = 1U;
//     }

//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, pump2_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
// }

