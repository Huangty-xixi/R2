#include "weapon.h"
#include "remote_control.h"
#include "Motion_Task.h"
#include "clamp_head_ctrl.h"
#include "main.h"
#include "tim.h"
#include "chassis.h"



// 武器状态
uint8_t servo_state = 0;    // 舵机状态
uint8_t clamp_state = 0;     // 夹爪状态
uint8_t sucker1_state = 0;     // 吸盘1状态
uint8_t sucker2_state = 0;     // 吸盘2状态
uint8_t sucker3_state = 0;     // 吸盘3状态
uint8_t sucker4_state = 0;     // 吸盘4状态

// 锁状态
uint8_t ch5_lock = 0;

/* 舵机 PWM（weapon.c 内宏，下次 servo_use() 写入 TIM2 CH1） */
#define WEAPON_SERVO_PWM_MID      (1450U)
#define WEAPON_SERVO_PWM_UPRIGHT  (2100U)


/* master_weapon_action_bits 主武器动作位 */
#define MASTER_WEAPON_SERVO_BIT   (1U << 0)
#define MASTER_WEAPON_CLAMP_BIT   (1U << 1)
#define MASTER_WEAPON_SUCKER1_BIT (1U << 2)
#define MASTER_WEAPON_SUCKER2_BIT (1U << 3)
#define MASTER_WEAPON_SUCKER3_BIT (1U << 4)
#define MASTER_WEAPON_SUCKER4_BIT (1U << 5)

/**
 * @brief 吸盘组1+2 与泵联动（占位实现：若工程内未单独实现泵控，仅满足链接；GPIO 仍由 sucker*_use 驱动）
 * @param open1 吸盘1名义开(1)/关(0)
 * @param open2 吸盘2名义开(1)/关(0)
 */
void pump1_two_suckers_linkage_nominal_open(uint8_t open1, uint8_t open2)
{
    (void)open1;
    (void)open2;
}

/**
 * @brief 吸盘组3+4 与泵联动（占位实现，同上）
 */
void pump2_two_suckers_linkage_nominal_open(uint8_t open3, uint8_t open4)
{
    (void)open3;
    (void)open4;
}

static void weapon_master_drive_by_bits(uint8_t action_bits)
{
    /* bit0 舵机状态 */
    servo_state = ((action_bits & MASTER_WEAPON_SERVO_BIT) != 0U) ? 0U : 1U;

    /* bit1: 夹爪状态 */
    clamp_state = ((action_bits & MASTER_WEAPON_CLAMP_BIT) != 0U) ? 1U : 0U;

    servo_use();
    clamp_use();

    /* bit2~bit5: 吸盘1~4状态 */
    sucker1_state = ((action_bits & MASTER_WEAPON_SUCKER1_BIT) != 0U) ? 1U : 0U;
    sucker2_state = ((action_bits & MASTER_WEAPON_SUCKER2_BIT) != 0U) ? 1U : 0U;
    sucker3_state = ((action_bits & MASTER_WEAPON_SUCKER3_BIT) != 0U) ? 1U : 0U;
    sucker4_state = ((action_bits & MASTER_WEAPON_SUCKER4_BIT) != 0U) ? 1U : 0U;

    // 吸盘1和吸盘2联动
    pump1_two_suckers_linkage_nominal_open((uint8_t)(sucker1_state & 0x01U), (uint8_t)(sucker2_state & 0x01U));
    // 吸盘3和吸盘4联动
    pump2_two_suckers_linkage_nominal_open((uint8_t)(sucker3_state & 0x01U), (uint8_t)(sucker4_state & 0x01U));
}




/**
  * @brief 手动武器功能
  */
void manual_weapon_function(void)
{
    /* master 主武器控制（见 Motion_Task.h 手动武器功能） */
    // if (control_mode == master_control)
    // {
    //     weapon_master_drive_by_bits(master_weapon_action_bits);
    //     return;
    // }

    /* 远程控制 */
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
    else if (control_mode == full_auto_control)
    {
        if (app_flow_mode == app_flow_zone1)
        {
            /* 一区：夹头状态机在此单点推进，勿与下方 servo_use/clamp_use 双跑 */
            ClampHeadCtrl_Run();
        }
        else
        {
            servo_use();
            clamp_use();
        }
        sucker1_use();
        sucker2_use();
        sucker3_use();
        sucker4_use();
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
* @brief 舵机使用
  */
void servo_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            servo_state ^= 1; // 舵机状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }   
    }
    if ((servo_state % 2U) == 0U)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)WEAPON_SERVO_PWM_MID);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)WEAPON_SERVO_PWM_UPRIGHT);
    }
}

/**
  * @brief 夹爪使用
  */
void clamp_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            clamp_state ^= 1; // 夹爪状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }


    if ((clamp_state % 2U) == 0U)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    }
}

/**
  * @brief 吸盘1使用
  */
void sucker1_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            sucker1_state ^= 1; // 吸盘1状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, sucker1_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief 吸盘2使用
  */
void sucker2_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            sucker2_state ^= 1; // 吸盘2状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, sucker2_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

    /**
  * @brief 吸盘3使用
  */
void sucker3_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            sucker3_state ^= 1; // 吸盘3状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, sucker3_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief 吸盘4使用
  */
void sucker4_use(void)
{
    if (control_mode == remote_control)
    {
        if (RCctrl.CH5 ==192 && ch5_lock == 0)
        {
            sucker4_state ^= 1; // 吸盘4状态反转
            ch5_lock = 1;
        }
        if (RCctrl.CH5 !=192)
        {
            ch5_lock = 0;
        }
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, sucker4_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
