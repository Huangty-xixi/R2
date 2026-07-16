#include "weapon.h"
#include "remote_control.h"
#include "Motion_Task.h"
#include "clamp_head_ctrl.h"
#include "main.h"
#include "tim.h"
#include "chassis.h"
#include "dji_motor.h"
#include <stdlib.h>



volatile weapon_tune_t g_weapon_tune = {
    .servo = {
        .pwm_mid = 1365U,
        .pwm_upright = 2230U,
    },
};


// 舵机状态
uint8_t servo_state = 1;    // 舵机状态
uint8_t clamp_state = 0;     // 夹爪状态
uint8_t sucker1_state = 0;     // 吸盘1状态
uint8_t sucker2_state = 0;     // 吸盘2状态
uint8_t sucker3_state = 0;     // 吸盘3状态..
uint8_t sucker4_state = 0;     // 吸盘4状态

// 舵机锁定
uint8_t ch5_lock = 0;

/* master_weapon_action_bits */
#define MASTER_WEAPON_SERVO_BIT   (1U << 0)
#define MASTER_WEAPON_CLAMP_BIT   (1U << 1)
#define MASTER_WEAPON_SUCKER1_BIT (1U << 2)
#define MASTER_WEAPON_SUCKER2_BIT (1U << 3)
#define MASTER_WEAPON_SUCKER3_BIT (1U << 4)
#define MASTER_WEAPON_SUCKER4_BIT (1U << 5)

/**
 * @brief 吸盘1+2 打开，吸盘3+4 关闭，GPIO 使用 sucker*_use 设置
 * @param open1 吸盘1打开(1)/关闭(0)
 * @param open2 吸盘2打开(1)/关闭(0)
 */
void pump1_two_suckers_linkage_nominal_open(uint8_t open1, uint8_t open2)
{
    (void)open1;
    (void)open2;
}

/**
 * @brief 吸盘3+4 打开，吸盘1+2 关闭，GPIO 使用 sucker*_use 设置
 * @param open3 吸盘3打开(1)/关闭(0)
 * @param open4 吸盘4打开(1)/关闭(0)
 */
void pump2_two_suckers_linkage_nominal_open(uint8_t open3, uint8_t open4)
{
    (void)open3;
    (void)open4;
}

void weapon_init(void)
{
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
#if APP_ZONE1_DBG_CLAMP_HEAD_ONLY
        ClampHeadCtrl_Run();
#else
        servo_use();
#endif
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
            servo_use();
        }
        else
        {
            ClampHeadCtrl_SetAutoGrabEnable(0U);
            servo_use();
        }
        sucker1_use();
        sucker2_use();
        sucker3_use();
        sucker4_use();
    }
    else
    {
        ClampHeadCtrl_SetAutoGrabEnable(0U);
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
        static uint8_t ch5_zone_prev = 1;  // 0=上, 1=中, 2=下
        uint8_t ch5_zone;

        if (RCctrl.CH5 <= 500)
            ch5_zone = 0;       // 上拨 → 直立
        else if (RCctrl.CH5 >= 1500)
            ch5_zone = 2;       // 下拨 → 水平
        else
            ch5_zone = 1;       // 中位 → 保持

        if (ch5_zone != ch5_zone_prev)
        {
            if (ch5_zone == 0)
                servo_state = 1;   // 直立
            else if (ch5_zone == 2)
                servo_state = 0;   // 水平
            ch5_zone_prev = ch5_zone;
        }
    }
    if ((servo_state % 2U) == 0U)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)g_weapon_tune.servo.pwm_mid);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)g_weapon_tune.servo.pwm_upright);
    }
}

/**
  * @brief 夹爪使用
  */
void clamp_use(void)
{
    if (control_mode == remote_control)
    {
        static uint8_t ch5_zone_prev = 1;  // 0=上, 1=中, 2=下
        uint8_t ch5_zone;

        if (RCctrl.CH5 <= 500)
            ch5_zone = 0;       // 上拨 → 张开
        else if (RCctrl.CH5 >= 1500)
            ch5_zone = 2;       // 下拨 → 夹紧
        else
            ch5_zone = 1;       // 中位 → 保持

        if (ch5_zone != ch5_zone_prev)
        {
            if (ch5_zone == 0)
                clamp_state = 0;   // 张开
            else if (ch5_zone == 2)
                clamp_state = 1;   // 夹紧
            ch5_zone_prev = ch5_zone;
        }
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, clamp_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
