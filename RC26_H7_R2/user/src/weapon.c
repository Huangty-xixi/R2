#include "weapon.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"

// ״̬��־
uint8_t servo_state = 1;    // ���״̬
uint8_t pump_state = 0;     // ����״̬

// ��������ȫ��Ψһ��
uint8_t ch5_lock = 0;


/**
  * @brief 武器运行逻辑
  */
void manual_weapon_function(void)
{
	//空函数
}



/**
  * @brief ���ʼ������
  */
void weapon_init(void)
{
    // 设置舵机初始位置
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500); // 中间位置
    // 测试PWM输出
//    HAL_Delay(1000);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000); // 另一个位置
//    HAL_Delay(1000);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000); // 最小位置
//    HAL_Delay(1000);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500); // 回到中间位置
}









/**
  * @brief �������
  */
void steering_use(void)
{
    // 检查遥控器信号是否正常
    if (RCctrl.rc_lost) {
        return;
    }
    
    // ͳһ�� CH5=192 ����
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        servo_state ^= 1; // ��ת
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
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100); // 另一个位置
    }
}

/**
  * @brief ���ÿ���
  */
void pump_use(void)
{
    // ͳһ�� CH5=192 ����
    if (RCctrl.CH5 ==192 && ch5_lock == 0)
    {
        pump_state ^= 1; // ��ת
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
  * @brief ��ȫ����λ����������״̬
  */
void weapon_reset_all(void)
{
    // �պ��� �� �κ�ģʽ�л�������λ��������ر�����
}