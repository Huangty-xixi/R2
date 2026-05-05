#include "clamp_head.h"

// ==================== 状态变量 ====================
ClampHeadState clamp_head_state = CLAMP_HEAD_IDLE;
static uint32_t clamp_close_start_time = 0;

// ==================== 初始化函数 ====================
void clamp_head_init(void)
{
    clamp_head_state = CLAMP_HEAD_IDLE;
    // 初始化舵机到中间位置
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100);
}

// ==================== 自动夹枪头处理函数 ====================
void clamp_head_auto_process(void)
{
    GPIO_PinState switch_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
    uint32_t current_time = HAL_GetTick();

    switch (clamp_head_state)
    {
        // ==================== 空闲状态 ====================
        case CLAMP_HEAD_IDLE:
        {
            // 检测开关状态
            if (switch_state == GPIO_PIN_SET)
            {
                // 开关打开，打开夹爪
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
                // 舵机转到中间位置
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100);
                // 记录开始时间
                clamp_close_start_time = current_time;
                // 进入等待闭合状态
                clamp_head_state = CLAMP_HEAD_WAIT_CLOSE;
            }
            else
            {
                // 开关关闭，夹爪保持关闭
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            }
            break;
        }

        // ==================== 等待夹爪闭合 ====================
        case CLAMP_HEAD_WAIT_CLOSE:
        {
            // 检测开关是否复位（夹爪闭合）
            if (switch_state == GPIO_PIN_RESET)
            {
                // 开关复位，夹爪已闭合，回到空闲状态
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
                clamp_head_state = CLAMP_HEAD_IDLE;
            }
            else
            {
                // 等待200ms后进入下一步
                if ((current_time - clamp_close_start_time) >= 200)
                {
                    // 舵机转到直立位置
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);
                    clamp_head_state = CLAMP_HEAD_WAIT_DELAY;
                }
            }
            break;
        }

        // ==================== 等待延时后直立 ====================
        case CLAMP_HEAD_WAIT_DELAY:
        {
            // 等待开关复位后回到空闲状态
            if (switch_state == GPIO_PIN_RESET)
            {
                clamp_head_state = CLAMP_HEAD_IDLE;
            }
            break;
        }

        default:
            break;
    }
}

// ==================== zone1专用接口 ====================

/**
 * @brief 启动夹爪序列（zone1步骤1使用）
 * @note  直接打开夹爪+舵机水平，不检测开关状态
 */
void ClampHead_StartSequence(void)
{
    // 打开夹爪
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    // 舵机水平
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100);
    // 重置夹枪状态，准备好接收触发
    clamp_head_state = CLAMP_HEAD_IDLE;
}

/**
 * @brief 触发夹爪闭合（zone1步骤5光电触发后使用）
 * @note  直接触发夹爪闭合+200ms后舵机直立
 */
void ClampHead_TriggerClose(void)
{
    if (clamp_head_state != CLAMP_HEAD_WAIT_CLOSE)
    {
        // 夹爪闭合
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
        // 记录开始时间
        clamp_close_start_time = HAL_GetTick();
        // 进入等待闭合状态（后续由clamp_head_auto_process处理200ms延时和舵机直立）
        clamp_head_state = CLAMP_HEAD_WAIT_CLOSE;
    }
}
