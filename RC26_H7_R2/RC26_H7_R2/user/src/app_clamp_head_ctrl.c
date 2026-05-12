#include "app_clamp_head_ctrl.h"

#include "main.h"
#include "tim.h"

#define APP_CLAMP_HEAD_SWITCH_PORT              (GPIOE)
#define APP_CLAMP_HEAD_SWITCH_PIN               (GPIO_PIN_9)

#define APP_CLAMP_HEAD_CLAMP_PORT               (GPIOC)
#define APP_CLAMP_HEAD_CLAMP_PIN                (GPIO_PIN_10)

#define APP_CLAMP_HEAD_SERVO_TIMER              (&htim2)
#define APP_CLAMP_HEAD_SERVO_CHANNEL            (TIM_CHANNEL_1)

#define APP_CLAMP_HEAD_SERVO_PWM_MID            (1150U)
#define APP_CLAMP_HEAD_SERVO_PWM_UPRIGHT        (2100U)

#define APP_CLAMP_HEAD_CLOSE_DELAY_MS           (200U)

#define APP_CLAMP_HEAD_OBJECT_PRESENT_LEVEL     (GPIO_PIN_RESET)
#define APP_CLAMP_HEAD_OBJECT_ABSENT_LEVEL      (GPIO_PIN_SET)

#define APP_CLAMP_HEAD_CLAMP_CLOSE_LEVEL        (GPIO_PIN_RESET)
#define APP_CLAMP_HEAD_CLAMP_OPEN_LEVEL         (GPIO_PIN_SET)

typedef struct
{
    AppClampHeadState state;
    uint32_t close_start_tick_ms;
} AppClampHeadCtrlCtx;                                          //夹爪控制上下文

static AppClampHeadCtrlCtx g_app_clamp_head_ctx = {app_clamp_head_state_idle, 0U};

static void app_clamp_head_set_servo_mid(void)                   //设置舵机回中
{
    __HAL_TIM_SET_COMPARE(APP_CLAMP_HEAD_SERVO_TIMER,
                          APP_CLAMP_HEAD_SERVO_CHANNEL,
                          APP_CLAMP_HEAD_SERVO_PWM_MID);
}

static void app_clamp_head_set_servo_upright(void)               //设置舵机直立
{
    __HAL_TIM_SET_COMPARE(APP_CLAMP_HEAD_SERVO_TIMER,
                          APP_CLAMP_HEAD_SERVO_CHANNEL,
                          APP_CLAMP_HEAD_SERVO_PWM_UPRIGHT);
}

static void app_clamp_head_set_clamp_open(void)                  //设置夹爪松开
{
    HAL_GPIO_WritePin(APP_CLAMP_HEAD_CLAMP_PORT,
                      APP_CLAMP_HEAD_CLAMP_PIN,
                      APP_CLAMP_HEAD_CLAMP_OPEN_LEVEL);
}

static void app_clamp_head_set_clamp_close(void)                 //设置夹爪关闭
{
    HAL_GPIO_WritePin(APP_CLAMP_HEAD_CLAMP_PORT,
                      APP_CLAMP_HEAD_CLAMP_PIN,
                      APP_CLAMP_HEAD_CLAMP_CLOSE_LEVEL);
}

static GPIO_PinState app_clamp_head_read_switch_level(void)      //读取开关电平
{
    return HAL_GPIO_ReadPin(APP_CLAMP_HEAD_SWITCH_PORT, APP_CLAMP_HEAD_SWITCH_PIN);
}

void AppClampHeadCtrl_Init(void)                                 //初始化
{
    g_app_clamp_head_ctx.state = app_clamp_head_state_idle;
    g_app_clamp_head_ctx.close_start_tick_ms = 0U;

    app_clamp_head_set_servo_mid();
    app_clamp_head_set_clamp_open();
}

void AppClampHeadCtrl_Run(void)                                  //运行
{
    uint32_t now_ms = HAL_GetTick();
    GPIO_PinState switch_level = app_clamp_head_read_switch_level();

    switch (g_app_clamp_head_ctx.state)
    {
        case app_clamp_head_state_idle:                         //空闲状态
            app_clamp_head_set_servo_mid();
            app_clamp_head_set_clamp_open();

            if (switch_level == APP_CLAMP_HEAD_OBJECT_PRESENT_LEVEL)
            {
                app_clamp_head_set_clamp_close();
                g_app_clamp_head_ctx.close_start_tick_ms = now_ms;
                g_app_clamp_head_ctx.state = app_clamp_head_state_wait_close_delay;
            }
            break;

        case app_clamp_head_state_wait_close_delay:             //等待关闭延迟状态
            if (switch_level == APP_CLAMP_HEAD_OBJECT_ABSENT_LEVEL)
            {
                app_clamp_head_set_clamp_open();
                g_app_clamp_head_ctx.state = app_clamp_head_state_idle;
                break;
            }

            if ((now_ms - g_app_clamp_head_ctx.close_start_tick_ms) >= APP_CLAMP_HEAD_CLOSE_DELAY_MS)
            {
                app_clamp_head_set_servo_upright();
                g_app_clamp_head_ctx.state = app_clamp_head_state_upright_hold;
            }
            break;

        case app_clamp_head_state_upright_hold:                 //直立保持状态
            app_clamp_head_set_servo_upright();
            app_clamp_head_set_clamp_close();

            if (switch_level == APP_CLAMP_HEAD_OBJECT_ABSENT_LEVEL)
            {
                app_clamp_head_set_clamp_open();
                g_app_clamp_head_ctx.state = app_clamp_head_state_idle;
            }
            break;

        case app_clamp_head_state_dock_ok:                      //对接成功状态
            app_clamp_head_set_servo_upright();
            app_clamp_head_set_clamp_open();
            break;

        default:                                               //默认状态   
            g_app_clamp_head_ctx.state = app_clamp_head_state_idle;
            app_clamp_head_set_servo_mid();
            app_clamp_head_set_clamp_open();
            break;
    }
}

AppClampHeadState AppClampHeadCtrl_GetState(void)                //获取状态
{
    return g_app_clamp_head_ctx.state;
}

void AppClampHeadCtrl_NotifyDockOk(void)                         //通知对接成功     
{
    if (g_app_clamp_head_ctx.state == app_clamp_head_state_upright_hold)
    {
        app_clamp_head_set_servo_upright();
        app_clamp_head_set_clamp_open();
        g_app_clamp_head_ctx.state = app_clamp_head_state_dock_ok;
    }
}
