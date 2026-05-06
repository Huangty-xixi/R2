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
} AppClampHeadCtrlCtx;

static AppClampHeadCtrlCtx g_app_clamp_head_ctx = {app_clamp_head_state_idle, 0U};

static void app_clamp_head_set_servo_mid(void)
{
    __HAL_TIM_SET_COMPARE(APP_CLAMP_HEAD_SERVO_TIMER,
                          APP_CLAMP_HEAD_SERVO_CHANNEL,
                          APP_CLAMP_HEAD_SERVO_PWM_MID);
}

static void app_clamp_head_set_servo_upright(void)
{
    __HAL_TIM_SET_COMPARE(APP_CLAMP_HEAD_SERVO_TIMER,
                          APP_CLAMP_HEAD_SERVO_CHANNEL,
                          APP_CLAMP_HEAD_SERVO_PWM_UPRIGHT);
}

static void app_clamp_head_set_clamp_open(void)
{
    HAL_GPIO_WritePin(APP_CLAMP_HEAD_CLAMP_PORT,
                      APP_CLAMP_HEAD_CLAMP_PIN,
                      APP_CLAMP_HEAD_CLAMP_OPEN_LEVEL);
}

static void app_clamp_head_set_clamp_close(void)
{
    HAL_GPIO_WritePin(APP_CLAMP_HEAD_CLAMP_PORT,
                      APP_CLAMP_HEAD_CLAMP_PIN,
                      APP_CLAMP_HEAD_CLAMP_CLOSE_LEVEL);
}

static GPIO_PinState app_clamp_head_read_switch_level(void)
{
    return HAL_GPIO_ReadPin(APP_CLAMP_HEAD_SWITCH_PORT, APP_CLAMP_HEAD_SWITCH_PIN);
}

void AppClampHeadCtrl_Init(void)
{
    g_app_clamp_head_ctx.state = app_clamp_head_state_idle;
    g_app_clamp_head_ctx.close_start_tick_ms = 0U;

    app_clamp_head_set_servo_mid();
    app_clamp_head_set_clamp_open();
}

void AppClampHeadCtrl_Run(void)
{
    uint32_t now_ms = HAL_GetTick();
    GPIO_PinState switch_level = app_clamp_head_read_switch_level();

    switch (g_app_clamp_head_ctx.state)
    {
        case app_clamp_head_state_idle:
            app_clamp_head_set_servo_mid();
            app_clamp_head_set_clamp_open();

            if (switch_level == APP_CLAMP_HEAD_OBJECT_PRESENT_LEVEL)
            {
                app_clamp_head_set_clamp_close();
                g_app_clamp_head_ctx.close_start_tick_ms = now_ms;
                g_app_clamp_head_ctx.state = app_clamp_head_state_wait_close_delay;
            }
            break;

        case app_clamp_head_state_wait_close_delay:
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

        case app_clamp_head_state_upright_hold:
            app_clamp_head_set_servo_upright();
            app_clamp_head_set_clamp_close();

            if (switch_level == APP_CLAMP_HEAD_OBJECT_ABSENT_LEVEL)
            {
                app_clamp_head_set_clamp_open();
                g_app_clamp_head_ctx.state = app_clamp_head_state_idle;
            }
            break;

        case app_clamp_head_state_dock_ok:
            app_clamp_head_set_servo_upright();
            app_clamp_head_set_clamp_open();
            break;

        default:
            g_app_clamp_head_ctx.state = app_clamp_head_state_idle;
            app_clamp_head_set_servo_mid();
            app_clamp_head_set_clamp_open();
            break;
    }
}

AppClampHeadState AppClampHeadCtrl_GetState(void)
{
    return g_app_clamp_head_ctx.state;
}

void AppClampHeadCtrl_NotifyDockOk(void)
{
    if (g_app_clamp_head_ctx.state == app_clamp_head_state_upright_hold)
    {
        app_clamp_head_set_servo_upright();
        app_clamp_head_set_clamp_open();
        g_app_clamp_head_ctx.state = app_clamp_head_state_dock_ok;
    }
}
