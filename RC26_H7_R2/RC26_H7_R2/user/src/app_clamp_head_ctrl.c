#include "app_clamp_head_ctrl.h"



#include "main.h"

#include "weapon.h"



#define APP_CLAMP_HEAD_SWITCH_PORT              (GPIOE)

#define APP_CLAMP_HEAD_SWITCH_PIN               (GPIO_PIN_9)



#define APP_CLAMP_HEAD_CLOSE_DELAY_MS           (200U)



#define APP_CLAMP_HEAD_OBJECT_PRESENT_LEVEL     (GPIO_PIN_RESET)

#define APP_CLAMP_HEAD_OBJECT_ABSENT_LEVEL      (GPIO_PIN_SET)



typedef struct

{

    AppClampHeadState state;        //当前状态

    uint32_t close_start_tick_ms;   //夹爪关闭开始时间  

} AppClampHeadCtrlCtx;      



static AppClampHeadCtrlCtx g_app_clamp_head_ctx = {app_clamp_head_state_idle, 0U};   //夹爪控制上下文



#define APP_CLAMP_HEAD_PE9_DEBOUNCE_PRESENT_MS    (20U)
#define APP_CLAMP_HEAD_PE9_DEBOUNCE_ABSENT_MS     (300U)




/** 原始电平跟踪：电平变化后重新计时，用于去抖 */

static GPIO_PinState s_pe9_last_raw = GPIO_PIN_SET;

static uint32_t s_pe9_raw_since_ms = 0U;



static GPIO_PinState app_clamp_head_read_switch_level(void) 
{   //读取PE9引脚电平
    return HAL_GPIO_ReadPin(APP_CLAMP_HEAD_SWITCH_PORT, APP_CLAMP_HEAD_SWITCH_PIN);
}

static void app_clamp_head_pe9_filter_reset(uint32_t now_ms)

{

    s_pe9_last_raw = app_clamp_head_read_switch_level();

    s_pe9_raw_since_ms = now_ms;

}



static void app_clamp_head_pe9_filter_update(uint32_t now_ms)
{   //更新PE9引脚电平滤波       

    GPIO_PinState raw = app_clamp_head_read_switch_level();



    if (raw != s_pe9_last_raw)

    {

        s_pe9_last_raw = raw;

        s_pe9_raw_since_ms = now_ms;

    }

}



static uint8_t app_clamp_head_confirmed_present(uint32_t now_ms)    
{   //确认有物  
    uint32_t thr = APP_CLAMP_HEAD_PE9_DEBOUNCE_PRESENT_MS;



    if (thr == 0U)

    {

        return (uint8_t)(s_pe9_last_raw == APP_CLAMP_HEAD_OBJECT_PRESENT_LEVEL);

    }

    return (uint8_t)((s_pe9_last_raw == APP_CLAMP_HEAD_OBJECT_PRESENT_LEVEL) &&

                     ((now_ms - s_pe9_raw_since_ms) >= thr));

}



static uint8_t app_clamp_head_confirmed_absent(uint32_t now_ms) 
{   //确认无物      
    uint32_t thr = APP_CLAMP_HEAD_PE9_DEBOUNCE_ABSENT_MS;



    if (thr == 0U)

    {

        return (uint8_t)(s_pe9_last_raw == APP_CLAMP_HEAD_OBJECT_ABSENT_LEVEL);

    }

    return (uint8_t)((s_pe9_last_raw == APP_CLAMP_HEAD_OBJECT_ABSENT_LEVEL) &&

                     ((now_ms - s_pe9_raw_since_ms) >= thr));

}



/** 偶 servo_state -> 中位 PWM，经 weapon.servo_use 输出 */

static void app_clamp_head_apply_servo_mid(void)

{

    servo_state = 0U;

    servo_use();

}



/** 奇 servo_state -> 直立 PWM */

static void app_clamp_head_apply_servo_upright(void)

{

    servo_state = 1U;

    servo_use();

}



/** 偶 clamp_state -> 张开（PC10 SET），与 weapon.clamp_use 一致 */

static void app_clamp_head_apply_clamp_open(void)

{

    clamp_state = 0U;

    clamp_use();

}



/** 奇 clamp_state -> 夹紧（PC10 RESET） */

static void app_clamp_head_apply_clamp_close(void)

{

    clamp_state = 1U;

    clamp_use();

}



uint8_t AppClampHeadCtrl_IsObjectPresentRaw(void)

{

    return (uint8_t)((app_clamp_head_read_switch_level() == APP_CLAMP_HEAD_OBJECT_PRESENT_LEVEL) ? 1U : 0U);

}



void AppClampHeadCtrl_Init(void)

{

    uint32_t now_ms = HAL_GetTick();



    g_app_clamp_head_ctx.state = app_clamp_head_state_idle;

    g_app_clamp_head_ctx.close_start_tick_ms = 0U;



    app_clamp_head_pe9_filter_reset(now_ms);



    app_clamp_head_apply_servo_mid();

    app_clamp_head_apply_clamp_open();

}



void AppClampHeadCtrl_Run(void)

{

    uint32_t now_ms = HAL_GetTick();



    app_clamp_head_pe9_filter_update(now_ms);



    switch (g_app_clamp_head_ctx.state)

    {

    case app_clamp_head_state_idle:

        app_clamp_head_apply_servo_mid();

        app_clamp_head_apply_clamp_open();



        if (app_clamp_head_confirmed_present(now_ms) != 0U)

        {

            app_clamp_head_apply_clamp_close();

            g_app_clamp_head_ctx.close_start_tick_ms = now_ms;

            g_app_clamp_head_ctx.state = app_clamp_head_state_wait_close_delay;

        }

        break;



    case app_clamp_head_state_wait_close_delay:

        if (app_clamp_head_confirmed_absent(now_ms) != 0U)

        {

            app_clamp_head_apply_clamp_open();

            g_app_clamp_head_ctx.state = app_clamp_head_state_idle;

            break;

        }



        if ((now_ms - g_app_clamp_head_ctx.close_start_tick_ms) >= APP_CLAMP_HEAD_CLOSE_DELAY_MS)

        {

            app_clamp_head_apply_servo_upright();

            g_app_clamp_head_ctx.state = app_clamp_head_state_upright_hold;

        }

        break;



    case app_clamp_head_state_upright_hold:

        app_clamp_head_apply_servo_upright();

        app_clamp_head_apply_clamp_close();



        if (app_clamp_head_confirmed_absent(now_ms) != 0U)

        {

            app_clamp_head_apply_clamp_open();

            g_app_clamp_head_ctx.state = app_clamp_head_state_idle;

        }

        break;



    case app_clamp_head_state_dock_ok:

        app_clamp_head_apply_servo_upright();

        app_clamp_head_apply_clamp_open();

        break;



    default:

        g_app_clamp_head_ctx.state = app_clamp_head_state_idle;

        app_clamp_head_apply_servo_mid();

        app_clamp_head_apply_clamp_open();

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

        app_clamp_head_apply_servo_upright();

        app_clamp_head_apply_clamp_open();

        g_app_clamp_head_ctx.state = app_clamp_head_state_dock_ok;

    }

}


