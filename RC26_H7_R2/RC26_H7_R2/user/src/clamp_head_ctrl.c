#include "clamp_head_ctrl.h"

#include "main.h"
#include "weapon.h"

#define CLAMP_HEAD_SWITCH_PORT              (GPIOE)
#define CLAMP_HEAD_SWITCH_PIN               (GPIO_PIN_9)

#define CLAMP_HEAD_CLOSE_DELAY_MS           (200U)

#define CLAMP_HEAD_OBJECT_PRESENT_LEVEL     (GPIO_PIN_RESET)
#define CLAMP_HEAD_OBJECT_ABSENT_LEVEL      (GPIO_PIN_SET)

typedef struct
{
    ClampHeadState state;
    uint32_t close_start_tick_ms;
} ClampHeadCtrlCtx;

static ClampHeadCtrlCtx g_clamp_head_ctx = {clamp_head_state_idle, 0U};

#define CLAMP_HEAD_PE9_DEBOUNCE_PRESENT_MS    (20U) // 20ms有物稳定时间
#define CLAMP_HEAD_PE9_DEBOUNCE_ABSENT_MS     (300U) // 300ms无物稳定时间      

static GPIO_PinState s_pe9_last_raw = GPIO_PIN_SET;
static uint32_t s_pe9_raw_since_ms = 0U;

static GPIO_PinState clamp_head_read_switch_level(void)
{
    return HAL_GPIO_ReadPin(CLAMP_HEAD_SWITCH_PORT, CLAMP_HEAD_SWITCH_PIN);
}

static void clamp_head_pe9_filter_reset(uint32_t now_ms)
{
    s_pe9_last_raw = clamp_head_read_switch_level();
    s_pe9_raw_since_ms = now_ms;
}

static void clamp_head_pe9_filter_update(uint32_t now_ms)
{
    GPIO_PinState raw = clamp_head_read_switch_level();

    if (raw != s_pe9_last_raw)
    {
        s_pe9_last_raw = raw;
        s_pe9_raw_since_ms = now_ms;
    }
}

static uint8_t clamp_head_confirmed_present(uint32_t now_ms)
{
    uint32_t thr = CLAMP_HEAD_PE9_DEBOUNCE_PRESENT_MS;

    if (thr == 0U)
    {
        return (uint8_t)(s_pe9_last_raw == CLAMP_HEAD_OBJECT_PRESENT_LEVEL);
    }
    return (uint8_t)((s_pe9_last_raw == CLAMP_HEAD_OBJECT_PRESENT_LEVEL) &&
                     ((now_ms - s_pe9_raw_since_ms) >= thr));
}

static uint8_t clamp_head_confirmed_absent(uint32_t now_ms)
{
    uint32_t thr = CLAMP_HEAD_PE9_DEBOUNCE_ABSENT_MS;

    if (thr == 0U)
    {
        return (uint8_t)(s_pe9_last_raw == CLAMP_HEAD_OBJECT_ABSENT_LEVEL);
    }
    return (uint8_t)((s_pe9_last_raw == CLAMP_HEAD_OBJECT_ABSENT_LEVEL) &&
                     ((now_ms - s_pe9_raw_since_ms) >= thr));
}

static void clamp_head_apply_servo_mid(void)
{
    servo_state = 0U;
    servo_use();
}

static void clamp_head_apply_servo_upright(void)
{
    servo_state = 1U;
    servo_use();
}

static void clamp_head_apply_clamp_open(void)
{
    clamp_state = 0U;
    clamp_use();
}

static void clamp_head_apply_clamp_close(void)
{
    clamp_state = 1U;
    clamp_use();
}

uint8_t ClampHeadCtrl_IsObjectPresentRaw(void)
{
    return (uint8_t)((clamp_head_read_switch_level() == CLAMP_HEAD_OBJECT_PRESENT_LEVEL) ? 1U : 0U);
}

void ClampHeadCtrl_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

    g_clamp_head_ctx.state = clamp_head_state_idle;
    g_clamp_head_ctx.close_start_tick_ms = 0U;

    clamp_head_pe9_filter_reset(now_ms);
    clamp_head_apply_servo_mid();
    clamp_head_apply_clamp_open();
}

void ClampHeadCtrl_Run(void)
{
    uint32_t now_ms = HAL_GetTick();

    clamp_head_pe9_filter_update(now_ms);

    switch (g_clamp_head_ctx.state)
    {
    case clamp_head_state_idle:
        clamp_head_apply_servo_mid();
        clamp_head_apply_clamp_open();

        if (clamp_head_confirmed_present(now_ms) != 0U)
        {
            clamp_head_apply_clamp_close();
            g_clamp_head_ctx.close_start_tick_ms = now_ms;
            g_clamp_head_ctx.state = clamp_head_state_wait_close_delay;
        }
        break;

    case clamp_head_state_wait_close_delay:
        if (clamp_head_confirmed_absent(now_ms) != 0U)
        {
            clamp_head_apply_clamp_open();
            g_clamp_head_ctx.state = clamp_head_state_idle;
            break;
        }

        if ((now_ms - g_clamp_head_ctx.close_start_tick_ms) >= CLAMP_HEAD_CLOSE_DELAY_MS)
        {
            clamp_head_apply_servo_upright();
            g_clamp_head_ctx.state = clamp_head_state_upright_hold;
        }
        break;

    case clamp_head_state_upright_hold:
        clamp_head_apply_servo_upright();
        clamp_head_apply_clamp_close();

        if (clamp_head_confirmed_absent(now_ms) != 0U)
        {
            clamp_head_apply_clamp_open();
            g_clamp_head_ctx.state = clamp_head_state_idle;
        }
        break;

    case clamp_head_state_dock_ok:
        clamp_head_apply_servo_upright();
        clamp_head_apply_clamp_open();
        break;

    default:
        g_clamp_head_ctx.state = clamp_head_state_idle;
        clamp_head_apply_servo_mid();
        clamp_head_apply_clamp_open();
        break;
    }
}

ClampHeadState ClampHeadCtrl_GetState(void)
{
    return g_clamp_head_ctx.state;
}

void ClampHeadCtrl_NotifyDockOk(void)
{
    if (g_clamp_head_ctx.state == clamp_head_state_upright_hold)
    {
        clamp_head_apply_servo_upright();
        clamp_head_apply_clamp_open();
        g_clamp_head_ctx.state = clamp_head_state_dock_ok;
    }
}
