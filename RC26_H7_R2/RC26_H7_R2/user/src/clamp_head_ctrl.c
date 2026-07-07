#include "clamp_head_ctrl.h"

#include "main.h"
#include "weapon.h"

#define CLAMP_HEAD_SWITCH_PORT              (GPIOE)
#define CLAMP_HEAD_SWITCH_PIN               (GPIO_PIN_9)

#define CLAMP_HEAD_OBJECT_PRESENT_LEVEL     (GPIO_PIN_RESET)
#define CLAMP_HEAD_OBJECT_ABSENT_LEVEL      (GPIO_PIN_SET)


typedef struct
{
    ClampHeadState state;
    uint32_t close_start_tick_ms;
} ClampHeadCtrlCtx;

volatile clamp_head_ctrl_cfg_t g_clamp_head_ctrl_cfg = {
    .close_delay_ms = 20U,  /* 合闸等待时间(ms)，默认200 */
    .pe9_debounce_present_ms = 20U,  /* 有物去抖(ms)，默认20 */
    .pe9_debounce_absent_ms = 200U, /* 无物去抖(ms)，默认200 */
    .grab_cooldown_ms = 0U,  /* 夹取冷却期(ms)，默认500 */
};
static uint32_t s_last_grab_attempt_ms = 0U;
static ClampHeadCtrlCtx g_clamp_head_ctx = {clamp_head_state_idle, 0U};
volatile clamp_head_ctrl_dbg_t g_clamp_head_ctrl_dbg;
static uint8_t s_clamp_head_auto_grab_enable = 0U;

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
    uint32_t thr = g_clamp_head_ctrl_cfg.pe9_debounce_present_ms;

    if (thr == 0U)
    {
        return (uint8_t)(s_pe9_last_raw == CLAMP_HEAD_OBJECT_PRESENT_LEVEL);
    }
    return (uint8_t)((s_pe9_last_raw == CLAMP_HEAD_OBJECT_PRESENT_LEVEL) &&
                     ((now_ms - s_pe9_raw_since_ms) >= thr));
}

static uint8_t clamp_head_confirmed_absent(uint32_t now_ms)
{
    uint32_t thr = g_clamp_head_ctrl_cfg.pe9_debounce_absent_ms;

    if (thr == 0U)
    {
        return (uint8_t)(s_pe9_last_raw == CLAMP_HEAD_OBJECT_ABSENT_LEVEL);
    }
    return (uint8_t)((s_pe9_last_raw == CLAMP_HEAD_OBJECT_ABSENT_LEVEL) &&
                     ((now_ms - s_pe9_raw_since_ms) >= thr));
}

static void clamp_head_dbg_refresh(uint32_t now_ms)
{
    g_clamp_head_ctrl_dbg.state = g_clamp_head_ctx.state;
    g_clamp_head_ctrl_dbg.pe9_present_raw = ClampHeadCtrl_IsObjectPresentRaw();
    g_clamp_head_ctrl_dbg.pe9_present_filt = clamp_head_confirmed_present(now_ms);
    g_clamp_head_ctrl_dbg.pe9_absent_filt = clamp_head_confirmed_absent(now_ms);

    g_clamp_head_ctrl_dbg.close_start_tick_ms = g_clamp_head_ctx.close_start_tick_ms;
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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
}

static void clamp_head_apply_clamp_close(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
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
    clamp_head_dbg_refresh(now_ms);
}

void ClampHeadCtrl_SetAutoGrabEnable(uint8_t enable)
{
    s_clamp_head_auto_grab_enable = (uint8_t)(enable != 0U);
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

        if ((s_clamp_head_auto_grab_enable != 0U) &&
            (clamp_head_confirmed_present(now_ms) != 0U))
        {
            if ((now_ms - s_last_grab_attempt_ms) < g_clamp_head_ctrl_cfg.grab_cooldown_ms)
                break;
            s_last_grab_attempt_ms = now_ms;
            clamp_head_apply_clamp_close();
            g_clamp_head_ctx.close_start_tick_ms = now_ms;
            g_clamp_head_ctx.state = clamp_head_state_wait_close_delay;
        }
        break;



    case clamp_head_state_wait_close_delay:
        clamp_head_apply_clamp_close();

        if (clamp_head_confirmed_absent(now_ms) != 0U)
        {
            clamp_head_apply_clamp_open();
            g_clamp_head_ctx.state = clamp_head_state_idle;
            break;
        }

        if ((now_ms - g_clamp_head_ctx.close_start_tick_ms) >= g_clamp_head_ctrl_cfg.close_delay_ms)
        {
            if (clamp_head_confirmed_present(now_ms) != 0U)
            {
                clamp_head_apply_servo_upright();
                g_clamp_head_ctx.state = clamp_head_state_upright_hold;
            }
            else
            {
                clamp_head_apply_clamp_open();
                g_clamp_head_ctx.state = clamp_head_state_idle;
            }
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

    clamp_head_dbg_refresh(now_ms);
}

void ClampHeadCtrl_DoClose(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
}

void ClampHeadCtrl_DoOpen(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
}

void ClampHeadCtrl_DoServoMid(void)
{
    servo_state = 0U;
    servo_use();
}

void ClampHeadCtrl_DoServoUpright(void)
{
    servo_state = 1U;
    servo_use();
}
