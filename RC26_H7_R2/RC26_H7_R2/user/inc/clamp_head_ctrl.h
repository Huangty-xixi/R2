#ifndef CLAMP_HEAD_CTRL_H
#define CLAMP_HEAD_CTRL_H

#include <stdint.h>

typedef enum
{
    clamp_head_state_idle = 0,
    clamp_head_state_closing,
    clamp_head_state_wait_close_delay,
    clamp_head_state_upright_hold,
    clamp_head_state_opening,
    clamp_head_state_dock_ok,
} ClampHeadState;

/** Keil Watch£º¼ÐÍ·¿ØÖÆµ÷ÊÔ */
typedef struct
{
    ClampHeadState state;
    uint8_t pe9_present_raw;
    uint8_t pe9_present_filt;
    uint8_t pe9_absent_filt;
    uint8_t reached_close_limit;
    uint8_t motor_at_open;
    uint8_t motor_at_close;
    uint8_t motor_busy;
    uint32_t close_start_tick_ms;
} clamp_head_ctrl_dbg_t;

extern volatile clamp_head_ctrl_dbg_t g_clamp_head_ctrl_dbg;

/* 0=??????  1=?????????????????????????ClampHeadCtrl_Run */
#ifndef APP_ZONE1_DBG_CLAMP_HEAD_ONLY
#define APP_ZONE1_DBG_CLAMP_HEAD_ONLY  (1U)
#endif

void ClampHeadCtrl_Init(void);
void ClampHeadCtrl_Run(void);
void ClampHeadCtrl_NotifyDockOk(void);
ClampHeadState ClampHeadCtrl_GetState(void);
uint8_t ClampHeadCtrl_IsObjectPresentRaw(void);
uint8_t ClampHeadCtrl_ReachedCloseLimit(void);

#endif /* CLAMP_HEAD_CTRL_H */
