#include "app_flow_dispatch.h"

#include "Motion_Task.h"
#include "Process_Flow.h"
#include "R1_R2_connect.h"
#include "app_zone1.h"
#include "app_zone2.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "odom_nav_goto.h"

#include <math.h>
#include <string.h>

typedef enum
{
    app_flow_state_idle = 0,
    app_flow_state_zone1_flow,
    app_flow_state_zone2_flow,
    app_flow_state_done,
    app_flow_state_abort,
} AppFlowState;

typedef struct
{
    AppFlowState state;
    uint32_t state_enter_ms;
} AppFlowDispatchCtx;

static AppFlowDispatchCtx g_app_flow_ctx;

static app_zone2_mission_t s_zone2_mission_pending;
static uint8_t s_zone2_mission_pending_valid = 0U;
static uint8_t s_zone2_start_no_mission = 0U;

typedef enum
{
    app_flow_abort_reason_none = 0,
    app_flow_abort_reason_mode_none,
    app_flow_abort_reason_zone1_failed,
    app_flow_abort_reason_zone2_failed,
} app_flow_abort_reason_t;

static void app_flow_r1_mission_to_zone2(const r1_r2_mission_t *src, app_zone2_mission_t *dst)
{
    uint8_t i;
    uint8_t pn;
    uint8_t kn;

    if ((src == 0) || (dst == 0))
    {
        return;
    }

    memset(dst, 0, sizeof(*dst));

    pn = src->path_n;
    if ((pn == 0U) || (pn > APP_ZONE2_MAX_PATH))
    {
        for (i = 0U; i < R1_R2_CONNECT_MAX_PATH && i < APP_ZONE2_MAX_PATH; i++)
        {
            if (src->path[i] == 0U)
            {
                break;
            }
            dst->path[i] = src->path[i];
        }
        pn = i;
    }
    else
    {
        for (i = 0U; i < pn && i < APP_ZONE2_MAX_PATH; i++)
        {
            dst->path[i] = src->path[i];
        }
    }
    dst->path_n = pn;

    kn = src->kfs_n;
    if ((kn == 0U) || (kn > APP_ZONE2_MAX_KFS))
    {
        for (i = 0U; i < R1_R2_CONNECT_MAX_KFS && i < APP_ZONE2_MAX_KFS; i++)
        {
            if (src->kfs[i] == 0U)
            {
                break;
            }
            dst->kfs[i] = src->kfs[i];
        }
        kn = i;
    }
    else
    {
        for (i = 0U; i < kn && i < APP_ZONE2_MAX_KFS; i++)
        {
            dst->kfs[i] = src->kfs[i];
        }
    }
    dst->kfs_n = kn;
}

static void app_flow_on_r1_mission_decoded(const r1_r2_mission_t *mission, void *user)
{
    (void)user;
    if (mission == NULL)
    {
        return;
    }
    app_flow_r1_mission_to_zone2(mission, &s_zone2_mission_pending);
    s_zone2_mission_pending_valid = 1U;
}

void AppFlowDispatch_OnR1WireMission(const r1_r2_mission_t *mission)
{
    app_flow_on_r1_mission_decoded(mission, NULL);
}

static void app_flow_zone2_start(void)
{
    s_zone2_start_no_mission = 0U;

    if (s_zone2_mission_pending_valid != 0U)
    {
        app_zone2_mission_apply(&s_zone2_mission_pending);
        s_zone2_mission_pending_valid = 0U;
    }
    else
    {
        app_zone2_mission_clear();
        s_zone2_start_no_mission = 1U;
    }
    app_flow_mode = app_flow_zone2;
}

static void app_flow_enter_state(AppFlowState state, uint32_t now_ms)
{
    g_app_flow_ctx.state = state;
    g_app_flow_ctx.state_enter_ms = now_ms;
}

static void app_flow_cleanup_to_idle(void)
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_clear_state();
    AppZone1_Reset();
    app_zone2_mission_clear();
    app_flow_mode = app_flow_none;
    g_app_flow_ctx.state = app_flow_state_idle;
    g_app_flow_ctx.state_enter_ms = osKernelGetTickCount();
}

void AppFlowDispatch_Init(void)
{
    r1_r2_connect_hooks_t hooks;

    g_app_flow_ctx.state = app_flow_state_idle;
    g_app_flow_ctx.state_enter_ms = 0U;
    memset(&s_zone2_mission_pending, 0, sizeof(s_zone2_mission_pending));
    s_zone2_mission_pending_valid = 0U;
    s_zone2_start_no_mission = 0U;
    app_flow_mode = app_flow_none;

    AppZone1_Init();

    hooks.on_decoded = app_flow_on_r1_mission_decoded;
    hooks.user = 0;
    r1_r2_connect_set_hooks(&hooks);
}

void AppFlowDispatch_Run(void)
{
    uint32_t now_ms;
    app_flow_abort_reason_t abort_reason = app_flow_abort_reason_none;

    if ((control_mode == emergency_stop_mode) || (control_mode == remote_control))
    {
        app_flow_cleanup_to_idle();
        return;
    }

    if (control_mode != full_auto_control)
    {
        app_flow_cleanup_to_idle();
        return;
    }

    now_ms = osKernelGetTickCount();

    switch (g_app_flow_ctx.state)
    {
        case app_flow_state_idle:
            if (app_flow_mode == app_flow_zone1)
            {
                AppZone1_Start();
                app_flow_enter_state(app_flow_state_zone1_flow, now_ms);
            }
            break;

        case app_flow_state_zone1_flow:
            AppZone1_Run();
            if (AppZone1_IsFailed() != 0U)
            {
                abort_reason = app_flow_abort_reason_zone1_failed;
                app_flow_enter_state(app_flow_state_abort, now_ms);
            }
            else if (AppZone1_IsDone() != 0U)
            {
                app_flow_zone2_start();
                app_flow_enter_state(app_flow_state_zone2_flow, now_ms);
            }
            else if (app_flow_mode == app_flow_none)
            {
                abort_reason = app_flow_abort_reason_mode_none;
                app_flow_enter_state(app_flow_state_abort, now_ms);
            }
            break;

        case app_flow_state_zone2_flow:
            if (s_zone2_start_no_mission != 0U)
            {
                abort_reason = app_flow_abort_reason_zone2_failed;
                app_flow_enter_state(app_flow_state_abort, now_ms);
                break;
            }
            app_zone2_poll();
            if (app_zone2_is_done() != 0U)
            {
                app_flow_enter_state(app_flow_state_done, now_ms);
            }
            break;

        case app_flow_state_done:
            app_flow_cleanup_to_idle();
            break;

        case app_flow_state_abort:
            Process_Flow_ClearChassisOverride();
            app_flow_mode = app_flow_none;
            app_flow_cleanup_to_idle();
            break;

        default:
            app_flow_enter_state(app_flow_state_abort, now_ms);
            break;
    }

    (void)abort_reason;
    (void)now_ms;
}
