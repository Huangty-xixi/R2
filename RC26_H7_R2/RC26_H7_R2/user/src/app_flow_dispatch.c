#include "app_flow_dispatch.h"

#include "Motion_Task.h"
#include "Process_Flow.h"
#include "cmsis_os.h"
#include "odom_nav_goto.h"

#define APP_FLOW_NAV_TARGET_X_M            (1.0f)
#define APP_FLOW_NAV_TARGET_Y_M            (1.0f)
#define APP_FLOW_NAV_TIMEOUT_MS            (15000U)
#define APP_FLOW_ACTION_TIMEOUT_MS         (15000U)
#define APP_FLOW_SESSION_ID_INIT           (1U)

typedef enum
{
    app_flow_state_idle = 0,
    app_flow_state_nav_to_point,
    app_flow_state_do_action,
    app_flow_state_done,
    app_flow_state_abort
} AppFlowState;

typedef enum
{
    app_flow_action_none = 0,
    app_flow_action_upstairs,
    app_flow_action_downstairs,
    app_flow_action_get_kfs,
    app_flow_action_put_kfs
} AppFlowAction;

typedef odom_nav_goto_err_t (*AppFlowNavRunFn)(const odom_nav_goto_target_t *target, odom_nav_goto_status_t *status);
typedef void (*AppFlowActionFn)(void);

typedef struct
{
    AppFlowState state;
    AppFlowAction action;
    Semi_auto_mode action_mode;
    uint32_t state_enter_ms;
    uint32_t session_id_seed;
    odom_nav_goto_target_t target;
    AppFlowNavRunFn nav_run;
    AppFlowActionFn action_run;
} AppFlowDispatchCtx;

static AppFlowDispatchCtx g_app_flow_ctx;

static uint8_t app_flow_action_from_mode(Semi_auto_mode mode, AppFlowAction *action_out)
{
    if (action_out == 0)
    {
        return 0U;
    }

    switch (mode)
    {
        case semi_auto_upstairs_mode:
            *action_out = app_flow_action_upstairs;
            return 1U;
        case semi_auto_downstairs_mode:
            *action_out = app_flow_action_downstairs;
            return 1U;
        case semi_auto_get_kfs_mode:
            *action_out = app_flow_action_get_kfs;
            return 1U;
        case semi_auto_put_kfs_mode:
            *action_out = app_flow_action_put_kfs;
            return 1U;
        default:
            *action_out = app_flow_action_none;
            return 0U;
    }
}

static void app_flow_default_action_run(void)
{
    switch (g_app_flow_ctx.action)
    {
        case app_flow_action_upstairs:
            Process_UpStairs();
            break;
        case app_flow_action_downstairs:
            Process_DownStairs();
            break;
        case app_flow_action_get_kfs:
            Process_GetKFS();
            break;
        case app_flow_action_put_kfs:
            Process_PutKFS();
            break;
        case app_flow_action_none:
        default:
            break;
    }
}

static void app_flow_cleanup_to_idle(void)
{
    Process_Flow_ClearChassisOverride();
    odom_nav_goto_clear_state();
    g_app_flow_ctx.action = app_flow_action_none;
    g_app_flow_ctx.action_mode = semi_auto_none;
    g_app_flow_ctx.state = app_flow_state_idle;
    g_app_flow_ctx.state_enter_ms = osKernelGetTickCount();
}

static void app_flow_start_nav(AppFlowAction action, Semi_auto_mode action_mode)
{
    g_app_flow_ctx.action = action;
    g_app_flow_ctx.action_mode = action_mode;
    g_app_flow_ctx.target.x_m = APP_FLOW_NAV_TARGET_X_M;
    g_app_flow_ctx.target.y_m = APP_FLOW_NAV_TARGET_Y_M;
    g_app_flow_ctx.target.session_id = g_app_flow_ctx.session_id_seed++;
    odom_nav_goto_clear_state();
    g_app_flow_ctx.state = app_flow_state_nav_to_point;
    g_app_flow_ctx.state_enter_ms = osKernelGetTickCount();
}

void AppFlowDispatch_Init(void)
{
    g_app_flow_ctx.state = app_flow_state_idle;
    g_app_flow_ctx.action = app_flow_action_none;
    g_app_flow_ctx.action_mode = semi_auto_none;
    g_app_flow_ctx.state_enter_ms = 0U;
    g_app_flow_ctx.session_id_seed = APP_FLOW_SESSION_ID_INIT;
    g_app_flow_ctx.target.x_m = 0.0f;
    g_app_flow_ctx.target.y_m = 0.0f;
    g_app_flow_ctx.target.session_id = APP_FLOW_SESSION_ID_INIT;
    g_app_flow_ctx.nav_run = odom_nav_goto_run;
    g_app_flow_ctx.action_run = app_flow_default_action_run;
}

void AppFlowDispatch_Run(void)
{
    uint32_t now_ms;
    AppFlowAction request_action = app_flow_action_none;
    odom_nav_goto_err_t nav_rc;

    if ((control_mode != semi_auto_control) || (g_app_flow_ctx.nav_run == 0) || (g_app_flow_ctx.action_run == 0))
    {
        app_flow_cleanup_to_idle();
        return;
    }

    now_ms = osKernelGetTickCount();

    switch (g_app_flow_ctx.state)
    {
        case app_flow_state_idle:
            if (app_flow_action_from_mode(semi_auto_mode, &request_action) != 0U)
            {
                app_flow_start_nav(request_action, semi_auto_mode);
            }
            break;

        case app_flow_state_nav_to_point:
            if ((semi_auto_mode == semi_auto_none) || ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_NAV_TIMEOUT_MS))
            {
                g_app_flow_ctx.state = app_flow_state_abort;
                g_app_flow_ctx.state_enter_ms = now_ms;
                break;
            }

            nav_rc = g_app_flow_ctx.nav_run(&g_app_flow_ctx.target, 0);
            if (nav_rc == ODOM_NAV_GOTO_ERR_OK_ARRIVED)
            {
                Process_Flow_ClearChassisOverride();
                semi_auto_mode = g_app_flow_ctx.action_mode;
                g_app_flow_ctx.state = app_flow_state_do_action;
                g_app_flow_ctx.state_enter_ms = now_ms;
            }
            else if ((nav_rc == ODOM_NAV_GOTO_ERR_TIMEOUT) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_ODOM_READ) ||
                     (nav_rc == ODOM_NAV_GOTO_ERR_BAD_CONFIG))
            {
                g_app_flow_ctx.state = app_flow_state_abort;
                g_app_flow_ctx.state_enter_ms = now_ms;
            }
            else
            {
                /* moving */
            }
            break;

        case app_flow_state_do_action:
            if ((now_ms - g_app_flow_ctx.state_enter_ms) > APP_FLOW_ACTION_TIMEOUT_MS)
            {
                g_app_flow_ctx.state = app_flow_state_abort;
                g_app_flow_ctx.state_enter_ms = now_ms;
                break;
            }

            g_app_flow_ctx.action_run();
            if (semi_auto_mode == semi_auto_none)
            {
                g_app_flow_ctx.state = app_flow_state_done;
                g_app_flow_ctx.state_enter_ms = now_ms;
            }
            break;

        case app_flow_state_done:
            app_flow_cleanup_to_idle();
            break;

        case app_flow_state_abort:
            Process_Flow_ClearChassisOverride();
            semi_auto_mode = semi_auto_none;
            app_flow_cleanup_to_idle();
            break;

        default:
            g_app_flow_ctx.state = app_flow_state_abort;
            g_app_flow_ctx.state_enter_ms = now_ms;
            break;
    }
}
