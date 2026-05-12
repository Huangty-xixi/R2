#include "app_hook_init.h"

#include <stddef.h>

#include "Process_Flow.h"
#include "app_yaw_heading_ctrl.h"
#include "app_zone2.h"
#include "odom_nav_goto.h"

static app_zone2_nav_poll_result_t app_hook_zone2_nav_poll(void)
{
    return odom_nav_goto_run(&odom_nav_target, NULL);
}

void AppHook_Init(void)
{
    /* 上坡：到点 + 场地朝向摆头（与二区 face 钩子同一套 AppYawHeadingCtrl_RunFieldDir） */
    Process_UpSlope_Init(odom_nav_goto_set_target, AppYawHeadingCtrl_RunFieldDir);

    app_zone2_init_hooks(
        odom_nav_goto_set_target,
        app_hook_zone2_nav_poll,
        Process_UpStairs,
        Process_DownStairs,
        AppYawHeadingCtrl_RunFieldDir,
        Process_GetKFS);
}
