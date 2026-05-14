/* Compile switches: user/inc/app_init.h or Keil -D */
#include "app_init.h"

#include <stddef.h>

#include "Process_Flow.h"
#include "app_yaw_heading_ctrl.h"
#include "app_zone2.h"
#include "odom_nav_goto.h"

static app_zone2_nav_poll_result_t app_hook_zone2_nav_poll(void)
{
    return odom_nav_goto_run(&odom_nav_target, NULL);
}

void App_Init(void)
{
    Process_UpSlope_Init(odom_nav_goto_set_target, AppYawHeadingCtrl_RunFieldDir);

    app_zone2_init_hooks(
        odom_nav_goto_set_target,
        app_hook_zone2_nav_poll,
        Process_UpStairs,
        Process_DownStairs,
        AppYawHeadingCtrl_RunFieldDir,
        Process_GetKFS,
        AppYawHeadingCtrl_IsBusy,
        Process_UpStairs_IsBusy,
        Process_DownStairs_IsBusy,
        Process_GetKFS_IsBusy);
}
