/**
 * @file rc_odom_snap.c
 * @brief 里程计快照：由协议栈 ODOM 回调写入（默认在 rc_init 中已注册）
 */
#include "rc_odom_snap.h"
#include <string.h>
#include "main.h"

static rc_odom_t s_snap;

void RcOdomSnap_OnRcOdom(const rc_odom_t *odom)
{
    if (odom == NULL)
    {
        return;
    }
    __disable_irq();
    memcpy(&s_snap, odom, sizeof(s_snap));
    __enable_irq();
}

void RcOdomSnap_Read(rc_odom_t *out)
{
    if (out == NULL)
    {
        return;
    }
    __disable_irq();
    memcpy(out, &s_snap, sizeof(*out));
    __enable_irq();
}
