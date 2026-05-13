#ifndef APP_HOOK_INIT_H
#define APP_HOOK_INIT_H

/**
 * Zone2 / ODOM half: 1=red, 0=blue (app_zone2.c, upper_pc_protocol handle_odom). Edit here or -D in Keil.
 */
#ifndef APP_ZONE2_RED_SIDE
#define APP_ZONE2_RED_SIDE 0
#endif

/**
 * @brief 应用层钩子集中初始化（在此注册回调、弱符号实现等）
 */
void AppHook_Init(void);

#endif
