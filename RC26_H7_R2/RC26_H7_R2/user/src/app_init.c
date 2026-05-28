/* Compile switches: user/inc/app_init.h or Keil -D */
#include "app_init.h"

#include "r1_link.h"

void App_Init(void)
{
    R1Link_Init();
}
