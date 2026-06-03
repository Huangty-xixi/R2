/* Compile switches: user/inc/app_init.h or Keil -D */
#include "app_init.h"

#include "r1_link.h"
#include "r1_link_z3_cmd_link.h"
#include "r1_uart_rx_dispatch.h"
#include "app_zone3.h"

void App_Init(void)
{
    R1Link_Init();
    R1LinkZ3CmdLink_Init();
    AppZone3_Init();
    R1UartRxDispatch_Start();
}
