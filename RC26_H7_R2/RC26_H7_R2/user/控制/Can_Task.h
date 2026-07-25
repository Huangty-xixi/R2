#ifndef __CAN_TASK_H__
#define __CAN_TASK_H__

#include "cmsis_os.h"
#include "chassis.h"

/* TX FIFO min-free-level trackers (debugger-visible) */
extern volatile uint32_t g_can1_tx_fifo_min_free;
extern volatile uint32_t g_can2_tx_fifo_min_free;
extern volatile uint32_t g_can3_tx_fifo_min_free;

#endif
