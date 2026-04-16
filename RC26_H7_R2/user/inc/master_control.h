#ifndef __MASTER_CONTROL_H__
#define __MASTER_CONTROL_H__

#include <stdint.h>

/* 通用“电平变化单次触发”门控结构
 * 用途：
 * - 上位机连续发送同一个1/0时，只在电平变化那一拍触发一次动作
 * - 避免“到位后仍重复触发”问题
 */
typedef struct
{
    uint8_t last_level; /* 上一拍的电平值（已归一化为0/1） */
    uint8_t inited;     /* 是否完成初始化 */
} MasterLevelGate;

/* 初始化门控
 * @param gate       门控对象指针
 * @param init_level 初始电平（非0按1处理）
 */
void master_level_gate_init(MasterLevelGate *gate, uint8_t init_level);

/* 检测电平变化
 * @param gate  门控对象指针
 * @param level 当前电平（非0按1处理）
 * @return
 *   1: 本次检测到电平变化（或首次调用），可触发一次动作
 *   0: 电平未变化，不触发动作
 */
uint8_t master_level_gate_on_change(MasterLevelGate *gate, uint8_t level);


#endif
