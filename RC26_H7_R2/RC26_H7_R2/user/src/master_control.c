#include "master_control.h"

/* 初始化门控对象：
 * - 把last_level设置成给定初值
 * - 标记已初始化
 */
void master_level_gate_init(MasterLevelGate *gate, uint8_t init_level)
{
    if (gate == 0)
    {
        return;
    }

    gate->last_level = (init_level != 0U) ? 1U : 0U;
    gate->inited = 1U;
}

/* 检测“当前电平”是否相对上一拍发生变化：
 * - 有变化：返回1（允许触发一次动作）
 * - 无变化：返回0（不重复触发）
 */
uint8_t master_level_gate_on_change(MasterLevelGate *gate, uint8_t level)
{
    uint8_t norm_level = (level != 0U) ? 1U : 0U;

    if (gate == 0)
    {
        return 0U;
    }

    if (gate->inited == 0U)
    {
        /* 首次调用按“变化”处理，确保首个有效命令能生效一次 */
        gate->last_level = norm_level;
        gate->inited = 1U;
        return 1U;
    }

    if (gate->last_level != norm_level)
    {
        gate->last_level = norm_level;
        return 1U;
    }

    return 0U;
}
