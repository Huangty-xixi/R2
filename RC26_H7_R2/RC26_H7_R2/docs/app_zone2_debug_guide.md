# 二区（app_zone2）调试指南

本文档整理二区梅花桩流程的**推荐调试步骤**（含调试假数据），与 `user/src/app_zone2.c`、`user/inc/app_zone2.h`、`user/src/Motion_Task.c`、`user/src/app_hook_init.c` 当前实现一致。

更完整的状态机与调度说明见 `app_zone2.h` 中 `@anchor app_zone2_scheduling` 注释块。

---

## 1. 调试前须满足的条件

### 1.1 遥控与模式（`Motion_Task.c`）

- **CH8** 置于中间档：**全自动**（`control_mode == full_auto_control`）。  
  - CH8 急停或遥控档会 `app_zone2_mission_clear()`，二区任务被清除。
- **切入二区独占**：在 `full_auto_mode == full_auto_none` 时，**CH6 最大（二区）** 与 CH5/CH7 等全自动请求**互斥**（四路中恰好一路有效）后，进入 `full_auto_zone2_mode`。
- **每周期执行 `app_zone2_poll()`**：须 **`full_auto_mode == full_auto_zone2_mode` 且 CH6 仍为最大**。  
  - CH6 **最小**：`app_zone2_mission_clear()` 并退出二区模式。  
  - CH6 **中位**：本周期不调 `poll`，相当于暂停、不 abort 任务。

### 1.2 钩子与半场

- 上电后须调用 **`AppHook_Init()`**（内部 `app_zone2_init_hooks(...)` 已绑定导航、上/下桩、摆头、取 KFS、航向忙闲）。
- **`APP_ZONE2_RED_SIDE`**（见 `app_hook_init.h` 或 Keil 预定义）须与当前半场、里程计/地图镜像一致。

---

## 2. 任务来源（与假数据的关系）

| 方式 | 说明 |
|------|------|
| R1 / 上层协议 | 解析任务后调用 **`app_zone2_mission_apply(&mission)`**。 |
| 调试假数据 | 定义 **`APP_ZONE2_DBG_FAKE_MISSION=1`**。内置 **path = 2,5,8,9,12**，**kfs = 4,6,11**；在无任务且允许自动装载时，于 **`app_zone2_poll()` 开头** 自动填充并 `apply`。 |
| 手写 | 填写 **`app_zone2_mission_t`**（`path_n` / `kfs_n` 与数组一致）后调用 **`app_zone2_mission_apply()`**。 |

**与 R1 的协调**：任意 **`app_zone2_mission_apply()`** 会清除「自动假数据」排队标志，避免显式真任务被下一拍假数据覆盖。**`app_zone2_mission_clear()`** 在开启调试宏时会重新允许下一轮自动套假数据（便于反复试）。

---

## 3. 推荐调试步骤（假数据）

按顺序操作即可在不上 R1 的情况下跑通二区状态机（路径与秘籍为固定假数据）。

1. **打开假数据宏**  
   在 Keil **Options → C/C++ → Define**（或等价预处理器符号）中增加：  
   **`APP_ZONE2_DBG_FAKE_MISSION=1`**  
   发布或联机 R1 真跑时请**关闭**该宏，避免无任务时误装载假路径。

2. **确认初始化**  
   保证启动流程中已调用 **`AppHook_Init()`**（二区钩子未注册则流程无法下发动作）。

3. **确认半场宏**  
   根据实际红/蓝区设置 **`APP_ZONE2_RED_SIDE`**，与场地及 ODOM 使用方式一致。

4. **上电后进入二区轮询**  
   - CH8：**全自动**。  
   - 在 **`full_auto_none`** 下，仅拉高 **CH6 最大**（二区），且不要同时触发 CH5 放/上坡、CH7 取 KFS 等其它独占请求。  
   - 进入 **`full_auto_zone2_mode`** 后，保持 **CH6 最大**，使 **`app_zone2_poll()`** 每周期被调用。

5. **等待自动 apply 假任务**  
   若当前**无任务**且调试宏允许自动装载（例如刚 `mission_clear` 后或上电首次满足条件），**第一个进入的 `app_zone2_poll()`** 会自动写入假 **path/kfs** 并 **`app_zone2_mission_apply()`**。  
   若已从 R1 或其它路径 **`apply` 过任务**，则不会再用假数据覆盖该次任务。

6. **观察状态**  
   可使用 **`app_zone2_is_busy()`**、**`app_zone2_is_done()`** 辅助判断。`Motion_Task` 中在 **`app_zone2_is_done()`** 为真时会将 **`full_auto_mode`** 置回 **`full_auto_none`**。

7. **反复调试**  
   需要重新从假数据起跑时：先 **CH6 最小** 或切遥控/急停等会触发 **`app_zone2_mission_clear()`** 的路径，再按步骤 4 重新切入二区；在 **`APP_ZONE2_DBG_FAKE_MISSION=1`** 下 **`mission_clear`** 会再次允许自动假数据装载。

8. **可选：手动取假数据结构**  
   在 **`APP_ZONE2_DBG_FAKE_MISSION=1`** 时可调用 **`app_zone2_debug_fake_mission_get(&m)`** 仅填充结构体，再自行决定何时 **`app_zone2_mission_apply(&m)`**。

---

## 4. 常见问题（简要）

- **有任务但机子不发令**：检查是否仍为全自动、`full_auto_mode` 是否为二区、CH6 是否最大；并对照 `app_zone2_motion_gate_ok`（仅 `full_auto_none` 或 `full_auto_zone2_mode` 时发钩子）。  
- **卡在导航等待**：检查 `nav_poll` / 里程计到点是否返回 **`ODOM_NAV_GOTO_ERR_OK_ARRIVED`**。  
- **层高与现场不符**：可在任务前调用 **`app_zone2_set_robot_tier()`** 与初始台阶档对齐。

---

*文档版本：与仓库内 app_zone2 / Motion_Task 实现同步维护。*
