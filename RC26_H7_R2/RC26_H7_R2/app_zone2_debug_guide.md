# 二区（app_zone2）调试指南

本文档整理二区梅花桩流程的**推荐调试步骤**（含调试假数据），与 `user/src/app_zone2.c`、`user/inc/app_zone2.h`、`user/src/Motion_Task.c`、`user/src/app_init.c` 当前实现一致。

更完整的状态机与调度说明见 `app_zone2.h` 中 `@anchor app_zone2_scheduling` 注释块。

---

## 1. 调试前须满足的条件

### 1.1 遥控与模式（`Motion_Task.c`）

- **CH8** 置于中间档：**全自动**（`control_mode == full_auto_control`）。  
  - CH8 急停或遥控档会 `app_zone2_mission_clear()`，二区任务被清除。
- **切入二区（App 层）**：在 **`full_auto_mode == full_auto_none`** 且 **`app_flow_mode == app_flow_none`** 时，**CH6 最大（二区）** 与 CH5/CH7 等请求**互斥**（四路中恰好一路有效）后，置 **`app_flow_mode = app_flow_zone2`**（**不再**占用 `full_auto_zone2_mode`；`full_auto_mode` 仍留给 CH5 上台阶 / CH7 取 KFS 等调试流程）。
- **每周期执行 `app_zone2_poll()`**：须 **`app_flow_mode == app_flow_zone2` 且 CH6 仍为最大**（兼容旧工程若仍写入了 **`full_auto_zone2_mode`**，二者任一成立且 CH6 最大时也会 poll）。  
  - CH6 **最小**：`app_zone2_mission_clear()` 并 **`app_flow_mode = app_flow_none`**（并兼容清除 legacy **`full_auto_zone2_mode`**）。  
  - CH6 **中位**：本周期不调 `poll`，相当于暂停、不 abort 任务；且 **`full_auto_mode == full_auto_none` 且 `app_flow_mode != app_flow_none`** 时**不会**误进「互斥选路」分支。

### 1.2 钩子与半场

- 上电后须调用 **`App_Init()`**；当前实现不再走 `app_zone2_init_hooks(...)`，二区执行层直接调用 `odom_nav_goto_*`、`Process_*` 与 `YawHeadingCtrl_*`。
- **`APP_ZONE2_RED_SIDE`**（见 `user/inc/app_init.h` 或 Keil 预定义）须与当前半场、里程计/地图镜像一致。

---

## 2. 任务来源（与假数据的关系）

| 方式 | 说明 |
|------|------|
| R1 / 上层协议 | 解析任务后调用 **`app_zone2_mission_apply(&mission)`**。（真数据，非下表假数据） |
| **调试假数据** | 定义 **`APP_ZONE2_DBG_FAKE_MISSION=1`**。桩号内容以 **§2.1 假数据对照表** 为准；在无任务且允许自动装载时，于 **`app_zone2_poll()` 开头** 自动填充并 `apply`。 |
| 手写 | 填写 **`app_zone2_mission_t`** 后 **`app_zone2_mission_apply()`**；若自行填与下表相同桩号，即等价于手动套用假数据。 |

### 2.1 调试专用假数据对照表（`APP_ZONE2_DBG_FAKE_MISSION`）

以下与源码 **`app_zone2_debug_fake_mission_get()`** / `app_zone2.c` 中赋值一致，均为**示意图桩号**（蓝区 1..12 与 MF 逐行一致；红区见 `app_zone2.h` 总图说明）。

| 字段 | **假数据**取值 | 含义 |
|------|----------------|------|
| `path_n` | **5** | 路径桩个数 |
| `path[]` | **2 → 5 → 8 → 9 → 12** | 路径顺序（`path[0]` 为首桩） |
| `kfs_n` | **3** | 秘籍桩个数 |
| `kfs[]` | **4，6，11** | 秘籍所在桩号（与 `path[]` 邻格关系由状态机按图判定） |

**与 R1 的协调**：任意 **`app_zone2_mission_apply()`** 会清除「自动假数据」排队标志，避免显式真任务被下一拍假数据覆盖。**`app_zone2_mission_clear()`** 在开启调试宏时会重新允许下一轮自动套假数据（便于反复试）。

---

## 3. 推荐调试步骤（假数据）

按顺序操作即可在不上 R1 的情况下跑通二区状态机；路径与秘籍桩号固定为 **§2.1 假数据对照表** 所列。

1. **打开假数据宏**  
   在 Keil **Options → C/C++ → Define**（或等价预处理器符号）中增加：  
   **`APP_ZONE2_DBG_FAKE_MISSION=1`**  
   发布或联机 R1 真跑时请**关闭**该宏，避免无任务时误装载假路径。

2. **确认初始化**  
   保证启动流程中已调用 **`App_Init()`**；二区执行层直接调用导航、流程与航向模块，不再依赖 hook 注册。

3. **确认半场宏**  
   根据实际红/蓝区设置 **`APP_ZONE2_RED_SIDE`**，与场地及 ODOM 使用方式一致。

4. **上电后进入二区轮询**  
   - CH8：**全自动**。  
   - 在 **`full_auto_none`** 下，仅拉高 **CH6 最大**（二区），且不要同时触发 CH5 放/上坡、CH7 取 KFS 等其它独占请求。  
   - 进入 **`full_auto_zone2_mode`** 后，保持 **CH6 最大**，使 **`app_zone2_poll()`** 每周期被调用。

5. **等待自动 apply 假任务**  
   若当前**无任务**且调试宏允许自动装载（例如刚 `mission_clear` 后或上电首次满足条件），**第一个进入的 `app_zone2_poll()`** 会自动写入 **§2.1** 所列假 **`path[]` / `kfs[]`** 并 **`app_zone2_mission_apply()`**。  
   若已从 R1 或其它路径 **`apply` 过任务**，则不会再用假数据覆盖该次任务。

6. **观察状态**  
   可使用 **`app_zone2_is_busy()`**、**`app_zone2_is_done()`** 辅助判断。`Motion_Task` 中在 **`app_zone2_is_done()`** 为真时会将 **`full_auto_mode`** 置回 **`full_auto_none`**。

7. **反复调试**  
   需要重新从假数据起跑时：先 **CH6 最小** 或切遥控/急停等会触发 **`app_zone2_mission_clear()`** 的路径，再按步骤 4 重新切入二区；在 **`APP_ZONE2_DBG_FAKE_MISSION=1`** 下 **`mission_clear`** 会再次允许自动假数据装载。

8. **可选：手动取假数据结构**  
   在 **`APP_ZONE2_DBG_FAKE_MISSION=1`** 时可调用 **`app_zone2_debug_fake_mission_get(&m)`** 仅填充结构体，再自行决定何时 **`app_zone2_mission_apply(&m)`**。

---

## 4. 假数据任务下的二区自动流程（机内状态顺序）

以下按 **§2.1** 假数据 **path = 2→5→8→9→12**、**kfs = 4，6，11** 描述 `app_zone2_poll()` 状态机的大致顺序（桩号为示意图；蓝区与 MF 格一致，红区经 `user_pile_to_mf` 映射后逻辑相同）。**实际发令**仍须满足全自动档、**`app_flow_zone2`（或 legacy `full_auto_zone2_mode`）**、**CH6 最大** 及 `app_zone2_motion_gate_ok`（见 §1、§5）。

### 4.1 上任务（`app_zone2_mission_apply`）

- 检查 **`path[0]=2`** 上是否存在「`kfs[j]==2` 且未取完」：**无**（秘籍在 4、6、11）。  
- 因此**不**走「一区台面先清 `path[0]` 秘籍」分支，进入 **`Z2_ENTER_UP`**，并要先上桩把层高对齐到 **2 号桩顶档（200mm → tier 0）**，再上梅林、去 **2** 号桩心。

### 4.2 桩 2：上台面 → 导航到点 → 无邻格秘籍则换桩

1. **`Z2_ENTER_UP`**：上桩节拍完成后 **`s_robot_tier`** 对齐 **2** 的层档。  
2. **`Z2_ENTER_NAV` → `Z2_ENTER_WAIT_NAV`**：对 **2** 号梅林中心下导航，**`nav_poll` 到点**后进入 **`Z2_KFS_TURN`**。  
3. 在站立桩 **2** 上查找**相邻**且未完成的 `kfs`：**4、6、11** 与 **2** 均**不构成四邻**（机内 `piles_adjacent`），**不**发取秘籍。  
4. 判定「当前桩无未完邻格秘籍」→ **`path_idx++`**，下一 path 桩为 **5**，进入 **`Z2_PATH_NEXT_PILE`**（摆头 + 按层高差逐档上/下桩，再导航去 **5**）。

**小结**：**2 号仅作首桩入口（上台 + 到点），本假数据下不在 2 上取 4/6/11。**

### 4.3 桩 5：邻格取 **4**、再取 **6**

- **5** 与 **4**、**6** 相邻；按 `kfs[]` 下标顺序，先 **4** 后 **6**。  
- 每本秘籍：**`Z2_KFS_TURN`**（`field_dir_between_user_piles(5, 秘籍桩)` 摆头）→ **`Z2_KFS_RUN`**（`request_get_kfs(rel)`，`rel` 由 **5 号顶高** 与 **秘籍桩顶高** 比较得到高→低 / 低→高）。  
- 两本完成后，**11** 与 **5** 不相邻 → **`path_idx++`** → **`Z2_PATH_NEXT_PILE`** 去 **8**。

### 4.4 桩 8：邻格取 **11**

- **8** 与 **11** 相邻；**4、6** 已取完。  
- **`Z2_KFS_TURN` / `Z2_KFS_RUN`** 取 **11** 后，**8** 上无未完邻格 kfs → **`path_idx++`** → 换桩去 **9**。

### 4.5 桩 9：过渡（本假数据下不再取秘籍）

- **9** 与未取完的 **4、6、11** 无「邻格且未完成」组合（秘籍已全部取完）→ **`path_idx++`** → 换桩去 **12**。

### 4.6 末桩 12：path 走完 + 末桩 200mm 下地面序列

- 导航到 **12** 桩心后，若无未完邻格秘籍，**path** 已走完。  
- **12** 在高度表里为 **200mm**，且属于规则中的末桩 **10 / 12 / 6** 之一 → **`Z2_LAST_DOWN_TURN`**：**10 / 12** 为 **朝场后（`BACK`）**；**仅 6 号**末桩在红/蓝侧分别为 **左 / 右**（见源码 `Z2_LAST_DOWN_TURN`）→ **`Z2_LAST_DOWN_DISMOUNT`**（一次下地面）→ **`Z2_DONE`**。

若末桩不是 **10/12/6** 或不是 **200mm**，则 path 结束后**不**走上述「末桩下地面」子序列，而直接 **`Z2_DONE`**（见 `app_zone2.h` 调度说明）。

### 4.7 一句话串流程

**2** 只上台到点不占秘籍 → **5** 取 **4**、**6** → **8** 取 **11** → **9** 过渡 → **12** 为 **200mm** 末桩并走「摆头 + 一次下地面」收尾 → **完成**。

---

## 5. 常见问题（简要）

- **有任务但机子不发令**：检查是否仍为全自动、**`app_flow_mode == app_flow_zone2`**（或 legacy **`full_auto_zone2_mode`**）、CH6 是否最大；并对照 `app_zone2_motion_gate_ok`（`app_flow_zone2` 或 `full_auto_none`/`full_auto_zone2_mode` 组合，见 `app_zone2.h`）。
- **卡在导航等待**：检查 `nav_poll` / 里程计到点是否返回 **`ODOM_NAV_GOTO_ERR_OK_ARRIVED`**。  
- **层高与现场不符**：可在任务前调用 **`app_zone2_set_robot_tier()`** 与初始台阶档对齐。

---

*文档版本：与仓库内 app_zone2 / Motion_Task 实现同步维护。*

**附**：上梅林之前得加偏置（与假数据表无关，属现场/部署说明）。
