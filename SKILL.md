# E:\EK\main_R2 项目管家

## 项目概述

ROBOCON 2026 "武林探秘" R2 机器车。MCU: STM32H723VG (Cortex-M7)，编译器: Keil MDK-ARM ARMCC v5，RTOS: FreeRTOS CMSIS-V2。

真项目根为 `RC26_H7_R2\RC26_H7_R2\`（双重嵌套）。源文件在 `user\src\` (50个.c) 和 `user\inc\` (51个.h)。

### 编译模式 (Keil -D 宏)
- `APP_MATCH_SKILL_Z12=1` — 技能赛 Z12
- `APP_MATCH_SKILL_Z3=1` — 技能赛 Z3
- 两者都不设 → 竞技赛 (`APP_MATCH_IS_ARENA=1`)
- `APP_ZONE2_RED_SIDE=1` — 红方，0=蓝方

---

## 命名对照表（必读）

| 中文俗称 | 变量名 | 结构体/类型 | 电机 | 说明 |
|----------|--------|------------|------|------|
| 大风车 | `three_kfs` | `DM_MotorModule` | DM 电机 | 三轴吸盘旋转机构，位置 4 格 |
| 三轴 / 大风车位置 | `three_kfs_position` | `Three_kfs_position`（p1~p5） | — | 大风车当前槽位 |
| 前臂旋转 | `kfs_spin` | `DM_MotorModule` | DM 电机 | 前臂旋转，位置 p1~p4 |
| 前臂位置 | `kfs_spin_position` | `Kfs_spin_position`（p1~p4） | — | offset 可调（`g_kfs_spin_gain`） |
| 主轴 / 升降 | `main_lift` | `DM_MotorModule` | DM 电机 | 主轴抬升，位置 p0~p4 |
| 主轴位置 | `main_lift_position` | `Main_lift_position`（p0~p4） | — | p0=停止，p1~p4=各级高度 |
| 上伸缩 | `kfs_above` | KFS flex motor | DM 电机 | 上方伸缩机构，P1=缩回 P3=伸出 |
| 上伸缩位置 | `kfs_above_position` | `Kfs_Above_Cmd`（stop/p0~p3） | — | stop=0=速度模式，非0=位置模式 |
| 下伸缩 | `kfs_below` | KFS flex motor | DM 电机 | 下方伸缩机构 |
| 下伸缩位置 | `kfs_below_position` | `Kfs_Below_Cmd`（stop/p0~p3） | — | 同上 |
| 吸盘1~4 | `sucker1_state`~`sucker4_state` | `uint8_t` | — | 1=吸住/开，0=关 |

---

## 编码约定

### 强制规则
- **.c/.h 文件为 GBK/GB2312 编码**，中文注释以 GBK 存储
- **严禁使用内置 Edit/Write/NotebookEdit** 编辑任何 GBK 编码源文件——会永久损坏中文注释
- 编辑 .c/.h 唯一安全方式: **`mcp__safe-rw__safe_edit`** (MCP 工具)
  - 读取: `mcp__safe-rw__safe_read`
  - 搜索: `mcp__safe-rw__safe_search`
- 每次改 .c/.h 必须优先用 safe-rw MCP 工具

### 编码风格
- **ARM Compiler 5 (C90)**: 变量声明必须在块首（不能在代码中间声明变量）
- **中文注释**: 新增/修改代码时注释一律用中文，不改动现有注释内容
- 改完代码跑 `E:\EK\check_syntax.ps1` 静态检查

### 常用改代码流程
1. 先用 `safe_rw_safe_read` 读文件确认缩进和行尾
2. 用 `safe_rw_safe_edit` 做精确替换
3. 若匹配多处且不想全部替换，加更多上下文行定位唯一匹配
4. 改后跑 `check_syntax.ps1` 验证

---

## 关键子系统

| 赛区 | 文件 | 核心机制 |
|------|------|----------|
| Zone1 | `app_zone1.c/h` | 扁平10状态，锚点单次触发，VW=0停车抓 |
| Zone2 | `app_zone2.c/h` | 双层架构：调度层20态 + 执行层，梅花桩导航 |
| Zone3 | `app_zone3.c/h` | R1 指令驱动，PutKFS/UpR1/GetKFS/STOP |
| Zone3Prep | `app_zone3_prep.c/h` | 等R1上坡→上坡→导航G1→取KFS G1→导航G2→取KFS G2 |
| 底层原语 | `Process_Flow.c/h` | UpStairs/DownStairs/GetKFS/PutKFS/UpR1/UpSlope |

---

## 排障速查

### 上伸缩不动
1. 先查 `wait_extend_ms` —— 如果是 0U，extend 一帧就过，P3 命令没时间生效。设 2000U
2. 再查 `wait_sucker_close_ms` —— 如果改了 sucker_wait 立刻跳，extend 又是 0，P3 只活 2 帧
3. 查 `kfs.c:595` —— 切入位置模式第一帧 `flex_above_target = flex_pos0` 清零，第二帧才读到正确值（预存行为，不影响）
4. 查 `control_mode` 是否等于 `full_auto_control`

### 大风车位置连减
1. 先查 `s_nav_armed` —— 静态变量跨周期残留，第二把 KFS 不 arm 导航导致 Process_PutKFS 在 put_kfs 状态死循环
2. 查 `tail service` 是否在 `put_kfs` 状态下也调了 `Process_PutKFS`（双重调用重置 `static now_ms`）

### 三区流程卡住
1. 查 `g_nav_offset_dir` —— recenter 完成后没清零，下台阶吃到旧偏移
2. 查 `s_no_mission_tail_armed` —— zone2 没任务时自动上坡跑路
3. 查 zone1 是否调了 `R1Link_ClearNewMission()` —— 把 AA..BB 帧销毁了

### 一二区衔接断
1. 查 zone1 的 `R1Link_ClearNewMission()` 是否销毁了帧
2. 查 zone2 第一帧 `s_has_mission` 是否为 0
3. 查 R1 发包时序（两个 AA..BB 帧之间的间隔）

---

## 已解决的问题记录

### 2026-07-06: 下台阶导航偏移
- **问题**: `g_nav_offset_dir` 在 recenter 完成后残留，下台阶 recenter 吃到旧偏移
- **根因**: recenter substep done 时未清理
- **解决**: `z2_exec_nav_recenter_substep` done 分支中加 `g_nav_offset_dir = SKIP`
- **文件**: `app_zone2.c`
- **教训**: 导航偏移变量在 done/cleanup 处主动清零，不依赖下次 set_target

### 2026-07-06: kfs_below_position 刷新零点
- **问题**: `get_kfs_step_done` 设 `kfs_below_cmd_stop`，每次取完 KFS 重定零点
- **解决**: 删掉该行，保持位置模式
- **文件**: `Process_Flow.c`

### 2026-07-06: 取KFS上伸缩到位置一
- **问题**: 取KFS时 `kfs_above` 未设定位
- **解决**: `get_kfs_step_start` 加 `kfs_above_position = kfs_above_cmd_p1`
- **文件**: `Process_Flow.c`

### 2026-07-06: 三区预备上坡三轴改p1
- **问题**: 三区预备上坡 `three_kfs` 走 p4
- **解决**: `ProcessUpSlopeTune` 加 `three_kfs_pos` 字段，zone3_prep 传入 p1
- **文件**: `Process_Flow.h` / `Process_Flow.c` / `app_zone3_prep.c`

### 2026-07-06: 三区预备启动预置
- **问题**: 等R1上坡时主轴和伸缩未预设
- **解决**: `AppZone3Prep_Start` 设 `main_lift=p2, kfs_below/above=p1`
- **文件**: `app_zone3_prep.c`

### 2026-07-06: P3/P4两段式导航
- **问题**: P3/P4 放KFS直接导航到终点，精度不够
- **解决**: `nav_to_put_prep`(粗,0.08m/3次) → `nav_to_put_fine`(精,0.02m/60次)
- **文件**: `app_zone3.h` / `app_zone3.c`

### 2026-07-06: 枚举值漏加
- **问题**: 新状态未加入 enum 和 odom 丢失检测
- **解决**: 补充 enum + odom 检测列表
- **教训**: 加新状态值同步做三件事: enum、odom 列表、所有 switch-case

### 2026-07-06: s_nav_armed 导致上伸缩不伸 & 大风车连减
- **问题**: 第二次放KFS时 `s_nav_armed=1` 残留，导航不 arm → `Process_PutKFS` 在 `put_kfs` 状态死循环 → three_kfs 连减、kfs_above P3↔P1 震
- **根因**: `s_nav_armed` 是 static 变量，第一次放KFS设1后从不复位
- **解决**: 删 `s_nav_armed`，arm 条件改 `g_z3.state != return_point1`
- **文件**: `app_zone3.c`
- **教训**: **不要用 static 变量做状态标记**，用 `g_z3.state` 判断当前是否已 arm

### 2026-07-06: 尾段 service 双重调用 Process_PutKFS
- **问题**: 尾段在 `return_point1` 状态每帧调 Process_PutKFS，但 `put_kfs` 状态 run_put_kfs 也调——不同状态不会同时调，但尾段按步值判断有漏洞
- **根因**: 尾段用 `put_kfs_step == retract || done` 判断，步变了就死
- **解决**: 尾段放 `AppZone3_Run` 顶层，只用 `IsBusy()` 判断（学二区 `z2_get_kfs_tail_service`）
- **文件**: `app_zone3.c`

### 2026-07-06: 上伸缩不伸（wait_extend_ms=0U）
- **问题**: `wait_extend_ms = 0U` + sucker_wait 立刻跳 = P3 命令只活 2 帧，电机没时间动
- **根因**: extend 和 sucker_wait 的等待都没了
- **解决**: `wait_extend_ms = 2000U`
- **文件**: `Process_Flow.c`
- **教训**: 至少保证一个步骤有等待时间让电机收到命令并执行

### 2026-07-06: 三四区速度阈值未生效
- **问题**: `AppZone3Config` 加了 4 个 vmax 字段，但 `begin_nav_tol` 函数签名没改，调用处也没传
- **解决**: 函数签名 +2 参数，两处调用补传粗/精速
- **文件**: `app_zone3.h` / `app_zone3.c`

### 2026-07-06: 一二区衔接帧被zone1销毁
- **问题**: zone1 收到 AA..BB 帧后调 `R1Link_ClearNewMission()` 销毁了帧，zone2 捡不到
- **解决**: 删 zone1 的 `R1Link_ClearNewMission()`；删 zone2 的自动上坡安全出口块
- **文件**: `app_zone1.c` / `app_zone2.c`

---

## 改动文件清单

| 文件 | 本次改了什么 |
|------|-------------|
| `app_zone3.c` | 两段导航、尾段 service、run_put_kfs 删 s_nav_armed、速度参数、return_point1 gate |
| `app_zone3.h` | Config 加 prep 坐标/粗精 tol/vmax 字段 |
| `app_zone3_prep.c` | 启动预置 main_lift=p2/kfs=p1 |
| `app_zone2.c` | g_nav_offset_dir 清零、删安全出口块 |
| `app_zone1.c` | 删 R1Link_ClearNewMission() |
| `Process_Flow.c` | sucker_wait 立刻跳 retract、retract 不碰底盘、wait_extend_ms=2000U、get_kfs 上伸缩、upslope three_kfs_pos |
| `Process_Flow.h` | ProcessUpSlopeTune 加 three_kfs_pos、文件末补换行 |
| `kfs.c` | kfs_spin 加 p4（offset=0.1） |
| `kfs.h` | Kfs_spin_position 枚举 + KfsSpinGainCfg 加 p4 |

---

## 死代码（可清理）

| 文件 | 变量/函数 | 原因 |
|------|----------|------|
| `r1_link.c` | `R1Link_ClearNewMission()` | 零调用者，zone1 已不再消费帧 |
| `app_zone2.c` | `s_no_mission_tail_armed` | 只写不读，安全出口已删 |

---

## 参考文档

| 文档 | 路径 |
|------|------|
| 一二区衔接 | `RC26_H7_R2\RC26_H7_R2\赛区流程\一区衔接二区流程.html` |
| 二区KFS流程 | `RC26_H7_R2\RC26_H7_R2\赛区流程\二区KFS流程.html` |
| 二区V2导航优化 | `RC26_H7_R2\RC26_H7_R2\赛区流程\二区V2导航优化.html` |
| 三区流程 | `RC26_H7_R2\RC26_H7_R2\赛区流程\三区流程.html` |
| 放KFS流程 | `RC26_H7_R2\RC26_H7_R2\子流程\放KFS流程.html` |
| 取KFS流程 | `RC26_H7_R2\RC26_H7_R2\子流程\取KFS流程.html` |
| 上R1爬升流程 | `RC26_H7_R2\RC26_H7_R2\子流程\上R1爬升流程.html` |
| syntax检查脚本 | `E:\EK\check_syntax.ps1` |
