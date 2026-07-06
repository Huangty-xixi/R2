# E:\EK\main_R2 项目管家

## 项目概述

ROBOCON 2026 "武林探秘" R2 机器车。MCU: STM32H723VG (Cortex-M7)，编译器: Keil MDK-ARM ARMCC v5，RTOS: FreeRTOS CMSIS-V2。

真项目根为 `RC26_H7_R2\RC26_H7_R2\`（双重嵌套）。源文件在 `user\src\` (50个.c) 和 `user\inc\` (51个.h)。

### 编译模式 (Keil -D 宏)
- `APP_MATCH_SKILL_Z12=1` — 技能赛 Z12
- `APP_MATCH_SKILL_Z3=1` — 技能赛 Z3
- 两者都不设 → 竞技赛 (`APP_MATCH_IS_ARENA=1`)
- `APP_ZONE2_RED_SIDE=1` — 红方，0=蓝方

### 关键子系统

| 赛区 | 文件 | 核心机制 |
|------|------|----------|
| Zone1 | `app_zone1.c/h` | 扁平10状态（0 idle→1 边导航边转90→2 倒退靠限位→3 右移搜料→4 夹爪等待→5 转180+前进→6 等R1→7 post_wait→8 done→9 abort→10 lap2），锚点单次触发，VW=0停车抓 |
| Zone2 | `app_zone2.c/h` | 双层架构：调度层20态 + 执行层，梅花桩导航，path/kfs 数组 |
| Zone3 | `app_zone3.c/h` | R1 指令驱动（AA..BB/EE..FF/55..AA 帧），PutKFS/UpR1/GetKFS/STOP |
| Zone3Prep | `app_zone3_prep.c/h` | 9状态：等R1上坡→上坡→导航G1→取KFS G1→导航G2→取KFS G2→done/failed |
| 底层原语 | `Process_Flow.c/h` | UpStairs/DownStairs/GetKFS/PutKFS/UpR1/UpSlope + 底盘覆盖优先级 |

| 系统 | 文件 | 要点 |
|------|------|------|
| 夹爪 | `clamp_head_ctrl.c/h` | PE9 传感器高低电平 + 20ms去抖 + state机(idle→closing→upright_hold→dock_ok) |
| 导航 | `odom_nav_goto.c/h` | 远近双区PID，Vy+Vw输出，到达确认N帧。`set_target`设点，`service_tick`统一周期推进，`peek_last_run_result`读结果 |
| 底盘 | `chassis.c/h` | 4x3508 Mecanum，VX/VY/VW 优先级覆盖(PROCESS_FLOW_OVERRIDE) |
| R1通信 | `r1_link*.c/h` | USART1+USART10双通道，AA..BB/CC..DD/EE..FF/55..AA四帧，3秒去重窗口 |
| KFS | `kfs.c/h` | main_lift(主轴)、kfs_spin(旋转)、three_kfs(三轴吸盘)、kfs_above(上伸缩)、kfs_below(下伸缩)。`*_position`=目标位置，`*_cmd`=速度控制 |
| Yaw转向 | `yaw_heading_ctrl.c/h` | 双环PID(外角内速) + RunFieldDir + ParallelLegSettle |

### 硬件
- MCU: STM32H723VG (Cortex-M7)
- 电机: DJI 3508 (Mecanum底盘) + DM电机(关节)
- 通信: CAN总线（CAN1/CAN2）、USART1+USART10（R1-R2红外通信）
- 传感器: BMI088 IMU、PE9夹爪传感器

### 任务调度
`Can_Task` 1ms 周期:
```
manual_chassis_function() → Motion_Task → zone poll
manual_weapon_function()  → ClampHeadCtrl_Run
manual_lift_function()
manual_kfs_function()
```
`Motion_Task` 派发顺序: zone2 > zone1 > zone3_prep > zone3 > flow_none

---

## 编码约定

### 强制规则
- **.c/.h 文件为 GBK/GB2312 编码**，中文注释以 GBK 存储
- **严禁使用内置 Edit/Write/NotebookEdit** 编辑任何 GBK 编码源文件——会永久损坏中文注释
- 编辑 .c/.h 唯一安全方式: **`mcp__safe-rw__safe_edit`** (MCP 工具)
  - 备选: `mcp__safe-rw__safe_write` (整文件覆写)
  - 读取: `mcp__safe-rw__safe_read` (显示 GBK 中文)
  - 搜索: `mcp__safe-rw__safe_search` (GBK 内容匹配)
  - 兜底: `/gbk-batch-edit` skill (MCP 不可用时)
- 每次改 .c/.h 必须优先用 safe-rw MCP 工具

### 编码风格
- **ARM Compiler 5 (C90)**: 变量声明必须在块首（不能在代码中间声明变量）
- **中文注释**: 新增/修改代码时注释一律用中文，不改动现有注释内容
- 编译器: Keil MDK ARMCC v5，工程文件为 `.uvprojx`，无 Makefile
- 本机无 ARM 编译器，改完代码后需在 Keil 环境 F7 验证

### 常用改代码流程
1. 先用 `safe_rw_safe_read` 读文件确认缩进和行尾
2. 用 `safe_rw_safe_edit` 做精确替换（old_string 必须含缩进和中文注释，精确匹配）
3. 若匹配多处且不想全部替换，加更多上下文行定位唯一匹配
4. 改后读关键区域确认语法+注释对齐

---

## 调试与在线调参

- Keil Watch 可实时看所有 `volatile` 的 `g_*_dbg` / `g_*_cfg` 结构体
- Zone1 锚点 mask: `g_app_zone1_dbg.anchor_triggered_mask` (bit0~5)
- Zone1 调试跳过: `g_app_zone1_cfg.debug_skip_to_wait_r1 = 1` → 启动直达状态6等R1
- GetKFS 等待时间: `g_process_get_kfs_tune` (ms级在线调参)
- PutKFS 等待时间: `g_process_put_kfs_tune`
- 上坡: `g_process_upslope_tune`
- 上R1: `g_process_up_r1_tune` / `up_r1_step`

---

## 已解决的问题记录

### 2026-07-06: 下台阶导航偏移
- **问题**: `g_nav_offset_dir` 在 recenter 完成后残留，导致下台阶时 recenter 吃到旧偏移量
- **根因**: recenter substep done 时未清理 offset_dir，下台阶流程复用该变量产生错误偏移
- **解决**: 在 `z2_exec_nav_recenter_substep` done 分支中清零 `g_nav_offset_dir`
- **文件**: `app_zone2.c`
- **教训**: 导航偏移量变量要在每个使用场景的 done/cleanup 处主动清零，不要依赖下次 set_target 覆盖

### 2026-07-06: kfs_below_position 刷新零点
- **问题**: `get_kfs_step_done` 中设置了 `kfs_below_cmd_stop`，导致每次完成取KFS后下伸缩都重定零点
- **根因**: stop命令使下伸缩丢失位置记忆，下次取KFS时无法从当前位置出发
- **解决**: 删除 `get_kfs_step_done` 中的 `kfs_below_cmd_stop` 行，保持位置模式持续跟踪
- **文件**: `Process_Flow.c`
- **教训**: 伸缩机构完成动作后应保持位置模式而非 stop，否则零点丢失

### 2026-07-06: 取KFS流程上伸缩到位置一
- **问题**: 取KFS时上伸缩(`kfs_above`)未设定位，导致上伸缩处于不确定位置
- **解决**: 在 `get_kfs_step_start` 中增加 `kfs_above_position = kfs_above_cmd_p1`
- **文件**: `Process_Flow.c`
- **教训**: 取KFS流程启动时必须同时设置上下伸缩的目标位置

### 2026-07-06: 三区预备上坡三轴改p1
- **问题**: 三区预备上坡时 `three_kfs` 走了 p4 角度，与实际需求不符
- **解决**: 在 `ProcessUpSlopeTune` 结构体中新增 `three_kfs_pos` 字段，`zone3_prep` 调用上坡时传入 p1
- **文件**: `Process_Flow.h` / `Process_Flow.c` / `app_zone3_prep.c`
- **教训**: 上坡时需要三轴吸盘回到安全位置(p1)，避免碰撞

### 2026-07-06: 三区预备启动预置位置
- **问题**: 等R1上坡时，主轴(main_lift)和上下伸缩未预设到安全位置
- **解决**: `AppZone3Prep_Start` 中增加 `main_lift = p2`, `kfs_below = p1`, `kfs_above = p1`
- **文件**: `app_zone3_prep.c`
- **教训**: 每个赛区/子流程的 Start 函数应明确设置所有执行机构的初始位置

### 2026-07-06: P3/P4两段式导航
- **问题**: P3/P4 放KFS时直接导航到终点，一次性到位精度不够
- **根因**: 远距离导航累积误差大，需要粗定位+精定位两步走
- **解决**: 新增两个状态: `nav_to_put_prep` (粗定位到预备点) + `nav_to_put_fine` (精定位到终点)
- **文件**: `app_zone3.c`
- **教训**: 高精度放置操作应分两段导航——粗调接近目标，精调完成对准

### 2026-07-06: 枚举值漏加
- **问题**: 新增状态 `nav_to_put_prep` / `nav_to_put_fine` 后，忘记加入枚举定义和 odom 丢失检测列表
- **解决**: 在 state enum 中补充这两个枚举值，并在 odom 丢失检测的 switch-case 中增加对应分支
- **教训**: **加新状态值要同步做三件事**: (1) enum 定义 (2) odom 丢失/超时检测列表 (3) 所有 switch-case 覆盖

---

## 待解决问题

- [ ] 地面取KFS时机未校验（下层取KFS高度计算待完善）
- [ ] 三区放KFS位置精度待验证（P3/P4两段式导航需实测验证）
- [ ] kfs伸缩位置参数需根据机械标定微调
- [ ] (待补充)

---

## 参考文档

### 项目内文档
| 文档 | 路径 |
|------|------|
| 一区流程 | `RC26_H7_R2\RC26_H7_R2\赛区流程\一区流程.html` |
| 一区调参指南 | `RC26_H7_R2\RC26_H7_R2\赛区流程\一区调参指南.html` |
| 二区KFS流程 | `RC26_H7_R2\RC26_H7_R2\赛区流程\二区KFS流程.html` |
| 二区V2导航优化 | `RC26_H7_R2\RC26_H7_R2\赛区流程\二区V2导航优化.html` |
| 三区流程 | `RC26_H7_R2\RC26_H7_R2\赛区流程\三区流程.html` |
| 放KFS流程 | `RC26_H7_R2\RC26_H7_R2\子流程\放KFS流程.html` |
| 取KFS流程 | `RC26_H7_R2\RC26_H7_R2\子流程\取KFS流程.html` |
| 上R1爬升流程 | `RC26_H7_R2\RC26_H7_R2\子流程\上R1爬升流程.html` |
| 下台阶流程 | `RC26_H7_R2\RC26_H7_R2\子流程\下台阶流程.html` |
| 导航函数参数清单 | `RC26_H7_R2\RC26_H7_R2\调试报告\导航函数参数清单.html` |
| 伸缩位置调用汇总 | `RC26_H7_R2\RC26_H7_R2\调试报告\伸缩位置调用汇总.html` |
| 上坡调试报告 | `RC26_H7_R2\RC26_H7_R2\调试报告\上坡调试报告.html` |
| Zone2调试指南 | `RC26_H7_R2\RC26_H7_R2\app_zone2_debug_guide.md` |
| 项目根CLAUE | `CLAUDE.md` |
| 项目根AGENTS | `AGENTS.md` |
| 工作日记 | `工作日记.md` |

### 已有技能
| 技能 | 用途 |
|------|------|
| gbk-batch-edit | 批量编辑GBK C/H文件 |
| gbk-edit | 单文件GBK编辑 |
| safe-rw (MCP) | GBK安全读写编辑 |
