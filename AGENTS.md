# AGENTS.md

## 硬规则

### .c/.h 文件编码
所有 `RC26_H7_R2/RC26_H7_R2/user/` 下 `.c/.h` 是 **GBK 编码**。严禁用内置 Edit/Write 工具直接写，必须用 **gbk-batch-edit** skill。内置工具会损坏中文注释。

### 改代码正确方式
```bash
python -c "..."  # 用 gbk-batch-edit skill 提供的 g() 函数做 replace
```
- old 必须精确匹配（含缩进、换行），不匹配打 MISS 不改文件
- 多行用 `\n`（Python 自动适应 `\r\n`）
- 改后必须读关键区域确认语法+注释对齐

---

## 项目概况

ROBOCON 2026 "武林探秘" R2 机器车。STM32H723VG (Cortex-M7)，Keil MDK (ARMCC v5)，FreeRTOS CMSIS-V2。

真项目根：`RC26_H7_R2/RC26_H7_R2/`（双重嵌套）

### 编译模式 (Keil -D 宏)
- `APP_MATCH_SKILL_Z12=1` — 技能赛 Z12
- `APP_MATCH_SKILL_Z3=1` — 技能赛 Z3
- 两者都不设 → 竞技赛 (`APP_MATCH_IS_ARENA=1`)
- `APP_ZONE2_RED_SIDE=1` — 红方，0=蓝方

### 任务调度
`Can_Task` 1ms 周期：
```
manual_chassis_function() → Motion_Task → zone poll
manual_weapon_function()  → ClampHeadCtrl_Run
manual_lift_function()
manual_kfs_function()
```

`Motion_Task` 派发顺序：zone2 > zone1 > zone3_prep > zone3 > flow_none

---

## 赛区架构

| 赛区 | 文件 | 核心机制 |
|------|------|----------|
| Zone1 | `app_zone1.c/h` | 扁平10状态，锚点单次触发，VW=0停车抓，ShouldAllowAutoGrab源头拦截 |
| Zone2 | `app_zone2.c/h` | 双层架构（调度层20态+执行层），梅花桩导航 |
| Zone3 | `app_zone3.c/h` | R1指令驱动（AA..BB/EE..FF帧），PutKFS/UpR1/GetKFS |
| Zone3Prep | `app_zone3_prep.c/h` | 上坡+预取KFS |
| 底层原语 | `Process_Flow.c/h` | UpStairs/DownStairs/GetKFS/PutKFS/UpR1/UpSlope + 底盘覆盖优先级 |

---

## 关键子系统

| 系统 | 文件 | 要点 |
|------|------|------|
| 夹爪 | `clamp_head_ctrl.c/h` | PE9 传感器高低电平 + 20ms去抖 + state机(idle→closing→upright_hold→dock_ok) |
| 导航 | `odom_nav_goto.c/h` | 远近双区PID，Vy+Vw输出，到达确认N帧 |
| 底盘 | `chassis.c/h` | 4×3508 Mecanum，VX/VY/VW 优先级覆盖(PROCESS_FLOW_OVERRIDE) |
| R1通信 | `r1_link*.c/h` | USART1+USART10双通道，AA..BB/CC..DD/EE..FF/55..AA四帧，3秒去重窗口 |
| KFS | `kfs.c/h` | *_position=目标位置，*_cmd=速度控制，flex模式自动斜坡 |
| Yaw转向 | `yaw_heading_ctrl.c/h` | 双环PID(外角内速) + RunFieldDir + ParallelLegSettle |

---

## 调试

Keil Watch 可实时看所有 `volatile` 的 `g_*_dbg` / `g_*_cfg` 结构体。

Zone1 锚点 mask: `g_app_zone1_dbg.anchor_triggered_mask` (bit0~5)
Zone1 调试跳过: `g_app_zone1_cfg.debug_skip_to_wait_r1 = 1` → 启动直达状态6等R1

---

## 已有技能

| 技能 | 用途 |
|------|------|
| gbk-batch-edit | 批量编辑GBK C/H文件 |
| gbk-pitfalls | GBK编辑常见错误 |
| gbk-refactor-pattern | 大块删除二进制兜底 |
| safe-edit | PowerShell安全编辑(旧方案) |
| worklog | 工作日记 |
