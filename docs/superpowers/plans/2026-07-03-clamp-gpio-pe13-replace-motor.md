# PE13 GPIO 直驱夹爪替换 CAN 电机 — 方案 A 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans

**Goal:** 用提交 b8e08ee 的纯时序夹枪头状态机替换当前 CAN 电机驱动的夹爪，PE13 GPIO 直驱

**Architecture:** 删除 weapon_clamp_motor (DJI 2006 CAN2 0x1FF) 全部代码，简化 clamp_head_ctrl 状态机为 4 态（idle/wait_close_delay/upright_hold/dock_ok），clamp_use() 改为 HAL_GPIO_WritePin(PE13)

**Global Constraints:**
- 所有 .c/.h 为 GBK 编码，必须用 gbk-batch-edit skill（python -c 内联编辑）
- CubeMX/ioc 需手动在 STM32CubeMX 中操作

---

## 改动文件清单（9 个文件）

| # | 文件 | 操作 | 说明 |
|---|------|------|------|
| 1 | `user/src/clamp_head_ctrl.c` | 修改 | 删 closing/opening 状态，删 motor 反馈，简化状态机 |
| 2 | `user/inc/clamp_head_ctrl.h` | 修改 | 删 ReachedCloseLimit 声明，删 motor dbg 字段 |
| 3 | `user/src/weapon.c` | 修改 | 删 ~200行 Weapon_ClampMotor_*，clamp_use() 改 PE13 GPIO |
| 4 | `user/inc/weapon.h` | 修改 | 删 motor 类型/声明，删 PublishClamp 声明 |
| 5 | `user/src/motor.c` | 修改 | 删 overtemp list 中 weapon_clamp_motor |
| 6 | `user/src/register.c` | 修改 | 删 DJImotor_Create weapon_clamp_motor |
| 7 | `user/src/bsp_can.c` | 修改 | 删 CAN 反馈 weapon_clamp_motor |
| 8 | `user/src/Can_Task.c` | 修改 | 删 3 处 PublishClampZero |
| 9 | `user/src/app_zone1.c` | 修改 | 删 Weapon_ClampMotor_IsBusy 引用 |
