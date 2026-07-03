---
name: gbk-pitfalls
description: |
  GBK .c/.h 二进制编辑的常见错误和预防。
  TRIGGER: 批量替换 GBK 文件中的 #define 宏、引脚定义、或结构体字段，涉及多处文件改动时。
author: Claude Code
version: 1.0.0
date: 2026-07-04
---

# GBK 编辑易错点

## 1. #define 自引用陷阱

替换宏值时如果目标名称和原有宏名相同，会造出自引用宏：

```c
// 错误：GPIO_PIN_10 替换 CLAMP_CTRL_Pin → 结果变成 #define GPIO_PIN_10 GPIO_PIN_10
// 这会覆盖 HAL 库的正确定义，连带破坏所有引用 GPIO_PIN_10 的文件
```

**规则：删除旧 #define 行，不要替换成新名称的 #define。** 如果 HAL 已有同名宏，直接用，不再定义。

## 2. 删除操作顺序

```python
# ❌ 错误顺序：先替换，后删除 → #define 行被污染
raw.replace(old_name, new_name)   # #define OLD → #define NEW
raw.replace(define_line, b'')     # 找不到 #define OLD 了！

# ✅ 正确顺序：先删 #define 行，再替换代码中的引用
raw.replace(define_line, b'')     # 先删掉 #define 行
raw.replace(old_name, new_name)   # 再替换代码中的引用
```

## 3. 孤立引用

删除 .h 中的宏后，必须全局搜所有引用该宏的文件。尤其 CubeMX 生成的 `main.h`、`gpio.c` 可能包含同一宏。

```python
# 改完必须：
grep -r "OLD_MACRO_NAME" user/src user/inc Core/Src Core/Inc
```

## 4. 批量编辑后逐步骤验证

```python
# 每步改完立即检查，不要攒到最后
print(f'Step1: {raw.count(b"new_pattern")} occurrences')  # 确认替换成功
print(f'Step2: {raw.count(b"old_pattern")} occurrences')  # 确认旧值清零
```

## 5. GBK 中文注释用实际 bytes

```python
# ❌ 自己猜的中文编码大概率不对
old = b'// \xb5\xbc\xba\xbd\xb7\xb5\xbb\xd8\xc2\xeb'  # 可能是错的

# ✅ 从文件读实际 bytes
with open(path, 'rb') as f:
    raw = f.read()
idx = raw.find(b'unique_ascii_nearby')
print(repr(raw[idx:idx+60]))  # 复制这一段
```

## 6. head -3 先确认再动手

批量替换大段代码前，先确认 old 字符串在文件中只出现一次（或按预期次数出现）：
```python
n = raw.count(old_string)
print(f'{n} occurrences')  # 期望值 ≠ 实际值 → 先不替换
```
