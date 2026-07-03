---
name: gbk-refactor-pattern
description: |
  GBK C/H 文件删除整个类型/函数（含中文注释）的二进制模式。TRIGGER: gbk-batch-edit g() 连续 MISS 2 次以上，或需删除 typedef struct/enum + 中文注释。
author: Claude Code
version: 1.0.0
date: 2026-07-03
---

# GBK 大规模删除模式

gbk-batch-edit 的补充：当文本 `g()` 因中文注释反复 MISS 时用。

## 为什么 g() 会 MISS

`\xbc` 在 Python str 中是 Unicode U+00BC，不是 GBK byte 0xBC。所以 old 字符串里的 hex escape 永远匹配不到 GBK 文件内容。解决：要么用实际中文字符，要么切二进制模式。

## 三招

### 1. 行过滤（删已知行）

```python
with open(path, 'rb') as f: raw = f.read()
lines = raw.split(b'\r\n')
kept = [l for l in lines if not any(kw in l for kw in ascii_keywords)]
raw = bytearray(b'\r\n'.join(kept))
```

### 2. 括号匹配（删函数体）

行过滤后函数签名行被删但 body 行残留（不含函数名关键词），用 `{` `}` 计数器找完整范围：

```python
p = text.find(b'func_signature_marker')
ob = text.find(b'{', p)
depth, i = 1, ob + 1
while i < len(text) and depth > 0:
    if text[i:i+1] == b'{': depth += 1
    elif text[i:i+1] == b'}': depth -= 1
    i += 1
le = text.find(b'\n', i) + 1
# 删 [line_start_of_signature, le]
```

### 3. 逆序块删除（防位移）

```python
ranges.sort(key=lambda r: r[0], reverse=True)
for start, end, desc in ranges:
    del raw[start:end]
```

## 铁律

- old 锚点只用 ASCII bytes，绝不含 `//` 后的内容
- 二进制模式行尾是 `\r\n`，不是 `\n`
- 改 struct 字段后检查 initializer 数量（多了会 warning）
- 每一步后 grep 残留
