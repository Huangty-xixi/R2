---
name: gbk-batch-edit
description: |
  批量编辑 GBK/GB2312 编码的 C/H 文件。用一个 python -c 命令同时改多个文件，零临时文件，比逐行 gbk_edit 快 10 倍。
  TRIGGER: 用户说"改代码"、"改文件"、"修改"、"替换"、要批量编辑多个 GBK 文件，或在 E:\R2 下改 .c/.h 文件时。
  OVERRIDE: Keil 工程 C/H 文件为 GBK 编码，严禁使用内置 Edit/Write 工具（会损坏中文注释）。
---

# GBK Batch Edit

修改 `E:\R2\RC26_H7_R2\RC26_H7_R2\` 下 .c/.h 必须用本 skill，其他文件用内置 Edit/Write。

## 用法

```bash
python -c "
def g(path, old, new):
    r = open(path, 'r', encoding='gbk', errors='replace')
    c = r.read(); r.close()
    if old not in c:
        print(f'MISS: {path.split(chr(92))[-1]} — not found, SKIPPED')
        return
    c = c.replace(old, new)
    w = open(path, 'w', encoding='gbk')
    w.write(c); w.close()
    print(f'OK: {path.split(chr(92))[-1]}')

base = r'E:\R2\RC26_H7_R2\RC26_H7_R2'
g(f'{base}/user/src/xxx.c', 'old', 'new')
g(f'{base}/user/inc/xxx.h', 'old', 'new')
"
```

## 规则

1. 所有 g() 放同一个 python -c，一次改完
2. old 必须精确匹配（含缩进），不匹配打 MISS 跳过
3. 多行用 \n
4. 零临时文件
