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
/c/Users/Administrator/AppData/Local/Programs/Python/Python311/python -c "
import os
def g(path, old, new):
    r = open(path, 'r', encoding='gbk', errors='replace')
    c = r.read(); r.close()
    if old not in c:
        print(f'MISS: {os.path.basename(path)}')
        return
    c = c.replace(old, new)
    w = open(path, 'w', encoding='gbk')
    w.write(c); w.close()
    print(f'OK: {os.path.basename(path)}')

base = r'E:\R2\RC26_H7_R2\RC26_H7_R2'
g(f'{base}/user/src/xxx.c', 'old', 'new')
g(f'{base}/user/inc/xxx.h', 'old', 'new')
"
```

## 规则

1. 所有 g() 放同一个 python -c，一次改完
2. old 必须精确匹配（含缩进、换行），不匹配打 MISS 跳过
3. 多行用 \n
4. 零临时文件、不产生 .bak、不生成 .py 脚本
5. old 字符串只用 ASCII 字符——GBK 中文会被 errors='replace' 替换成乱码，无法匹配

## Python 路径

本机 Python 在 `C:\Users\Administrator\AppData\Local\Programs\Python\Python311\python.exe`，Bash 路径为 `/c/Users/Administrator/AppData/Local/Programs/Python/Python311/python`。

Git Bash 自带的 `python`/`python3` 是 Windows Store 占位 stub（exit 49），不可用。
