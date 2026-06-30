---
name: gbk-batch-edit
description: |
  批量编辑 GBK/GB2312 编码的 C/H 文件。用一个 python -c 命令同时改多个文件，零临时文件，比逐行 gbk_edit 快 10 倍。
  TRIGGER: 用户说"改代码"、"改文件"、"修改"、"替换"、要批量编辑多个 GBK 文件，或在 E:\R2 下改 .c/.h 文件时。
  OVERRIDE: Keil 工程 C/H 文件为 GBK 编码，严禁使用内置 Edit/Write 工具（会损坏中文注释）。
---

# GBK Batch Edit

修改 `E:\R2\RC26_H7_R2\RC26_H7_R2\` 下 .c/.h 必须用本 skill，其他文件用内置 Edit/Write。

## 三步流程

### 第一步：repr 探针 —— 必须做，禁止跳过

改代码前，先用 `repr()` 拿到目标行的精确内容。不用 Read 工具（会显示乱码导致后续匹配失败）。

```bash
/c/Users/www/AppData/Local/Programs/Python/Python312/python.exe -c "
path = r'E:\R2\RC26_H7_R2\RC26_H7_R2\user\src\xxx.c'
r = open(path, 'r', encoding='gbk', errors='replace')
lines = r.readlines(); r.close()
for i in [N1, N2, N3]:  # 行号（1-indexed）
    print(f'{i}: {repr(lines[i-1])}')
"
```

### 第二步：按 repr 输出写 old，一次改完

把第一步 repr 输出的字符串（去掉外层单引号和 `\n`）直接粘进 `g()` 的 `old` 参数。多行用 `\n` 连接。

```bash
/c/Users/www/AppData/Local/Programs/Python/Python312/python.exe -c "
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
g(f'{base}/user/src/xxx.c', 'exact from repr', 'replacement')
g(f'{base}/user/inc/xxx.h', 'exact from repr', 'replacement')
"
```

### 第三步：验证改动

改完后用同样的 repr 探针确认改动区域。

## 规则

1. **禁止跳过第一步骤** —— 不用 repr 确认就直接写 old 必然 MISS
2. old 直接用 repr 输出，不要手打中文字符
3. 所有 g() 放同一个 python -c，一次改完
4. 多行用 `\n`
5. 零临时文件
6. 出现 MISS 立即用 `re.search` 重新探针修复，不要猜
