---
name: gbk-batch-edit
description: |
  批量编辑 GBK/GB2312 编码的 C/H 文件。用一个 python -c 命令同时改多个文件，零临时文件，比逐行 gbk_edit 快 10 倍。
  TRIGGER: 用户说"改代码"、"改文件"、"修改"、"替换"、要批量编辑多个 GBK 文件，或在 E:\EK 下改 .c/.h 文件时。
  OVERRIDE: 本项目的 C/H 文件为 GBK 编码，严禁使用内置 Edit/Write 工具（会损坏中文注释）。
---

# GBK Batch Edit — 批量改 GBK 文件

## 用法

一个 Bash 命令搞定全部修改，出问题立即停、不写坏数据：

```bash
python -c "
def g(path, old, new):
    r = open(path, 'r', encoding='gbk', errors='replace')
    c = r.read(); r.close()
    if old not in c:
        print(f'MISS: {path.split(chr(92))[-1]} — old not found, SKIPPED')
        return
    c = c.replace(old, new)
    w = open(path, 'w', encoding='gbk')
    w.write(c); w.close()
    print(f'OK: {path.split(chr(92))[-1]}')

base = r'E:\EK\main_R2\RC26_H7_R2\RC26_H7_R2'
g(f'{base}/user/inc/xxx.h', 'old', 'new')
g(f'{base}/user/src/xxx.c', 'old', 'new')
"
```

## 规则

1. **一个 Bash 一个 python -c** — 全部 g() 放同一个 -c 里，一次 Python 冷启动改完所有文件
2. **old 要精确** — 从 Read 工具直接复制，包括缩进、多行。匹配不到会打 MISS 跳过，保护文件不损坏
3. **多行用 \n** — 例如 `'line1\nline2\nline3'`
4. **无垃圾文件** — 不生成临时文件、不留 .bak
5. **严禁用内置 Edit/Write** — 会损坏 GBK 中文注释

## 和旧 gbk-edit 的区别

| | gbk-edit | gbk-batch-edit |
|------|---------|---------------|
| 改 4 个文件 | 15+ 次 Bash | 1 次 Bash |
| 临时文件 | 需要 | 不需要 |
| 行号漂移 | 有 | 无（用字符串匹配） |
| 合并行 bug | 有 | 无 |
| 适用场景 | 单行替换 | 批量修改 |
