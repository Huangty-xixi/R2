# 硬规则

## 改 .c/.h 代码

必须用 **gbk-batch-edit** skill。Keil 工程源文件为 GBK 编码，内置 Edit/Write 会损坏中文注释。

用法：
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
"
```

规则：
1. 多行 old/new 用 `\n`（Python 字面量，会自动匹配文件里的 `\r\n`）
2. old 必须精确匹配含缩进
3. 不匹配打 MISS，不改文件
4. 改前不确定缩进/行尾就先读文件确认
