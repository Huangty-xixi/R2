---
name: gbk-batch-edit
description: |
  批量编辑 GBK/GB2312 编码的 C/H 文件。用一个 python -c 命令同时改多个文件，零临时文件，比逐行 gbk_edit 快 10 倍。
  TRIGGER: 用户说"改代码"、"改文件"、"修改"、"替换"、要批量编辑多个 GBK 文件，或在 E:\R2 下改 .c/.h 文件时。
  OVERRIDE: Keil 工程 C/H 文件为 GBK 编码，严禁使用内置 Edit/Write 工具（会损坏中文注释）。
---

# GBK Batch Edit

修改 `E:\R2\RC26_H7_R2\RC26_H7_R2\` 下 .c/.h 必须用本 skill，其他文件用内置 Edit/Write。

## 执行流程（严格按顺序，禁止跳步或插入额外检查）

### Step 0: 定位 Python（1 次调用）

```bash
where python
```

**取第一条非 WindowsApps 的路径**，转为 Bash 格式（`C:\Users\xxx\...` → `/c/Users/xxx/...`）。后续所有步骤用这个绝对路径。

⚠️ **禁止**：`python3`（Windows Store 占位 stub → exit 49）
⚠️ **禁止**：信任 skill 里写的硬编码路径（用户名和 Python 版本每台机器不同）
⚠️ **禁止**：用不带路径的 `python`（Git Bash 可能解析到 Windows Store stub）

### Step 1: 确认缩进 + GBK 健康度（1 次调用，覆盖所有目标文件）

```bash
<python> -c "
import os
base = r'E:\R2\RC26_H7_R2\RC26_H7_R2'
files = [
    f'{base}/user/src/xxx.c',
    f'{base}/user/inc/xxx.h',
]
for path in files:
    # 缩进：取第一个非空行的首字节
    with open(path, 'rb') as f:
        for line in f:
            if line[0:1] not in (b'\r', b'\n'):
                t = 'SPACE' if line[0]==0x20 else 'TAB' if line[0]==0x09 else 'OTHER'
                print(f'{os.path.basename(path)} indent=0x{line[0]:02x} ({t})')
                break
    # GBK 健康度：用 � escape（纯 ASCII，免疫 shell 编码），避免字面量自身被损坏导致永远报 CLEAN
    with open(path, 'r', encoding='gbk', errors='replace') as f:
        c = f.read()
    bad = c.count('�')  # Python escape，纯 ASCII，不会被 shell 损坏
    print(f'{os.path.basename(path)} GBK: {bad} bad chars', 'CLEAN' if bad==0 else 'CORRUPTED!')
"
```

一次命令确认：每个文件的缩进类型 + GBK 是否有坏字节。

### Step 2: 构造 old 字符串

- **缩进**：Step 1 确认是空格还是 tab，直接写对应字符（4 个空格 or `\t`）
- **中文**：Step 1 显示 `CLEAN` 就**直接含中文**；`CORRUPTED` 则用 ASCII 跨行锚点（见下方技巧 B）
- **换行**：用 `\n`（Python 文本模式自动匹配文件的 `\r\n`）
- **引号**：old/new 含单引号 `'` 时，Python 字符串外层改用双引号 `"`；两者都含则用三引号 `'''`
- **多行 old**：`\n` 连接，3 行以内最稳

### Step 3: 一次性执行所有编辑

```bash
<python> -c "
import os
ok = miss = 0
def g(path, old, new):
    global ok, miss
    r = open(path, 'r', encoding='gbk', errors='replace')
    c = r.read(); r.close()
    if old not in c:
        print(f'MISS [{ok+miss+1}]: {os.path.basename(path)}')
        miss += 1
        return
    c = c.replace(old, new)
    w = open(path, 'w', encoding='gbk')
    w.write(c); w.close()
    print(f'OK   [{ok+miss+1}]: {os.path.basename(path)}')
    ok += 1

base = r'E:\R2\RC26_H7_R2\RC26_H7_R2'
g(f'{base}/user/src/xxx.c', 'old', 'new')
g(f'{base}/user/inc/xxx.h', 'old', 'new')

print(f'--- {ok} OK, {miss} MISS ---')
"
```

序号 `[N]` 帮助定位：同一文件多次 g() 时，MISS 对应第几个调用一目了然。

### Step 4: 自检（1-2 次调用）

- grep 确认旧字段消失
- grep 确认保留字段还在
- Read 关键区域确认语法完整

## 中文行删除技巧（按优先级）

**A. 直接含中文（首选）**：old 包含中文原文，大多数环境 GBK→Unicode←UTF8 一致。
```python
g(path, '    .field = 100U, // 中文注释\n', '')
```

**B. ASCII 跨行锚点**：中文行夹在两条 ASCII 行之间时，old 跨三行，两端 ASCII 锁定位置，中间中文无论是否匹配都被整段替换。
```python
g(path,
    'ASCII_line_before\nChinese_line_to_delete\nASCII_line_after',
    'ASCII_line_before\nASCII_line_after')
```

**C. 大括号匹配**：用 `{` `}` 计数器定位函数/结构体，整块替换（见 [[gbk-edit-chinese-anchors]]）。

**D. 写 .py 文件执行**：以上都失败时才用 Write 写 UTF-8 .py 脚本再 Bash 执行。

## 文本模式 MISS 后的二进制兜底

当 g() 返回 MISS（通常是中文损坏导致），**不要反复尝试不同中文编码**——直接用二进制模式，一次到位。

### 二进制删除（删整行）

```bash
<python> -c "
path = r'<full_path>'
with open(path, 'rb') as f:
    raw = f.read()

# ASCII-only anchor，绝不含 // 后的中文
anchor = b'UNIQUE_ASCII_HOOK'
pos = raw.find(anchor)
if pos < 0: print('MISS'); exit(1)

# 定位行首和行尾（含 \r\n）
ls = raw.rfind(b'\n', 0, pos) + 1  # 行首
le = raw.find(b'\n', pos)            # \n 位置
if le > 0 and raw[le-1:le] == b'\r': le -= 1  # 回到 \r

new_raw = raw[:ls] + raw[le+2:]  # 跳过 \r\n
with open(path, 'wb') as f: f.write(new_raw)
print('OK')
"
```

### 二进制插入（在指定行后加新行）

```bash
<python> -c "
path = r'<full_path>'
with open(path, 'rb') as f:
    raw = f.read()

# ASCII-only anchor
anchor = b'UNIQUE_ASCII_HOOK'
pos = raw.find(anchor)
if pos < 0: print('MISS'); exit(1)

# 找到 anchor 所在行的 \r\n
le = raw.find(b'\r\n', pos)
insert = b'\r\nNEW_LINE_CONTENT'

new_raw = raw[:le+2] + insert + raw[le+2:]
with open(path, 'wb') as f: f.write(new_raw)
print('OK')
"
```

### 二进制铁律

| 规则 | 为什么 |
|------|--------|
| anchor 只用 ASCII，绝不含 `//` 后的内容 | `//` 后是 GBK 中文，字节值不确定 |
| 插入文本**首尾都要 `\r\n`** | 漏了会把两行粘成一行 |
| 删除时从 `ls`（行首）删到 `le+2`（跳过 `\r\n`） | 不留空行 |
| 插入后立刻 grep + Read 验证 | 防止 `\r\n` 遗漏导致行合并 |

## 常见陷阱

| 陷阱 | 后果 | 正确做法 |
|------|------|---------|
| old 字符串用单引号但内容含 `'` | Python SyntaxError | 外层改用 `"` 或 `'''` |
| old 不含 `\n` 只匹配行片段 | 残留中文注释后半截 | 始终包含 `\n` 删除整行 |
| 同一字符串在文件中出现多次 | 所有出现都被替换 | 用跨行锚点限定唯一位置 |
| Bash 双引号内写 `$var` | Bash 尝试展开变量 | `$` 不是 Python 变量则无关；如冲突用 `\$` |
| 改 struct 字段不改 config init | config 编译报错（多余 initializer） | 同步修改 .h 和 .c，保持一致 |
| **文本 g() MISS 后反复试中文** | 浪费时间，每次 MISS | **立刻切二进制 ASCII anchor** |
| **二进制 insert 漏 `\r\n`** | 两行粘成一行 | insert 首尾都加 `\r\n` |
| **二进制 anchor 含 `//` 后 GBK 字节** | anchor 找不到 | 只用 ASCII 部分定位 |
| **编辑前不查唯一性** | 改错位置 | Step 2.5: grep 确认 old 出现次数 |

## 规则

1. 所有 g() 放同一个 python -c，一次改完
2. old 必须精确匹配（含缩进、换行），不匹配打 MISS → **立刻切二进制兜底**
3. 零临时文件、不产生 .bak、不生成 .py 脚本
4. 不要分多次 python -c（每次启动 ~200ms 开销）
5. 不要单独测试中文 roundtrip（Step 1 已合并）
6. 不要分文件检查缩进（Step 1 的 hex dump 一次覆盖所有文件）
7. **Step 2.5: 检查 old 字符串在文件中出现次数，!=1 则用更长的跨行锚点**
8. **二进制编辑后必须 Read 验证相邻行，防止 `\r\n` 遗漏**
