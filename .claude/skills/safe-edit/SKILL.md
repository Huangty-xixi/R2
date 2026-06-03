---
name: safe-edit
description: 编辑本项目的 GB2312/GBK 编码源码文件（.c/.h/.s 等），防止中文注释乱码。当用户说"改代码"、要求修改源代码、或任务涉及编辑项目源文件时触发。本项目所有源文件为 GB2312 编码，严禁对源文件使用内置 Edit/Write 工具。
---

## 核心规则

本项目所有源码文件使用 **GB2312 (GBK)** 编码。内置 Edit/Write 工具可能破坏编码导致中文注释乱码，**严禁** 对源文件使用 Edit/Write 工具。

## 适用范围

- `.c` / `.h` 源码
- `.s` 汇编文件
- 本项目所有其他源代码文件

## 操作流程

### 步骤 1：读取文件

```powershell
$file = "文件绝对路径"
$enc = [System.Text.Encoding]::GetEncoding("GB2312")
$content = [System.IO.File]::ReadAllText($file, $enc)
```

### 步骤 2：替换内容

使用 `$content.Replace("旧字符串", "新字符串")` 替换。

要点：
- 行尾是 `\r\n` (CRLF)
- 旧字符串必须精确匹配，含 TAB/空格缩进
- 建议先 `Write-Host` 打印目标区域确认缩进，再构造替换字符串
- 字符串中的 `$` 需要写成 `` `$ `` 转义，反引号写成 ``` `` ```，双引号写成 `` `" ``
- 多行字符串用 here-string `@"..."@` 或显式 `` `r`n `` 拼接

### 步骤 3：写入文件

```powershell
[System.IO.File]::WriteAllText($file, $content, $enc)
```

### 步骤 4：替换失败必须人工介入

如果 `$content.Contains($old)` 返回 `$false`：

1. 打印 "匹配失败，字符串未找到"
2. 打印目标区域附近原始内容
3. **停止操作，等待用户手动处理**，禁止猜测重试

```powershell
if ($content.Contains($old)) {
    $content = $content.Replace($old, $new)
    [System.IO.File]::WriteAllText($file, $content, $enc)
    Write-Host "OK"
} else {
    Write-Host "匹配失败，字符串未找到。目标区域内容："
    Write-Host $content.Substring($start, $length)
}
```

### 步骤 5：验证

改完后重新读取文件相关区域，确认改动正确无乱码。
