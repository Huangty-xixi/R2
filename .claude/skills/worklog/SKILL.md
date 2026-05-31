---
name: worklog
description: 工作日记 — 录入待完成事项。用户说"记一下"、"备忘"、"加个待办"、"工作日记"、"worklog" 时触发。读取工作日记，将用户说的事项追加到"待完成"区域。
---

## 工作日记 Skill

### 文件

- 工作日记: `e:\EK\main_R2\工作日记.md` (GB2312/GBK 编码)
- 日记中包含 `## 待完成` 段落和按日期排列的已完成事项

### 操作流程

#### 读取日记

```powershell
$file = "e:\EK\main_R2\工作日记.md"
$enc = [System.Text.Encoding]::GetEncoding("GB2312")
$content = [System.IO.File]::ReadAllText($file, $enc)
```

#### 追加待完成事项

在 `## 待完成` 区域末尾（`<!-- skill ... -->` 注释行上方）插入 `- [ ] <用户说的事项>`。

用 `.Replace("旧文本", "新文本")` 完成插入。旧文本为 `<!-- skill` 注释行，新文本为 `- [ ] xxx\r\n<!-- skill`。

```powershell
$old = "<!-- skill"
$new = "- [ ] <事项内容>`r`n<!-- skill"
$content = $content.Replace($old, $new)
[System.IO.File]::WriteAllText($file, $content, $enc)
```

#### 标记完成

用户说"做完了"、"完成了"时，把对应行的 `- [ ]` 改成 `- [x]`。

#### 当日归档

用户说"归档"、"今天收工"时，在 `## 待完成` 上方插入 `## YYYY-MM-DD` 日期标题，把已完成事项移到该日期下，保留未完成事项在 `## 待完成` 中。

### 注意事项

- 日记文件是 GB2312 编码，务必用 PowerShell + `[System.Text.Encoding]::GetEncoding("GB2312")` 读写
- 事项一行一个，用 `- [ ]` 格式
- 操作完读取文件验证无乱码
