# 硬规则

## 改 .c/.h 代码

必须用 **safe_rw_safe_edit**（MCP 工具）。Keil 工程源文件为 GBK 编码，内置 Edit/Write 会损坏中文注释。safe_rw_safe_edit 自动检测并保持 GBK/UTF-8 编码。

用法：
```
safe_rw_safe_edit:
  file_path: 文件绝对路径
  old_string: 要替换的原文（精确匹配，含缩进、中文注释）
  new_string: 替换后的文本
  replace_all: 是否替换全部匹配（默认 false）
```

读文件用 **safe_rw_safe_read**，搜索用 **safe_rw_safe_search**，新建文件用 **safe_rw_safe_write**。

规则：
1. 改前必须先用 safe_rw_safe_read 读文件确认缩进和行尾
2. old_string 必须精确匹配，含缩进和中文注释
3. 若匹配多处且不想全部替换，加更多上下文行定位唯一匹配
4. 不确定缩进/行尾就先读文件确认
