# 01_STEP模型处理

本目录存放 **STEP 模型文件处理** 阶段的内容：原始 STEP 中文字符乱码的修复、翻译，以及 SolidWorks 零件模型。

## 目录结构

```
01_STEP模型处理/
├── 翻译工具/          # STEP 翻译相关脚本与数据
│   ├── extract_chinese_names.py   # 以合适编码打开 STEP，提取中文字符并整理成列表
│   ├── step_translate.py          # 以相同编码打开 STEP，将中文替换为英文
│   ├── translate.csv              # 中英文对照翻译表（用于比对）
│   ├── GB2312中文.txt             # 提取出的中文字符清单
│   └── result.txt                 # 处理结果
├── 模型文件/          # STEP/SolidWorks 模型文件
│   ├── origin/                    # 原始 STEP
│   └── modul_SW/                  # SolidWorks 零件/装配
└── 处理流程说明.md     # STEP 乱码修复流程说明
```

## 处理流程（摘要）

1. **提取中文**：用 `extract_chinese_names.py` 以合适的编码方式打开 STEP 文件，提取其中的中文字符，以列表方式整理（对应 `GB2312中文.txt`）。
2. **翻译**：得到翻译文件，采用 csv 格式（`translate.csv`）便于比对中英文，注意避免特殊字符以防编码错误。
3. **替换**：用 `step_translate.py` 以相同编码打开 STEP 文件，将中文替换为英文，解决 STEP 文件乱码问题并可在不同平台使用。
4. **转存 SolidWorks 格式**：可将 STEP 保存为 SolidWorks 格式，加快 SolidWorks 模型加载速度，便于后续处理（URDF 导出）。

## 注意

- 本目录文件**未做任何内容修改**，仅按主题归并。
- 翻译后的模型继续流向 `../02_模型转换工具/` 与 `../03_模型流水线_SW转ROS2/` 进行 URDF/XACRO 转换。
