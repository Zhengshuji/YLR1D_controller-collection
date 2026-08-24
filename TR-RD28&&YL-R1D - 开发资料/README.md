# TR-RD28 && YL-R1D —— 开发资料（归档）

> 本目录是 TR-RD28 && YL-R1D 机器人系统二次开发的**成果归档**，对应《TR-RD28&&YL-R1D机器人系统控制器开发报告》附录 A.2「处理过的内容」：
>
> - **机器人模型**：原始 STEP → 修复乱码/翻译 → SolidWorks 加工 → URDF/XACRO → ROS2 功能包
> - **机器人参数**：基于模型与仿真程序确定的运动学（DH）、动力学、传感器位姿参数
> - **模型转换工具链**：STEP→URDF、URDF↔XACRO、XACRO→DH 等自研/第三方工具
> - **控制验证**：Gazebo 仿真中的三层控制架构验证（中层已验证通过）
>
> 原始厂方资料（技术说明书、SDK、例程等）在 `../TR-RD28&&YL-R1D - 副本/`；本目录是在其基础上**加工、转换、验证**的成果。

## 目录结构

```
TR-RD28&&YL-R1D - 开发资料/
├── README.md                        # 本说明
├── 01_STEP模型处理/                  # STEP 文件翻译与 SolidWorks 加工
│   ├── 翻译工具/                    # extract_chinese_names.py、step_translate.py、translate.csv 等
│   ├── 模型文件/                    # 原始/翻译后 STEP、SolidWorks 零件（含加超声波传感器版本）
│   └── 处理流程说明.md              # STEP 乱码修复流程说明
├── 02_模型转换工具/                  # STEP→URDF、URDF↔XACRO、XACRO→DH 工具链
│   ├── urdf_from_step_ros1/         # 第三方 ROS1 包：STEP 直接转 URDF（含 .git、README）
│   ├── urdf_from_step_ros2/         # ROS2 移植版（含 launch/template.launch）
│   ├── urdf_from_step_ros2_备份/    # ROS2 移植版（另一份）
│   └── urdf2dh/                     # 模型参数提取工作区：xacro2dh、xacro2urdf、urdf2dh 库
├── 03_模型流水线_SW转ROS2/           # 核心流水线项目
│   └── urdf_ros2_rviz2/             # SW 导出 URDF → 完整 ROS2 功能包（8 步流水线，含 README）
├── 04_控制与仿真/                    # 控制验证与仿真
│   ├── robot_control/               # 三层控制架构（描述/中层控制，含 .git 与 ARCHITECTURE.md）
│   ├── Matlab_Simulate/             # Matlab 车辆/控制器仿真脚本
│   └── ylr1d_controller/            # 空目录（预留）
├── 05_历史版本与备份/                # 各版本的中间产物与备份
│   └── others/                      # SWurdf、test_urdf、urdf_and_xacro_from_SW 等历史迭代
└── 06_工作区配置/                    # 开发工具配置
    └── .vscode/                     # VSCode 工作区设置
```

## 各主题目录速览

| 目录 | 内容 | 与报告的对应关系 |
|---|---|---|
| `01_STEP模型处理` | STEP 乱码修复/翻译工具与模型文件 | 附录 A.2.1 机器人模型（前期处理） |
| `02_模型转换工具` | STEP→URDF、URDF↔XACRO、XACRO→DH 工具 | 附录 A.2.1 机器人模型（转换环节） |
| `03_模型流水线_SW转ROS2` | SolidWorks URDF → ROS2 功能包自动流水线 | 附录 A.2.1 机器人模型（成品生成） |
| `04_控制与仿真` | Gazebo 三层控制验证、Matlab 仿真 | 附录 A.2.4 数学模型与控制器验证 |
| `05_历史版本与备份` | 各阶段中间产物、调试记录 | 附录 A.2 处理过程存档 |

## 如何继续开发

1. **模型相关**：从 `03_模型流水线_SW转ROS2/urdf_ros2_rviz2` 开始，其 README 说明了完整流水线与用法；最终生成的 `ylr1d_description` 功能包在 `urdf_ros2_rviz2/src/` 下。
2. **控制相关**：从 `04_控制与仿真/robot_control` 开始，`ARCHITECTURE.md` 与 `docs/control_methods.md` 说明三层控制架构；中层（`ylr1d_mid_control`）已在 Gazebo 中验证通过。
3. **参数提取**：`02_模型转换工具/urdf2dh` 提供 xacro2dh（提取 DH/动力学/传感器位姿）与 xacro2urdf（xacro 转静态 URDF）。
4. 各子目录均有独立 README，先读对应 README 再动手。
