# 02_模型转换工具

本目录集中存放 **模型格式转换工具链**：从 STEP 直接转 URDF、URDF 与 XACRO 互转、从 XACRO/URDF 提取 DH 运动学参数等。

## 目录结构

```
02_模型转换工具/
├── urdf_from_step_ros1/       # 第三方 ROS1 包：STEP → URDF（源自 ReconCycle 项目）
├── urdf_from_step_ros2/       # ROS2 移植版
├── URDF2Xacro/                # URDF 转换为 Xacro 的工具
└── urdf2dh/                   # 模型参数提取工作区（详见其 README）
    ├── README.md
    ├── config/                # ylr1d 模型配置（links/colors/limits/scale/dynamics/sensors...）
    ├── urdf/                  # ylr1d.urdf / ylr1d.xacro（权威模型）
    ├── meshes/                # STL 网格
    ├── xacro2dh/              # 工具：xacro 展开 → 提取 DH/动力学/传感器位姿 → YAML
    ├── xacro2urdf/            # 工具：xacro + config → 自包含静态 URDF
    └── ThirdParty/            # 第三方 urdf2dh 库（URDF → DH 参数）
```

## 工具速览

| 工具 | 输入 → 输出 | 说明 |
|---|---|---|
| `urdf_from_step_ros1/ros2` | STEP → ROS 功能包（URDF + STL + launch） | 第三方 ReconCycle 项目；依赖 pythonocc-core，官方推荐 Docker 安装；本项目中用于评估「直接从 STEP 生成 URDF」的路线 |
| `urdf2dh/xacro2dh` | xacro + config → kinematics/dynamics/sensors YAML | 自研；展开 xacro 后提取标准 DH 表、动力学参数、传感器位姿，并做 FK 交叉验证 |
| `urdf2dh/xacro2urdf` | xacro + config → 自包含 URDF | 自研；仅实现当前 ylr1d 模型所需的最小 xacro 子集 |
| `urdf2dh/ThirdParty` (urdf2dh) | URDF → DH 参数 | 第三方开源库，仅支持串联机器人 |

## 如何继续开发

- 模型转换的**最终成果**不在本目录：由 `../03_模型流水线_SW转ROS2/urdf_ros2_rviz2` 流水线生成 `ylr1d_description` 功能包。
- 若需提取参数（DH/动力学/传感器位姿），进入 `urdf2dh/` 阅读其 README，直接运行 `python xacro2dh/run.py`（默认读取 `urdf/ylr1d.xacro` 与 `config/`）。
- `urdf_from_step` 系工具依赖较重（pythonocc-core），仅当需要验证 STEP→URDF 自动转换时再使用。

> 整理原则：**只移动、不修改任何文件内容**。两版 `urdf_from_step_ros2` 内容略有差异（顶层版多 `launch/template.launch`），均原样保留。

# 附录
[urdf_from_step_ros1库](https://github.com/ReconCycle/urdf_from_step)

[urdf_from_step_ros2库](https://github.com/Zhengshuji/urdf_from_step_ros2)