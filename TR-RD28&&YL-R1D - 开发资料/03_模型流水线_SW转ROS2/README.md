# 03_模型流水线_SW转ROS2

本目录存放 **SolidWorks 导出 URDF → 完整 ROS2 功能包** 的核心流水线项目，是「机器人模型」处理的最终成果所在。

## 目录结构

```
03_模型流水线_SW转ROS2/
└── urdf_ros2_rviz2/        # 核心项目（完整保留，含 .git、README.md）
    ├── README.md           # 项目 README：流水线 8 步说明、用法、生成包结构
    ├── model_pipeline/     # 核心流水线（Python 模块，pipeline.py + steps/）
    │   ├── pipeline.py     # 主入口，编排 8 个处理步骤
    │   ├── utils.py        # 共享工具
    │   └── steps/          # step_clean/rename/fix_urdf/sensors/config/urdf2xacro/gazebo/package
    ├── Create_Model.bat/.sh    # Windows/Linux 一键运行流水线
    ├── Restore_Model.bat/.sh   # 从备份恢复
    ├── YLR1D_Model/        # 示例输入：SW 导出的原始模型
    ├── YLR1D_Model_copy/   # 流水线自动生成的备份
    ├── Robot/              # 另一份输入模型（含 gazebo 版）
    ├── src/                # 输出目录
    │   └── ylr1d_description/   # 示例输出：完整 ROS2 功能包（URDF/XACRO + meshes + config + launch + rviz）
    ├── sensors_description.yaml  # 传感器复杂功能配置
    ├── backup/             # 历史备份（Robot、src、ThirdParty、ylr1d_sonar_description 等）
    ├── docs/ config/       # 文档与配置
    └── src.zip             # 输出目录压缩包
```

## 流水线做什么

将 SolidWorks 导出的 URDF 目录自动处理为**开箱即用**的 ROS2 功能包，共 8 步：

1. `step_clean` 清理 SW 导出产物 → 2. `step_rename` 全局重命名 → 3. `step_fix_urdf` 修复 URDF（去重关节、mimic、传感器 link、限幅）→ 4. `step_sensors` 提取传感器 → 5. `step_config` 生成 links/colors/limits/scale YAML → 6. `step_urdf2xacro` 转 XACRO（可选）→ 7. `step_gazebo` 添加 Gazebo 传感器与 ROS2 插件 → 8. `step_package` 生成标准 ROS2 包。

## 快速使用

```bash
# Windows
Create_Model.bat Robot ylr1d
# Linux
./Create_Model.sh Robot ylr1d
```

生成的包在 `src/ylr1d_description/`，可直接 `colcon build` 后用 `ros2 launch ylr1d_description xacro_display.launch.py` 在 RViz2 显示、`xacro_gazebo.launch.py` 在 Gazebo 仿真。

## 如何继续开发

- 修改流水线逻辑：编辑 `model_pipeline/steps/` 下对应 step。
- 新模型接入：把 SW 导出的模型目录放到项目下，按 `YLR1D_Model` 的格式准备，再运行 `Create_Model`。
- 详细说明请先阅读 `urdf_ros2_rviz2/README.md`。

> 整理说明：本目录为核心流水线项目的完整归档，内部结构未做任何改动。
