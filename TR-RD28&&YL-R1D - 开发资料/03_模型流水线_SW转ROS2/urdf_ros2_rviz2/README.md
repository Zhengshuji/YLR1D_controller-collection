# SW-URDF 转 ROS2 模型流水线

将 SolidWorks 导出的 URDF 模型自动处理为完整的 ROS2 功能包，开箱支持 RViz2 可视化和 Gazebo 仿真。

## 目录结构

```
├── model_pipeline/          # 核心流水线 (Python 模块)
│   ├── pipeline.py          # 主入口，编排 8 个处理步骤
│   ├── utils.py             # 共享工具函数
│   └── steps/
│       ├── step_clean.py    # 清理 SW 导出产物
│       ├── step_rename.py   # 重命名模型/包名
│       ├── step_fix_urdf.py # 修复 URDF 问题
│       ├── step_sensors.py  # 提取传感器配置
│       ├── step_config.py   # 提取配置数据 (links/colors/limits/scale)
│       ├── step_urdf2xacro.py # URDF → Xacro 转换
│       ├── step_gazebo.py   # 添加 Gazebo 标签和 ROS2 插件
│       └── step_package.py  # 生成 ROS2 包 (launch/CMakeLists/package.xml)
├── Create_Model.bat         # Windows 脚本：运行流水线
├── Create_Model.sh          # Linux/macOS 脚本：运行流水线
├── Restore_Model.bat        # Windows 脚本：从备份恢复
├── Restore_Model.sh         # Linux/macOS 脚本：从备份恢复
├── YLR1D_Model_copy/        # 示例输入：SW 导出的原始模型
├── src/                     # 输出目录：生成的 ROS2 包
│   └── ylr1d_description/   # 示例输出：完整 ROS2 功能包
├── sensors_description.yaml # 传感器配置文件，用于实现复杂的传感器功能

## 工作流程

流水线自动执行 **8 个步骤**，将原始 SW 导出的 URDF 转化为可直接使用的 ROS2 包：

| 步骤 | 模块 | 功能 |
|------|------|------|
| 1 | `step_clean` | 清理 SW 导出产物（ROS1 文件、textures、目录重构） |
| 2 | `step_rename` | 将旧模型名全局替换为新名称 |
| 3 | `step_fix_urdf` | 去重关节、修复 mimic 关节、清理传感器 link、限制极值 |
| 4 | `step_sensors` | 从 URDF 提取传感器信息，生成 `sensors.yaml` |
| 5 | `step_config` | 生成 `links.yaml`、`colors.yaml`、`limits.yaml`、`scale.yaml` |
| 6 | `step_urdf2xacro` | 可选：将 URDF 转为 Xacro 宏格式 |
| 7 | `step_gazebo` | 添加 Gazebo 传感器标签和对应的 ROS2 插件 |
| 8 | `step_package` | 生成完整 ROS2 功能包（标准目录布局 + launch 文件 + RViz 配置） |

### 生成 ROS2 包的目录结构

```
<name>_description/
├── urdf/            # .urdf / .xacro 模型文件
├── meshes/          # .STL 网格文件
├── config/          # YAML 配置文件
│   ├── links.yaml   # link 惯性参数
│   ├── colors.yaml  # 可视化颜色
│   ├── limits.yaml  # 关节限位
│   ├── scale.yaml   # 密度缩放
│   └── sensors/     # 各传感器独立配置
├── launch/          # ROS2 启动文件
│   ├── urdf_display.launch.py    # URDF + RViz2
│   ├── urdf_gazebo.launch.py     # URDF + Gazebo
│   ├── xacro_display.launch.py   # Xacro + RViz2
│   └── xacro_gazebo.launch.py    # Xacro + Gazebo
├── rviz/            # RViz2 配置文件
├── CMakeLists.txt
├── package.xml
└── model.config     # Gazebo 模型数据库配置
```

## 快速开始

### 前置要求

- Python 3.8+
- PyYAML（可选，用于 YAML 输出）
- ROS2 Humble / Iron / Rolling（运行生成的包时）
- Gazebo + `gazebo_ros`（运行 Gazebo 仿真时）

### 使用示例

```bash
# 处理 Robot 目录中的模型，命名为 ylr1d，输出到 ./src
Create_Model.bat Robot ylr1d

# 指定输出目录 + 详细日志
Create_Model.bat Robot ylr1d -o ./src -v

# 跳过 Xacro 生成（仅保留 URDF）
Create_Model.bat Robot ylr1d --no-xacro

# 处理完成后删除备份
Create_Model.bat Robot ylr1d --no-backup
```
```bash
python -m model_pipeline -s YLR1D_Model -n ylr1d -o ./src -sc sensors_description.yaml
```
### 运行生成的 ROS2 包
需要提前对环境进行检查：
1. 是否包含python3的numpy等库
2. 环境变量GAZEBO_MODEL_PATH是否指向该项目的src路径。如果需要添加，可以使用下面指令：
  ```bash
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/your/workspace/src
```
3. gazebo相关进程是否关闭，端口也需要检查。
之后，可以进行编译。
```bash
# 1. 编译
cd <your_ros2_ws>
colcon build
source install/setup.bash

# 2. 在 RViz2 中显示
ros2 launch <name>_description urdf_display.launch.py

# 3. 在 Gazebo 中仿真
ros2 launch <name>_description urdf_gazebo.launch.py
```
如果最终没能得到预期结果，出现下面这些报错情形，则彻底关闭环境，重新启动再次尝试：
1. source install/setup.bash，但是仍然显示ros2无法找到对应执行文件
2. 因未知错误，导致gazebo启动失败，出现Error报告，并提前关闭

## 恢复原始模型

流水线在处理前会自动将源目录备份为 `<name>_copy`，可用恢复脚本还原：

```bash
# 从 Robot_copy 恢复 Robot
Restore_Model.bat Robot
```

## 输入要求

原始 SW 导出的 URDF 目录应包含：

- `.urdf` 文件（位于根目录或 `urdf/` 子目录下）
- `meshes/` 目录，包含 `.STL` 网格文件
- 命名规范：传感器 link 名称以 `Sensor` 结尾（如 `Link_IMUSensor`）

## 支持的传感器类型

| 类型 | 命名规则 | Gazebo 插件 |
|------|----------|-------------|
| IMU | 名称含 `IMU` | `libgazebo_ros_imu_sensor.so` |
| 摄像头 | 名称含 `Camera` | `libgazebo_ros_camera.so` |
| 激光雷达/雷达 | 名称含 `Radar`/`Lidar`/`Laser` | `libgazebo_ros_ray_sensor.so` |

## `.gitignore` 说明

```
_copy      # 自动生成的目录备份被忽略
YLR1D      # 历史模型目录
ylr1d      # 处理过程中的工作目录
build/ install/ log/   # ROS2 编译产物
```
