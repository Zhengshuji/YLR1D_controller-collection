# 04_控制与仿真

本目录存放 **控制架构设计与验证** 相关内容：Gazebo 中的三层控制架构（描述/中层控制已验证）、Matlab 仿真脚本等。

## 目录结构

```
04_控制与仿真/
├── robot_control/         # 控制架构项目（git 仓库，完整保留）
│   ├── ARCHITECTURE.md    # 三层控制架构总览（上层规划/中层转发/下层驱动）
│   ├── docs/              # 文档
│   │   └── control_methods.md   # 控制方法详解（控制器分类、关节明细、遥操作等）
│   ├── src/
│   │   ├── ylr1d_description/   # 共享模型包（meshes + urdf + config，被各层依赖）
│   │   ├── ylr1d_mid_control/   # 中层控制（已验证）：ros2_control + ForwardCommandController
│   │   │   ├── README.md        # 中层包说明（控制器清单、关节顺序、踩坑记录）
│   │   │   ├── config/controllers.yaml
│   │   │   ├── launch/gazebo.launch.py
│   │   │   └── urdf/ylr1d_mid.xacro
│   │   └── others/ylr1d_control # 备选控制包（PID 控制器、遥操作键盘、arm_commander）
│   └── .git/
├── Matlab_Simulate/       # Matlab 仿真脚本
│   ├── Vehicel_sim.m      # 车辆运动仿真
│   └── Controller_sim/    # 位置/速度控制仿真（pos_control_sim.m, vel_control_sim.m）
└── ylr1d_controller/      # 空目录（预留）
```

## 三层控制架构（摘要）

```
上层  运动学/规划层  (ylr1d_upper_control, 待实现)
  │   底盘麦克纳姆轮正逆解、双臂 IK/MoveIt2、躯干规划
  │   输出: 6 x Float64MultiArray → /<controller>/commands
  ▼
中层  指令转发层  (ylr1d_mid_control, 已验证)   ← 本目录已实现
  │   ros2_control + ForwardCommandController
  │   6 个控制器（底盘转向4/底盘驱动4/躯干4/左臂9/右臂9 + joint_state_broadcaster）
  │   Gazebo Classic + gazebo_ros2_control
  ▼
下层  关节驱动层  (ylr1d_lower_control, 待实现)
      PD 位置/速度闭环、重力补偿、限位保护、力矩/电流环
```

**当前进度**：中层已验证通过 —— 30 个可控关节在 Gazebo 中正确加载，通过 Topic 发布 `Float64MultiArray` 可控制全部关节。上层（运动学/规划）与下层（关节驱动）为 TODO。

## 快速开始（验证中层）

```bash
# WSL Ubuntu 22.04, ROS2 Humble
source /opt/ros/humble/setup.bash
cd <workspace>/robot_control/src/../..   # 将 src 下各包放入工作区
colcon build
source install/setup.bash
ros2 launch ylr1d_mid_control gazebo.launch.py

# 另一终端发送测试指令
ros2 topic pub /torso_controller/commands std_msgs/Float64MultiArray "data: [0.2, 0.5, -0.3, 0.1]" --once
```

## 如何继续开发

- 先读 `robot_control/ARCHITECTURE.md` 与 `robot_control/src/ylr1d_mid_control/README.md`（含解决过的问题：gazebo_ros2_control 插件参数解析、LD_LIBRARY_PATH、YAML 结构等）。
- 上层/下层实现可参考 `docs/control_methods.md` 中的接口约定。
- Matlab 仿真脚本用于验证底盘/控制器基本行为，可在 Matlab/Octave 中直接运行。

> 整理说明：本目录由 robot_control、Matlab_Simulate、ylr1d_controller 三个项目归档组成，内部结构未做任何改动。
