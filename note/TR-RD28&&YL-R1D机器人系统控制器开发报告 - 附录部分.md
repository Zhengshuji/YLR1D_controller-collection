---
title: TR-RD28&&YL-R1D机器人系统控制器开发报告
status: draft
owner: 待定
created: 2026-07-15
updated: 2026-08-22
version: 1.1.0
---

# TR-RD28&&YL-R1D机器人系统控制器开发报告

# 摘要

本文介绍基于 TR-RD28&&YL-R1D 机器人系统（双机械臂 + 全向底盘 + 夹爪 + 躯干）的控制器二次开发工作。系统以厂家随设备提供的开源 SDK 与三维模型为起点，按决策、规划、转译、控制、算法、仿真、感知等分层架构组织各功能包：原始 STEP 模型经加工转换为 URDF/XACRO，并自动提取运动学（标准 DH）、动力学与传感器位姿参数，控制系统在 Gazebo 仿真中完成分层搭建与中层验证；随后将 SDK 封装为 ROS2 节点并接入控制系统框架，在 SDK 自带仿真（TR-Sim）上完成驱动层、moveit、导航与决策层的全链路实测。正文按功能包介绍各层结构、功能与局限性；附录 A 盘点原始资料并记录加工、转换与验证过程，附录 B 记录 SDK 封装与真机路线验证。

# 1 总体框架

YLR1D 机器人控制系统采用分层框架，核心结构如下：

各层级的功能如下表所示：

|层级|功能|输入|输出|
|---|---|---|---|
|决策层|管理、维护、调用行为树|任务|编排好的、可规划的目标|
|规划层|管理、调用nav2、moveit2|目标位置、位姿|可执行的、封装的动作序列|
|转译层|与SDK包对齐|封装的动作序列|解包的目标动作|
|控制层|下位机信息枢纽|解包的目标动作|控制层管理、控制信息|
|算法层|封装控制算法|输入、反馈|控制律输出|
|仿真层|克服仿真局限性|控制信息|仿真控制器|
|仿真系统|仿真主要环境|仿真控制器|与环境交互结果|
|感知层|消除感知接口差异|各传感器结果|处理后结果、高级感知结果|

特别地，hmi 提供人机交互界面，description 提供全局统一的模型与参数资源，test 提供测试工具，三者未在上表中体现。

# 2 description

提供统一模型及参数，统一管理各类资源，尤其是资源需要跨包同步的情况下。

## 2.1 模型

核心为xacro模型，并提供相关的可变参数config以yaml形式管理，meshes贴图，以及查看模型用的launch启动文件。

## 2.2 项目全局参数

以头文件的形式管理，保证全局统一。例如模型运动学参数、传感器话题。

## 2.3 人机交互界面

管理rviz框架。

## 2.4 仿真环境

管理world、map等与仿真相关的内容

## 2.5 局限性

不包含urdf模型，在moveit需要额外的转换。

xacro 并非标准的外部参数引入形式，而是通过 Python 程序在构建期完成转换与参数展开。好处是，转换过程可脚本化、可复现，无需额外的运行时解析。

# 3 plant

底层仿真。目前主要为运动学仿真。可以进行动力学仿真。

为降低开销，使用gzserver不可视化。

## 3.1 运动学仿真

xacro提供直接设置位置进行控制的控制器。

在运动学仿真中，由于gazebo提供的控制器仅能控制速度、位置，因此，在plant中额外增加积分器，实现输入加速度、到输出的转换。

## 3.2 动力学仿真

xacro提供直接设置力矩进行控制的控制器。

# 4 algorithm

提供控制算法，理论上为纯C/C\+\+库，实际中采用ROS2通信以消除接口差异，使用composition方法降低通信开销。依赖三方库Eigen。

## 4.1 控制器

提供了统一的控制器纯抽象基类，提供统一的外部接口。

大致包含这样几类：核心的算法函数，参数、状态管理函数

### 4.1.1 基础控制器

目前仅包含PID控制器。

控制器使用公共的基类。对于控制算法，包含：输入、反馈、离散时间。绝大多数控制算法都只需要这些量，便能完成控制与自适应调节。

### 4.1.2 组合控制器

实现将基础控制器组合起来，实现控制器的组合，实现相对复杂的控制效果。

> 在项目中，为了演示，使用如下的控制流程：
> 
> ```Bash
> 所有关节 -> K=1比例控制器 -> 中间结果
> for i:中间结果
>     i -> 对应的PID控制器 -> 输出
> ```
> 
> 

特别地，根据需要可以固定为5个项目需要的组合控制器。

此外，还包含服务，作为外部接口。

## 4.2 接口

组合控制器及其内部所有的参数、状态均以topic进行展示。

提供service，对指定的控制器进行管理，包括配置参数、设置状态等等。

# 5 control

控制层，主要作为下位机信息收发核心节点。同时，还提供对控制的管理。

## 5.1 基础路由功能

接受上层的控制信号；

调用算法层的控制器，并对组合控制器进行分组管理；

向下层发送控制信号；

## 5.2 控制过程管理

为了减小反馈纯滞后对控制的影响，控制层额外管理了一个预测器，提供前馈补偿。

# 6 translate

转译层，主要实现接口与SDK包对齐。

## 6.1 机械臂

仅选取关节空间控制的方法。输入关节空间坐标即可，并进行限幅处理。

## 6.2 底座

选取3种主要运动方式：

1. 平移，沿任意方向，平动

2. 旋转，在原地旋转

3. 停车，车轮自锁

## 6.3 夹爪

使用bool量进行控制。

# 7 perception

基本实现所有传感器的管理，以及高级感知功能的实现、信息派发。

主要依赖message\_filters框架实现。

## 7.1 基础感知

使用message\_filters进行订阅。

对信息进行一定的预处理，主要是：

1. 数据清洗

2. 频率控制

3. TF变换

## 7.2 高级感知

暂时只实现轮式里程计、基于robot\_localization的定位

## 7.3 派发

使用message\_filters，收集信息，发送前检验数据有效性，统一发送。



# 8 plan

实际包含导航规划nav与机械臂规划moveit

两者使用同样的框架：

## 8.1 nav

nav主要依赖nav2实现

与定位相关的内容放在感知层。

在桥接部分，使用状态机决定是停止还是运动，使用滞回比较器判断是平移还是旋转。

## 8.2 moveit

moveit主要依赖moveit2实现

另外，编写了moveit2所需配置的脚本。

在服务部分，主动维护话题，同时为torso提供特别的方法。

# 9 hmi

人机交互界面统一管理。

## 9.1 rviz2

主要是监视面板，提供与系统相关的信息。rviz2的插件存放在这里，但是rviz2的框架存放在description。

## 9.2 qt5界面

主要是控制面板，提供与控制相关的信息。

# 10 bringup

快捷启动包。

# 11 test

测试包。

# 问题及隐患

1. 为改善性能增加了一些非必要的环境参数，提高了系统配置的复杂度；
2. bringup 各启动入口不一致，参数配置存在差异；
3. 系统整体鲁棒性较差，对启动顺序与环境变量有一定依赖。

# 附录

## A. 原始资料与二次开发成果

附录 A 记录二次开发的完整输入与加工过程：先盘点厂家随设备提供的原始开源资料（A.1，归档于 `TR-RD28&&YL-R1D - 副本/`），再说明模型、参数、设计、测试、建模与验证等加工成果（A.2，归档于 `TR-RD28&&YL-R1D - 开发资料/`），最后给出结论与后续工作（A.3）。原始资料内容重复、冗余且分散，本项目先对其整理归档，再完成加工、转换与验证；正文各层设计即建立在这些成果之上。

| 章节 | 内容 | 归档位置 |
|---|---|---|
| A.1 原始内容 | 厂家资料盘点（SDK/例程/模型/仿真/文档） | `TR-RD28&&YL-R1D - 副本/` |
| A.2 处理过的内容 | 模型、参数、设计、测试、建模、验证 | `TR-RD28&&YL-R1D - 开发资料/` |
| A.3 结论与后续工作 | 可复用/必须重做/已知问题/后续工作 | — |

### A.1 原始内容

厂家资料涵盖文档、模型、仿真环境、SDK 与例程五部分。其中对二次开发起决定性作用的是两点：其一，SDK 以闭源动态库形式提供，是控制机器人的唯一途径，其接口能力直接限定了上层控制系统的设计空间；其二，三维模型仅有几何外形、无运动学标注，无法直接用于仿真与规划，必须自行加工。其余资料（技术说明书、网络配置文档、仿真环境、ROS1 例程）为上述两点提供支撑。

#### A.1.1 SDK 包（RobotConSys_SDK）

SDK 是控制机器人本体的唯一接口，以预编译动态库提供，无源码，覆盖 `linux_aarch64`、`linux_aarch64_u20`、`linux_x64`、`win_x64` 四个平台。包内目录结构为：`include/`（全部头文件，含 `RobSoft/CDataStructure.hpp` 等）、`lib/`（四个平台的预编译库）、`example/`（入门 Demo：`RobotConSysDemo.cpp`、`CamDevDemo.cpp`）、`cs/`（C# 封装与设备配置示例）。接口按设备层组织，覆盖系统管理、机械臂、底盘、夹爪与相机：

- 系统：`init(ip, port)`、`close()`、`setAuthority()`；
- 机械臂：伺服开关、关节空间绝对/相对运动、直线/圆弧/跳跃运动、回零/回原点、JOG/步进、等待指令完成、参数查询；
- 底盘：控制类型设置、伺服开关、运动控制（平移/旋转/停车）、状态查询；
- 夹爪：开关状态设置与查询；
- 相机（CamDev_Client）：初始化、彩色/深度图像采集。

SDK 的使用流程固定：动态加载库 → 创建实例 → init 连接 → 设置权限 → 下发指令 → 释放。动态库在运行时由调用代码加载（`SysLayer::CLoadLibrary`，示例为 `libRobotConSys.open(".", "RobotConSys_Client")`），编译目标平台由 `platform.cmake` 的 `TARGET_PLATFORM` 指定。

SDK 存在两处关键限制，决定了后续设计：

1. 部分接口不可用。经测试（A.2.4），末端点动/步进会导致连接断开或报错，直线与圆弧轨迹规划不可用，臂形角点动/步进无效果。因此正文 translate 层（第 6 章）仅选用可用的关节空间控制、底盘三种运动模式与夹爪布尔量控制。
2. 传感器数据受限。SDK 经 TCP/IP 只能直接获取关节与底盘状态，不含 IMU、超声波、雷达，仅视觉图像可读取。这一限制决定了感知层（第 7 章）需自行接入外部传感器。

#### A.1.2 应用例程（YL-R1D 应用案例）

厂家提供的例程是一套 ROS1 导航控制系统，由设备层（`ros_r1d_device`）与上层（`ros_r1d_nav`）组成。设备层将 SDK 封装为 ROS 接口（`robot_package`，含话题、10 个服务与 MoveABSJ 动作），并集成 RealSense、YDLidar、Hipnuc IMU/GNSS 等传感器驱动；上层实现启动编排、机械臂控制、定位建图（amcl/ekf/gmapping）、相机标定、多点导航（move_base 系）、任务控制（TCP 服务端）与 YOLOv8 视觉识别。

例程对本项目的价值不在于直接复用，而在于其提供了三项参照：SDK 封装的完整范式、传感器标定程序（相机标定、里程计标定）与外部传感器/算法依赖清单。同时，例程也明确了必须重写的部分：SDK 自带 TCP 通信已过时（`task_controller/TCP_SDK_old` 为旧版实现），ROS1 框架整体不再适用。其中多数传感器驱动与算法为开源项目，RealSense 相机驱动来自 [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)，YDLidar 雷达驱动来自 [YDLIDAR/ydlidar_ros_driver](https://github.com/YDLIDAR/ydlidar_ros_driver)，YOLOv8 基于 [Ultralytics](https://github.com/ultralytics/ultralytics)。

#### A.1.3 设备模型文件

厂家提供的外发模型为 STEP 格式三维模型（`（标准-外发模型）TR-双臂全向移动复合机器人-0305.STEP`），仅含几何外形，无关节定义、坐标系或质量信息，不能直接用于仿真与规划。模型必须经过加工才能得到可用的 URDF/XACRO，处理过程见 A.2.1。

#### A.1.4 仿真环境及上位机程序

厂家以 Unity 编写仿真环境 TR-Sim 与上位机 TR-Pad，二者与 SDK 使用同一套动态库，可通过与真机一致的接口下发指令、获取状态，从而在没有实体设备时完成开发与联调。TR-Sim 同时是本项目 SDK 测试与真机路线验证（附录 B）的平台。

#### A.1.5 技术说明书与网络配置文档

技术说明书（开发手册）按设备、环境、接口、功能四部分组织：第 1、2 章介绍设备组成与 Linux/ROS 基础，第 3 章运动控制接口、传感器接口与 SDK 直接对应，第 4 章以例程讲解 SLAM、导航、图像识别与手眼标定。网络配置文档说明机器人本体网络、外接路由、Nano 共享 Windows 网盘与遥控器绑定，是联调的前提。

### A.2 处理过的内容

原始资料存在三类缺陷：模型无运动学标注、运动学/动力学/传感器参数缺失、SDK 自带 TCP 通信与 ROS1 例程框架过时。本项目分别处理如下。

#### A.2.1 机器人模型

针对原始 STEP 模型，处理链路为：修复 STEP 中文乱码（`extract_chinese_names.py` 提取中文、`translate.csv` 对照、`step_translate.py` 替换为英文）→ 转为 SolidWorks 格式并添加参考几何体以支持 URDF 导出（含加超声波传感器变体）→ 用 [URDF2Xacro](https://github.com/OpenGHz/URDF2Xacro) 将 SolidWorks 导出的 URDF 转 Xacro 并参数化 → 固化为 8 步流水线（清理、重命名、修复 URDF、提取传感器、生成配置、转 Xacro、添加 Gazebo 插件、打包），以 SolidWorks 导出的 URDF 为输入生成 ROS2 功能包 `ylr1d_description`（含 urdf/xacro、meshes、config、launch、rviz，支持 RViz2 与 Gazebo）。流水线代码开源在 [Zhengshuji/Model-Transformation-from-Urdf-Exported-by-SolidWorks](https://github.com/Zhengshuji/Model-Transformation-from-Urdf-Exported-by-SolidWorks)。同时评估了第三方 STEP 直转 URDF 工具 [urdf_from_step](https://github.com/ReconCycle/urdf_from_step)（依赖 pythonocc-core）及其 [ROS2 移植版](https://github.com/Zhengshuji/urdf_from_step_ros2)，未作为主路线。

#### A.2.2 机器人参数

模型完成后，需确定运动学、动力学与传感器位姿三类参数，均通过工具 `xacro2dh` 从展开后的 xacro 自动提取，并以随机关节角做 FK 交叉验证（DH 结果与 URDF 结果对比）确认一致性。三份参数文档归档于 `TR-RD28&&YL-R1D - 开发资料/02_模型转换工具/urdf2dh/xacro2dh/output/docs/`。

运动学参数采用标准 DH（Craig）约定，共三条链：左臂 `Link_Body2 → Link_LeftArm7`（7 个旋转关节）、右臂 `Link_Body2 → Link_RightArm7`（7 个旋转关节）、躯干 `Link_Base → Link_Body4`（4 个关节）。正运动学为 `FK = base_transform · Π A_i(q) · tool_transform`，各链 base/tool 变换与完整 DH 表（a、alpha、d、theta_offset 及关节限位）见 docs/运动学文档，FK 与 URDF 正运动学逐点一致。

动力学参数逐 link 给出质量、质心与惯量张量（定义于过质心、与 link 系平行的坐标系），其中底座 `Link_Base` 质量 16.208 kg，车身与躯干各节 0.186–3.319 kg，单臂 7 节 0.060–0.303 kg（左右对称）、两指各 0.0109 kg，车轮转向节 0.0653 kg、车轮 0.5905 kg，所有传感器 link 统一质量 0.001 kg、惯量对角 1e-06 kg·m²。

传感器位姿以 `Link_Base` 为参考系，逐传感器给出 `mount.transform`（相对父 link 的固定位姿）与 `pose_base`（零位形相对基座系的位姿）。传感器包括：基座侧全局相机（彩色/深度/红外共用 `Link_GlobalCameraSensor`，挂于 `Link_Body4`）、左右手部相机（各含彩色/深度/红外，挂于对应 `Link_*Arm7`）、雷达、IMU 与四角超声波（均固定于基座）。挂于 DH 链末端的相机，其任意关节状态下的位姿由链 FK 计算。

三份参数的在线文档（飞书）为评审副本，权威内容以本地 docs 与 YAML 为准：

| 参数 | 在线文档 |
|---|---|
| 运动学（标准 DH） | [机器人运动学参数文档（标准 DH）](https://qcnkr8qd7w8a.feishu.cn/wiki/FFWrwPo7wit4NVkCt7LcTu0DnDd) |
| 动力学 | [机器人动力学参数文档](https://qcnkr8qd7w8a.feishu.cn/wiki/EpqUw7PepiTG4NkGvsdcr5t0nMg) |
| 传感器位姿 | [机器人传感器位姿参数文档](https://qcnkr8qd7w8a.feishu.cn/wiki/I2b1wUOkSikqzekH0FAcWZ6Vnig) |

#### A.2.3 控制系统设计

在参数确定后，形成了控制系统总体设计文档（`robot_control/docs/YLR1D 机器人控制系统.md`），是正文各层设计的早期蓝本。其要点：系统按感知、决策、规划、控制、硬件五层组织；仿真物理层采用 xacro 模型，提供运动学仿真（控制位置，契合 SDK）与动力学仿真（控制力/力矩），并采用 gzserver 不可视化以降低开销；控制层将系统拆分为躯干、左臂、右臂、底座四部分，输入输出与 SDK 一致，内部实现闭环 PID（可扩展为任意离散系统模型）并含限幅与输入调整，夹爪用 0/1 控制、两指只需控制其一；转译层与规划/控制层之间采用 Action 通信，小车由八关节控制封装为运动方式与速度，夹爪由两关节变为一个布尔量；规划层底座依赖 nav2、机械臂依赖 moveit2。文档还记录了需修正的模型问题：左轮速度正时反转、左右臂部分关节反向、夹爪受初始位置限制。项目代码位于 [Zhengshuji/YLR1D_Controller](https://github.com/Zhengshuji/YLR1D_Controller)（分支 `model_fix`）。

#### A.2.4 SDK 接口测试

对 SDK 接口进行了系统测试，测试工程位于 WSL 工作区 `\\wsl.localhost\Ubuntu-22.04\home\zsj\WorkSpace\YLR1D\robot_package\RobotConSys_SDK\example`（基于 `RobotConSysDemo.cpp` 的扩展测试版，覆盖机械臂、底座、夹爪、错误码等用例），结论整理为说明文档（本地 `TR-RD28&&YL-R1D - 开发资料/02_模型转换工具/urdf2dh/xacro2dh/output/docs/YL-R1D说明文档（SDK接口与测试）.md`，在线引用 [YL-R1D 说明文档](https://qcnkr8qd7w8a.feishu.cn/wiki/F5ljwZO4Ziw9uHkl4U3csvC5nhc)）。

SDK 提供 `Joints`（关节）与 `Terminal`（末端）两个变量类型及对应数组类型，声明于 `include/RobSoft/CDataStructure.hpp`；接口能力覆盖系统操作、运动控制（点动/步进/关节空间规划/末端空间规划/轨迹/力矩控制/视觉伺服/夹爪/底座）与外部接口（IO、modbus、外部 TCP、程序执行）。

测试明确的问题有：`terminalJOG` 导致系统断开、`terminalStep` 极易报错、`armAngleJOG/Step` 无效果；直线与圆弧轨迹规划不可用；TCP/IP 直接获取的传感器不含 IMU、超声波、雷达；编译产物为 .so，运行时可能找不到库或函数无法调用，需设置 `LD_LIBRARY_PATH` 或在程序路径建软链接。此外，文档给出机械臂与躯干关节范围、完整错误码表及测试环境配置（WSL 镜像网络、127.0.0.1 回环）。上述结论与正文 translate 层接口设计一致。

#### A.2.5 被控对象数学建模

对被控对象（底座、机械臂、夹爪）进行了数学建模，作为控制系统设计的依据。底座为四轮转向（绕 Z 轴 revolute）+ 车轮驱动（continuous）结构，内部参数为左右轮距 d = 0.320 m、前后轮距 l = 0.426 m、轮径 r = 0.0775 m；静态运动学在纯滚动无侧滑假设下，由轮子接地点速度与转向角建立 `AX = b` 解算车体速度 (Vx, Vy, ω)。SDK 底座仅提供平移、原地旋转（轮向固定 `[2.2150, 0.9265, -2.2150, -0.9265]`，ω = 3.7538·v）、锁死（轮向固定 `[-0.6443, 0.6443, -2.4973, 2.4973]`）三种固定模式，且先转向后移动。机械臂采用 DH 建模；夹爪左爪行程 [-0.014, 0]、右爪行程 [0, 0.014]，两指绕 Z 相差约 180°、运动方向总相反，故只需控制其一。建模结论：SDK 仅提供基于运动学的控制，控制器设计应以前馈为主、反馈为辅；SDK 各部分相对分离，需重点处理部位配合；SDK 内传感器受限，建议增加状态估计器。完整推导见本地 `TR-RD28&&YL-R1D - 开发资料/02_模型转换工具/urdf2dh/xacro2dh/output/docs/被控对象数学建模.md`（在线引用 [被控对象数学建模](https://qcnkr8qd7w8a.feishu.cn/wiki/OzYtwg9zei3K6LkJFtecun2jnrh)）。

#### A.2.6 控制系统验证

在 YLR1D_Controller 项目中，控制子系统采用三层控制架构实现与验证（`robot_control/`，见 `ARCHITECTURE.md`）：上层为运动学/规划层（`ylr1d_upper_control`，待实现），负责底盘麦克纳姆轮正逆解、双臂 IK、躯干运动学、末端笛卡尔空间路径规划与整身协调；中层为指令转发层（`ylr1d_mid_control`，已实现并验证），基于 ros2_control + ForwardCommandController，含 5 个指令控制器（底盘转向 4、底盘驱动 4、躯干 4、左臂 9、右臂 9 关节）与 1 个 joint_state_broadcaster，合计 30 个可控关节，在 Gazebo Classic 中通过 Topic 发布 `Float64MultiArray` 即可控制全部关节；下层为关节驱动层（`ylr1d_lower_control`，待实现），负责各关节 PD 闭环、重力补偿、限位保护与力矩/电流环。验证环境为 ROS2 Humble、WSL Ubuntu 22.04、Gazebo Classic 11。中层验证过程中解决的环境配置问题见 A.3。

### A.3 结论与后续工作

**可复用的部分**：SDK 包（控制机器人的唯一途径）、TR-Sim 仿真环境（无实机联调平台）、例程的传感器标定程序与外部依赖清单。

**必须重做的部分**：SDK 自带 TCP 通信与 ROS1 例程框架均已过时；模型与参数在原始资料中缺失，已分别由 A.2.1、A.2.2 补齐。

**已知问题**：

1. SDK 接口限制（A.2.4）：末端点动/步进会导致断连或报错，直线/圆弧轨迹不可用，TCP/IP 直接获取的传感器不含 IMU、超声波、雷达，.so 需设置 `LD_LIBRARY_PATH` 或建软链接；
2. 中层验证的环境问题（`ylr1d_mid_control/README.md`）：gazebo_ros2_control 插件 CLI 参数解析失败、`LD_LIBRARY_PATH` 未含 `/opt/ros/humble/lib` 致插件不加载、控制器 type 参数未定义、joints 参数为空致控制器 unconfigured，均已解决；
3. 模型方向问题（A.2.3）：左轮速度正时反转、左右臂部分关节反向、夹爪受初始位置限制。

**后续工作**：仿真路线（Gazebo 三层架构）的中层已验证，上层运动学、下层关节驱动、整机协调仍待实现；真机路线（SDK 直驱，附录 B）已通过平替转译/控制/算法/plant 层完成全链路实测，遗留真机投入前的标定项（里程计死推比例、SDK 旋转方向复核、位姿 IK 可达性）。此外，需按建模结论增加状态估计器。

**开发环境与代码仓库**：控制系统基于 ROS2 Humble、WSL Ubuntu 22.04、Gazebo Classic 11（构建工具 colcon）；模型流水线与参数提取基于 Python 3.8+、numpy、PyYAML。自研代码仓库为 [模型流水线](https://github.com/Zhengshuji/Model-Transformation-from-Urdf-Exported-by-SolidWorks)、[urdf_from_step ROS2 移植版](https://github.com/Zhengshuji/urdf_from_step_ros2)、[控制系统 YLR1D_Controller](https://github.com/Zhengshuji/YLR1D_Controller)（分支 `model_fix`）。原始资料归档于 `TR-RD28&&YL-R1D - 副本/`，处理过程与工具归档于 `TR-RD28&&YL-R1D - 开发资料/`。

## B. SDK 封装与真机控制系统

正文第 1-11 章的各层框架基于 Gazebo 仿真（YLR1D_Controller）搭建，其 translate 层（第 6 章）定位为"与 SDK 包对齐"。本附录完成对齐的落地：把厂家 SDK（RobotConSys_Client）**封装为 ROS2 节点**（B.1），并**接入正文搭建的控制系统框架**（B.2），在 SDK 自带仿真（TR-Sim，与真机接口一致）上完成全链路实测。

代码位于 WSL 工作区 `YLR1D_ROS2`（`\\wsl.localhost\Ubuntu-22.04\home\zsj\WorkSpace\YLR1D_ROS2`）：自研包 `robot_interfaces`/`robot_package`/`robot_driver`/`robot_sensors` 与 `bringup` 位于 `src/`，框架（YLR1D_Controller 子模块）位于 `src/ThirdParty/`；RobotConSys 控制器地址 `172.22.224.1:8109`。**总体策略**：框架主体（感知/规划/moveit/HMI/nav2/决策层）原样复用，**驱动层平替「转译＋控制＋算法＋plant」整段**——3 个转译 action（复用 `ylr1d_translate` 类型）＋内联主线程 SDK 调用＋度/弧度换算＋三伺服使能；传感器层把 SDK 状态桥接为框架 30 关节 `/joint_states`，上层零感知差异。

### B.1 SDK 包封装为 ROS2 节点

封装工作由四个包完成：接口包 `robot_interfaces`（自建消息/动作）、SDK 驱动包 `robot_package`（SDK 库＋封装测试 Demo）、驱动层 `robot_driver`（3 action＋SDK 直调）、传感器层 `robot_sensors`（SDK 状态 → 框架关节）。

#### B.1.1 封装策略：dlopen 工厂＋薄封装类

SDK 全部为编译产物（无源码）：`libRobotConSys_Client.so`、`libRobSoft.so`、`libSystemLayer.so`、`libCamDev_Client.so`、`libTCPConDev_Client.so`、`libmodbus*.so`。封装沿用厂家 Demo 的 **dlopen 工厂模式**：`SysLayer::CLoadLibrary` 运行时加载动态库 → `loadFunc("createRobotConSys_Client")` 创建实例 → `init(ip, port)` 连接 → `setAuthority` 设权限 → 下发指令 → `free` 释放。

在此基础上封装为薄类 `RobotSdk`（`robot_driver/include/robot_driver/robot_sdk.hpp`），**所有 SDK 调用收口在此类**，驱动节点只依赖它：

- **单位换算在 SDK 边界统一完成**（实测约定）：臂关节角=**度**、末端位置=**mm**/姿态=**度**、底盘 vx/vy=**mm/s**、wz=**rad/s**；框架/moveit 用弧度＋米 → 下发 rad→deg、上报 deg→rad（mm→m）；
- **库路径由可执行文件位置推算**（`/proc/self/exe` → `install/robot_driver/lib`），不依赖 cwd，规避 A.2.4 的"找不到库"问题；运行库随包安装并以 `$ORIGIN/../lib` RPATH 命中；
- 连接生命周期、伺服使能、权限、结构探测等逻辑均收口在类内。

#### B.1.2 接口包 robot_interfaces

自建接口包（`rosidl_generate_interfaces` 自动发现构建 msg/srv/action）：

- **msg**：`TRArmMsg`（关节角＋末端位姿＋伺服/夹爪状态）、`TRVehicleMsg`（底盘速度/轮速/舵角/超声）、`TRVehicleIMUMsg`；
- **action**：`ArmMoveABSJ`（arm_index＋joints＋vel，对应 SDK `moveABSJoint`）；
- **srv**：按 A.1.2 应用例程中 `robot_package` 的 10 个服务迁移对齐（`MoveABSJ`/`MoveJ`/`MoveL`/`Claw`/`Servo`/`VehicleControl`/`VehicleServo`/`VehicleCommModel`/`CalculateCoordinate`/`ImageToFrame`），保留供后续扩展。

**驱动层 3 个 action 复用框架 `ylr1d_translate` 类型**（`ChassisMove`/`ArmMove`/`GripperMove`）——话题名、消息结构与仿真栈完全一致，规划层/HMI/决策层对接零改动（这是 B.2 接入控制系统的关键）。

#### B.1.3 SDK 驱动包 robot_package

随包携带 SDK 库与头文件（`install/robot_package/lib`），提供 4 个可执行文件作为 **SDK 封装测试的载体与权威参照**：

- `armDemo`：机械臂测试序列（伺服使能 → moveABSJoint → waitMotionCMDFinish → returnZero），20Hz 轮询发布 `/sensors/arm_raw`；
- `vehicleDemo`：底盘测试序列（前进 → 自旋 → 停车），发布 `/sensors/vehicle_raw`，含"SDK 上报恒 0 时回落命令速度"的判定逻辑；
- `camDemo`：相机图像采集（CamDev 库＋OpenCV）；
- `test_node`：最小连通性测试。

Demo 能在 SDK 仿真界面真实驱动设备，是排查驱动层问题的**权威参照**（驱动代码与其逐行对齐，见 B.2.2 根因 1）。

#### B.1.4 驱动层 robot_driver

`robot_driver` 节点是 SDK 封装的对外出口，**平替「转译＋控制＋算法＋plant」**：

- **3 个 action server**（话题名与转译层一致）：`/chassis_move`（mode/direction/speed/duration）→ `setMotionControl`；`/arm_move`（part＋positions）→ `moveABSJoint`＋`waitMotionCMDFinish`；`/gripper_move`（part＋open）→ `setClawState`；
- **20Hz 定时器动作状态机**（同转译层 translate_node 模式），不阻塞 executor；臂运动按线程亲和约束内联主线程执行（B.2.2 根因 1）；
- **状态轮询发布**：`/sensors/arm_raw`（双臂＋躯干，arm0/arm1/arm2）＋`/sensors/vehicle_raw`（底盘，速度按最后命令死推）供传感器层；`/health` 自维护（`OK`/`LOST_CONNECTION`/`NOT_CONNECTED`）；
- **连接生命周期**：2s 断线重连、结构探测（ARM_1..3 的 DOF/伺服/错误码）、三组伺服使能、权限 OPERATOR、可选启动回零；
- **底盘模式**：平移用 `MOVE_XY`（全向 vx/vy，实测 NORMAL 只认 vx/wz、纯横移不动）、旋转 `ROTATE`、停车 `NORMAL(0,0,0)`——与 A.2.5 建模的 SDK 三模式一致。

#### B.1.5 传感器层 robot_sensors

- `sensor_bridge`：订阅 `/sensors/arm_raw`＋`/sensors/vehicle_raw`，组装为**框架 30 关节名 `/joint_states`**（转向=servo_pos、轮=wheel_vel、臂=ARM_1/2、躯干=ARM_3、夹指=夹爪开合映射），关节名单一来源 `ylr1d_description/config/joint_config.hpp`；
- `odom_pub`：**命令速度死推**积分（SDK `getVehicleState` 不回报实际速度）→ `/odom`＋odom→Link_Base TF（真机无 EKF，本节点补齐里程计）。

### B.2 接入控制系统

#### B.2.1 接入架构

真机系统与框架的关系：**真机系统 = 框架的「规划层＋感知层＋HMI＋决策层」原样复用，把「转译层下游的仿真执行链」替换为 SDK 直驱，中间加一层「传感器桥」**（SDK 状态 → 框架 30 关节）。两个"共同语言"保证上层零感知差异：

- **3 个转译 action**（`/chassis_move` `/arm_move` `/gripper_move`）：moveit_bridge/goal_server、nav2 cmd_vel_bridge、决策层 plan_client 全部原样对接；
- **`/joint_states` 30 关节**：感知层（清洗/估计/里程计）、rsp TF、HMI 与仿真栈输入一致。

与正文层级对照：

| 正文层级 | 仿真栈（第 1-11 章） | 真机栈（本附录） |
|---|---|---|
| 决策层 | ylr1d_decision | **原样复用** |
| 规划层 | nav2＋moveit2 | **原样复用**（真机参数微调，B.2.2） |
| 转译层 | 转译 action → /desired_joint_states | **驱动层直调 SDK**（接口仍为同一 3 action） |
| 控制/算法层 | 采样保持＋PID＋A-lite 预测器 | **不存在**（闭环在 SDK/真机控制器内） |
| 物理层 | Gazebo＋plant_sim | **不存在**（TR-Sim/真机） |
| 感知层 | ylr1d_perception | **原样复用**（输入换成传感器桥 /joint_states） |

#### B.2.2 关键适配点（实测根因，均已修复）

1. **SDK 指令线程亲和**（根因 1）：`moveABSJoint`＋`waitMotionCMDFinish` 必须在**发起连接的线程**（executor/主线程）内联执行；worker 线程发出会被服务端忽略/卡死（armDemo 能动正因为指令在 main 线程）。修复：驱动内联主线程执行，回零/臂运动共用。
2. **单位制**（根因 2）：弧度当度发 → 肉眼不可见；SDK 的度当弧度喂 moveit → OMPL 报 invalid bounds（早期"越限"假象）。修复：`robot_sdk.hpp` 边界统一换算（B.1.1）。
3. **底盘无速度反馈**（根因 3）：`getVehicleState` 恒 0 → 里程计按**最后命令速度死推**（仿真忠实于命令，死推与实动一致；真机需标定比例）。
4. **伺服使能**：三组伺服（ARM_1 左臂/ARM_2 右臂/**ARM_3 躯干**）必须 `setServoState(ON)`（含 ARM_3，否则躯干 moveABSJoint 返回 -1）；权限用 OPERATOR（实测 ROOT 下 armDemo 序列臂不动）。
5. **结构映射**：SDK 探测确认 ARM_1 左臂(7)、ARM_2 右臂(7)、ARM_3 躯干(4)；**ARM_4 探测会卡死服务端（勿调用）**；`part 0→ARM_3 / 1→ARM_1 / 2→ARM_2`。
6. **规划层适配**：全栈 `use_sim_time=false`（墙钟，无 /clock）；无激光 → **无 amcl，静态 map=odom**（恒等 TF），地图用 nav_test；moveit_bridge 服务等待重试 180s、moveit_goal_server 段超时 240s（躯干大轨迹 ~178° 瞄准 >90s）；cmd_vel_bridge 速度放大 1.0（真机命令即真实速度）、wz 直通（SDK +wz=逆时针，与 ROS 约定一致）。
7. **进程一致性**：顶层统一注入 `ROS_LOCALHOST_ONLY=1`（loopback DDS，绕开 WSL multicast 慢发现）；单独启动的 CLI/脚本须带同一环境变量，否则跨组互相看不见。

#### B.2.3 启动方式（bringup 包）

```bash
ros2 launch bringup real_robot.launch.py          # 基线：驱动＋传感器＋感知＋rsp
ros2 launch bringup real_robot_manual.launch.py   # ＋hmi_translate（手动 3 action 直发）
ros2 launch bringup real_robot_nav.launch.py      # ＋导航核心＋hmi_plan（NavigateToPose）
ros2 launch bringup real_robot_moveit.launch.py   # ＋moveit＋HMI（等 1-2 分钟 move_group）
ros2 launch bringup real_robot_decision.launch.py # 决策层全栈一键：nav_core＋moveit＋决策＋HMI＋单一 rviz
```

导航核心抽为 `real_robot_nav_core.launch.py`（基线＋odom_pub＋map_server＋静态 map=odom＋nav2 核心＋cmd_vel_bridge），供 nav/decision 两 bringup 共用。

#### B.2.4 测试验证（SDK 仿真实测）

验证脚本位于工作区 `tmp/`（`verify_all.sh`、`run_decision.sh`、`send_mission.py` 等），运行日志 `WSL /tmp/bringup_decision.log` 等。实测结果：

**驱动层**（verify_all.sh 全链路）：底盘平移/原地旋转/横移(MOVE_XY)、左臂/右臂/躯干关节运动、夹爪开合**全部 SUCCEEDED**；关节回显与弧度目标一致（左臂 0.3/-0.2/0.1）；`/joint_states` 30 关节 20-33Hz；结构探测 ARM_1 DOF=7 / ARM_2 DOF=7 / ARM_3 DOF=4，三组伺服使能。

**moveit**：臂关节目标（OMPL 规划 → waypoint → 驱动 → 仿真真实转动）、臂姿态目标（IK）、躯干瞄准（~178° 回转＋俯仰）、夹爪段**全链路 SUCCEEDED**。

**nav**：`NavigateToPose` SUCCESS，底盘实动（/odom 死推位姿随动）。

**决策层**（Mission → BT → plan_client → 规划层 → 驱动 → SDK）：三演示任务实测：

| 任务 | 参数 | 结果 |
|---|---|---|
| arm_move | part=1，+x 5cm | ✅ outcome=0：moveit 规划 10 点 → 3 waypoint，左臂末端 TF 实测 x −0.393 → −0.345 |
| torso_aim | (0.6,0,1.2)（可达） | ✅ outcome=0：规划 37 点 → 17 waypoint 全部到位 |
| torso_aim | (1.0,0,0.6)（不可达） | ❌ outcome=1：倾角盲区 ray_error=0.065>3cm 判不可达（失败路径按预期返回） |
| base_move | 0.6m | ✅ outcome=0：nav2 实动到达，odom (0,0) → (0.42,0.02) |
| base_move | 0.2m | ⚠️ 假成功：nav2 xy_goal_tolerance 0.25m，0.2m 目标判"立即到达"（增量建议 ≥0.5m） |

#### B.2.5 遗留问题

- **真机投入前标定项**：里程计死推比例、SDK +wz 旋转方向复核、位姿 IK 可达性；
- **传感器受限**：SDK 经 TCP 仅回报关节＋底盘状态，相机/雷达/IMU 暂无数据（`/perception/health` 相应 unavailable 属预期）；如需视觉/激光功能需另行接入传感器驱动（A.1.2 例程中的 RealSense/YDLidar 等）；
- **"动作成功"≠"真动"**：SDK 只回显目标状态（`getRobotMotion` 返回期望而非实际），判定以仿真界面/真机实动为准，勿只信日志/状态回显；
- **接口能力约束沿用 A.2.4 测试结论**：仅暴露关节空间控制、底盘三模式、夹爪布尔量——直线/圆弧轨迹、terminalJOG 等不可用接口不暴露给上层。
