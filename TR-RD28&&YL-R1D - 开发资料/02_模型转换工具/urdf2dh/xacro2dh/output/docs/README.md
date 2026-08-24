# 机器人参数与建模文档（归档）

本目录为 YLR1D 机器人**参数文档、建模文档与 SDK 测试结论**的本地权威版本，内容与飞书在线文档同步（飞书为在线评审副本，本目录为权威数据源）。

## 文档清单

| 文档 | 内容 | 对应在线文档（飞书） | 对应 YAML |
|---|---|---|---|
| `机器人运动学参数文档（标准 DH）.md` | 标准 DH（Craig）约定、left_arm/right_arm/body 三条链的 base/tool 变换与 DH 参数表、FK 公式 | [运动学参数文档](https://qcnkr8qd7w8a.feishu.cn/wiki/FFWrwPo7wit4NVkCt7LcTu0DnDd) | `../kinematics.yaml` |
| `机器人动力学参数文档.md` | 逐 link 质量、质心、惯量张量（底座/车身/车轮/双臂/夹爪/传感器） | [动力学参数文档](https://qcnkr8qd7w8a.feishu.cn/wiki/EpqUw7PepiTG4NkGvsdcr5t0nMg) | `../dynamics.yaml` |
| `机器人传感器位姿参数文档.md` | 15 个传感器（全局相机×3、左右手相机×3×2、雷达、IMU、四角超声波）的 mount 变换与零位形位姿 | [传感器位姿参数文档](https://qcnkr8qd7w8a.feishu.cn/wiki/I2b1wUOkSikqzekH0FAcWZ6Vnig) | `../sensors.yaml` |
| `被控对象数学建模.md` | 底座/机械臂/夹爪建模、SDK 底座运动模式、夹爪行程、建模结论 | [被控对象数学建模](https://qcnkr8qd7w8a.feishu.cn/wiki/OzYtwg9zei3K6LkJFtecun2jnrh) | — |
| `YL-R1D说明文档（SDK接口与测试）.md` | SDK 数据类型/接口能力、测试发现的问题（terminalJOG 断开、直线圆弧不可用、传感器限制等）、关节范围与错误码表、测试环境 | [YL-R1D 说明文档](https://qcnkr8qd7w8a.feishu.cn/wiki/F5ljwZO4Ziw9uHkl4U3csvC5nhc) | — |

## 生成方式

- 参数文档由 `xacro2dh`（`..\xacro2dh\run.py`）从 `urdf/ylr1d.xacro` 与 `config/` 自动提取生成，并做 FK 交叉验证；
- 数据直接来自展开后的 URDF，未作修改；
- SDK 测试工程位于 WSL 工作区 `robot_package/RobotConSys_SDK/example`（`RobotConSysDemo.cpp` 扩展测试版），测试环境见说明文档第 8 节；
- 飞书文档为在线评审副本，若链接失效或内容不一致，以本目录文件为准。

## 相关文档

- 控制系统总体设计：`../../../../04_控制与仿真/robot_control/docs/YLR1D 机器人控制系统.md`（正文框架蓝本，对应 [Zhengshuji/YLR1D_Controller](https://github.com/Zhengshuji/YLR1D_Controller)）

