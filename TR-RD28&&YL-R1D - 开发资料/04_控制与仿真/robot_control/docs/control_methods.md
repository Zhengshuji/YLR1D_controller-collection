# ylr1d 机器人控制方法

## 控制架构总览

```
                                        ┌─────────────────────────┐
  ┌─────────────┐                      │   Gazebo 仿真环境        │
  │  teleop_key │  /steering_controller/commands                 │
  │  _board     │ ──────────────────►  │   forward_command_       │
  │             │  /drive_controller/commands   │   controller (steering)  │
  │  模式:      │ ──────────────────►  │   forward_command_       │
  │  1 Base     │                      │   controller (drive)     │
  │  2 Body     │  /body_controller/commands                     │
  │  3 L-Arm    │ ──────────────────►  │   forward_command_       │
  │  4 R-Arm    │                      │   controller (body)      │
  │  5 Gripper  │  /left_gripper_controller/commands             │
  │             │ ──────────────────►  │   forward_command_       │
  │             │                      │   controller (L-grip)    │
  │             │  /right_gripper_controller/commands             │
  │             │ ──────────────────►  │   forward_command_       │
  │             │                      │   controller (R-grip)    │
  └─────────────┘                      │                          │
                                       │   joint_trajectory_      │
  ┌─────────────┐   /arm_commander/cmd │   controller (L-arm)     │
  │ arm_        │ ──────────────────►  │                          │
  │ commander   │   FollowJointTraj.   │   joint_trajectory_      │
  │             │ ──────────────────►  │   controller (R-arm)     │
  │ 预设姿势:   │   (action)           │                          │
  │  home       │                      │   joint_state_           │
  │  neutral    │                      │   broadcaster            │
  │  reach      │                      │                          │
  │  fold       │                      └─────────────────────────┘
  └─────────────┘
```

### 控制器分类

| # | 控制器名 | 类型 | 关节数 | 接口类型 | 控制方式 |
|---|---------|------|--------|---------|---------|
| 1 | `steering_controller` | `ForwardCommandController` | 4 | position | Topic |
| 2 | `drive_controller` | `ForwardCommandController` | 4 | velocity | Topic |
| 3 | `body_controller` | `ForwardCommandController` | 4 | position | Topic |
| 4 | `left_arm_controller` | `JointTrajectoryController` | 7 | position | Action |
| 5 | `right_arm_controller` | `JointTrajectoryController` | 7 | position | Action |
| 6 | `left_gripper_controller` | `ForwardCommandController` | 2 | position | Topic |
| 7 | `right_gripper_controller` | `ForwardCommandController` | 2 | position | Topic |
| 8 | `joint_state_broadcaster` | `JointStateBroadcaster` | — | — | 自动发布 |

### 关节明细（30个可控关节）

```
转向 (position)                   驱动 (velocity)
  Joint_Base_to_RFWheelF ────────── Joint_RFWheelF_to_RFWheel
  Joint_Base_to_LFWheelF ────────── Joint_LFWheelF_to_LFWheel
  Joint_Base_to_RBWheelF ────────── Joint_RBWheelF_to_RBWheel
  Joint_Base_to_LBWheelF ────────── Joint_LBWheelF_to_LBWheel

身体 (position)
  Joint_Base_to_Body1   (升降, prismatic, ±0.3m)
  Joint_Body1_to_Body2  (腰部旋转)
  Joint_Body2_to_Body3  (腰部倾斜)
  Joint_Body3_to_Body4  (腰部倾斜)

左臂 (position, 7-DOF)             右臂 (position, 7-DOF)
  Joint_Body2_to_LeftArm1            Joint_Body2_RightArm1
  Joint_LeftArm1_to_LeftArm2         Joint_RightArm1_to_RightArm2
  Joint_LeftArm2_to_LeftArm3         Joint_RightArm2_to_RightArm3
  Joint_LeftArm3_to_LeftArm4         Joint_RightArm3_to_RightArm4
  Joint_LeftArm4_to_LeftArm5         Joint_RightArm4_to_RightArm5
  Joint_LeftArm5_to_LeftArm6         Joint_RightArm5_to_RightArm6
  Joint_LeftArm6_to_LeftArm7         Joint_RightArm6_to_RightArm7

左夹爪 (position)                   右夹爪 (position)
  Joint_LeftArm7_to_LeftFinger1      Joint_RightArm7_to_RightFinger1
  Joint_LeftArm7_to_LeftFinger2      Joint_RightArm7_to_RightFinger2
```

---

## 方法一：键盘遥操作（ylr1d_teleop_keyboard）

运行：
```bash
ros2 run ylr1d_control ylr1d_teleop_keyboard
```
或通过 launch 启动（自带独立终端窗口）：
```bash
ros2 launch ylr1d_control ylr1d_teleop.launch.py
```

### 操作方式

按数字键切换控制模式：

| 按键 | 模式 | 控制内容 |
|------|------|---------|
| `1` | **Base** | 底盘移动（转向+驱动） |
| `2` | **Body** | 身体升降+腰部 |
| `3` | **Left Arm** | 左臂7关节（逐个） |
| `4` | **Right Arm** | 右臂7关节（逐个） |
| `5` | **Gripper** | 左右夹爪 |

#### Base 模式（按 1）

```
     [Q]左转    [W]前进    [E]右转
     [A]左移    [S]后退    [D]右移
```

- 四个麦克纳姆轮实现全向移动
- 前/后：所有舵轮归零，驱动轮同向
- 横移：舵轮转 90°，驱动轮同向
- 旋转：舵轮对角线配置，驱动轮同向

#### Body 模式（按 2）

| 按键 | 动作 | 关节 |
|------|------|------|
| W/S | 升降 ↑↓ | Joint_Base_to_Body1 |
| A/D | 腰部旋转 ←→ | Joint_Body1_to_Body2 |
| Q/E | Body3 倾斜 | Joint_Body2_to_Body3 |
| Z/C | Body4 倾斜 | Joint_Body3_to_Body4 |

#### Arm 模式（按 3=左臂, 4=右臂）

| 按键 | 动作 |
|------|------|
| Tab | 切换当前控制的关节（0→1→...→6→0） |
| W/S | 增加/减少当前关节角度 |
| R | 全部归零（home） |

每个关节的运动范围被限制在 URDF 限位内。

#### Gripper 模式（按 5）

| 按键 | 动作 |
|------|------|
| W/S | 左夹爪 开/合 |
| A/D | 右夹爪 开/合 |
| R | 夹爪复位 |

#### 全局控制

| 按键 | 动作 |
|------|------|
| Space / X | 急停所有电机 |
| `+`/`=` | 加速（×1.0 → ×3.0） |
| `-`/`_` | 减速（×1.0 → ×0.25） |
| H | 显示帮助 |

---

## 方法二：机械臂预设动作（ylr1d_arm_commander）

运行：
```bash
ros2 run ylr1d_control ylr1d_arm_commander
```

### 通过 topic 发送指令

向 `/arm_commander/cmd` 发布 `std_msgs/String` 消息：

```bash
# 左臂回到 home 位姿
ros2 topic pub --once /arm_commander/cmd std_msgs/String "data: 'L:home'"

# 右臂到达 reach 位姿
ros2 topic pub --once /arm_commander/cmd std_msgs/String "data: 'R:reach'"

# 双臂同时到达 neutral 位姿
ros2 topic pub --once /arm_commander/cmd std_msgs/String "data: 'LR:neutral'"
```

### 预设姿势

| 名称 | 用途 | 描述 |
|------|------|------|
| `home` | 初始/归零 | 所有关节 = 0 |
| `neutral` | 中立 | 肩部放松，肘部微曲 |
| `reach` | 前伸 | 手臂前伸抓取姿态 |
| `fold` | 折叠 | 手臂折叠收纳 |

运动时间为 2 秒（`time_from_start = 2.0s`），轨迹自动插值。

---

## 方法三：ROS 2 CLI 直接控制

直接向控制器的 command topic 发布消息，适合调试和脚本化控制。

### 转向控制（position）

```bash
# 4个舵轮转 0.5 rad（约 28°）
ros2 topic pub --once /steering_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5]}"
```

### 驱动控制（velocity）

```bash
# 4个驱动轮以 2.0 rad/s 前进
ros2 topic pub --once /drive_controller/commands \
  std_msgs/Float64MultiArray "{data: [2.0, 2.0, 2.0, 2.0]}"

# 停止
ros2 topic pub --once /drive_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
```

### 身体控制（position）

```bash
# 升降台升起 0.1m，腰部旋转 0.2 rad
ros2 topic pub --once /body_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.1, 0.2, 0.0, 0.0]}"
```

### 夹爪控制（position）

```bash
# 左夹爪合拢（含指1、指2）
ros2 topic pub --once /left_gripper_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.02, 0.02]}"

# 右夹爪张开
ros2 topic pub --once /right_gripper_controller/commands \
  std_msgs/Float64MultiArray "{data: [-0.02, -0.02]}"
```

### 机械臂轨迹控制（action）

使用 `ros2 action send_goal` 发送轨迹：

```bash
# 左臂到达指定关节角度
ros2 action send_goal /left_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [
        'Joint_Body2_to_LeftArm1',
        'Joint_LeftArm1_to_LeftArm2',
        'Joint_LeftArm2_to_LeftArm3',
        'Joint_LeftArm3_to_LeftArm4',
        'Joint_LeftArm4_to_LeftArm5',
        'Joint_LeftArm5_to_LeftArm6',
        'Joint_LeftArm6_to_LeftArm7'
      ],
      points: [{
        positions: [0.5, 0.5, -0.8, -0.5, 0.3, 0.0, 0.0],
        time_from_start: {sec: 2, nanosec: 0}
      }]
    }
  }"
```

### 查看关节状态

```bash
# 所有关节位置
ros2 topic echo /joint_states

# 单个控制器状态
ros2 control view_controller_list
```

---

## 方法四：编程接口（C++）

### Keyboard Teleop 节点

参考 `src/ylr1d_teleop_keyboard.cpp` 中的 `Ylr1dTeleopKeyboard` 类：

- 创建 `rclcpp::Publisher<std_msgs::msg::Float64MultiArray>` 到对应 controller topic
- 用 `data.assign(N, 0.0)` 初始化数组
- 发布即生效

```cpp
// 示例：发布驱动指令
auto pub = create_publisher<Float64MultiArray>("/drive_controller/commands", 1);
auto msg = Float64MultiArray();
msg.data = {2.0, 2.0, 2.0, 2.0};
pub->publish(msg);
```

### Arm Commander 节点

参考 `src/ylr1d_arm_commander.cpp` 中的 `Ylr1dArmCommander` 类：

- 创建 `rclcpp_action::Client<FollowJointTrajectory>`
- 构建 `FollowJointTrajectory::Goal`，设置 `joint_names` 和 `trajectory.points`
- 调用 `async_send_goal()`

```cpp
// 示例：发送左臂轨迹目标
auto ac = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/left_arm_controller/follow_joint_trajectory");

auto goal = FollowJointTrajectory::Goal();
goal.trajectory.joint_names = {"Joint_Body2_to_LeftArm1", ...};
auto point = JointTrajectoryPoint();
point.positions = {0.5, 0.5, -0.8, -0.5, 0.3, 0.0, 0.0};
point.time_from_start = rclcpp::Duration::from_seconds(2.0);
goal.trajectory.points.push_back(point);
ac->async_send_goal(goal);
```

---

## 启动顺序

```bash
# 1. 启动 Gazebo 仿真（含所有控制器）
ros2 launch ylr1d_control ylr1d_control_gazebo.launch.py

# 2. 启动键盘遥操作 + 机械臂指挥官
ros2 launch ylr1d_control ylr1d_teleop.launch.py

# 3. 如果控制器管理器重启了，单独加载控制器
ros2 launch ylr1d_control ylr1d_load_controllers.launch.py
```

**初始化时间线：**
- 0s → Gazebo 启动、robot_state_publisher 启动、controller_manager 就绪
- 6s → 机器人模型放入仿真场景
- 8s → 所有 8 个控制器加载并激活

---

## 常见问题排查

### 机器人"软趴趴"或不受控

1. **检查控制器是否已激活：**
   ```bash
   ros2 control list_controllers
   ```
   应显示所有 8 个控制器状态为 `active`。

2. **检查 joint_states 是否发布：**
   ```bash
   ros2 topic echo /joint_states
   ```

3. **增大 PID 增益：** 如果机器人仍然无力，可以增大 `ylr1d_ros2_control.xacro` 中的 kp 值。

### 控制器加载失败

检查 `ros2_control_node` 节点是否正常运行，确认 `$(find ...)` 路径已正确解析为 xacro mappings。
