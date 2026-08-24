#!/bin/bash
# 在 WSL (Ubuntu 22.04) 中执行此脚本
# 构建打了补丁的 gazebo_ros2_control 插件

set -e

WORKSPACE=~/ws_gazebo_patch
SRC_DIR=$WORKSPACE/src
PATCH_FILE="/path/to/fix_gazebo_ros2_control.patch"  # 需要修改为实际路径

# 1. 安装构建依赖
sudo apt update
sudo apt install -y python3-vcstool python3-colcon-common-extensions

# 2. 克隆源码
mkdir -p $SRC_DIR
cd $SRC_DIR
if [ ! -d "gazebo_ros2_control" ]; then
    git clone https://github.com/ros-controls/gazebo_ros2_control.git -b humble
fi

# 3. 应用补丁
cd $SRC_DIR/gazebo_ros2_control
git checkout humble
cp "$PATCH_FILE" ./fix.patch
git apply ./fix.patch
echo "补丁已应用！"

# 4. 安装 rosdep 依赖
cd $WORKSPACE
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || echo "rosdep 完成（可忽略警告）"

# 5. 编译
cd $WORKSPACE
colcon build --packages-up-to gazebo_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release

# 6. 备份原插件并安装新插件
echo "备份原插件..."
sudo mv /opt/ros/humble/lib/libgazebo_ros2_control.so /opt/ros/humble/lib/libgazebo_ros2_control.so.bak
echo "安装新插件..."
sudo cp $WORKSPACE/install/gazebo_ros2_control/lib/libgazebo_ros2_control.so /opt/ros/humble/lib/
echo "完成！"
