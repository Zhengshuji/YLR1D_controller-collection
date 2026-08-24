# 在 WSL (Ubuntu 22.04) 终端中逐条执行

# ─── 1. 准备工作区 ───
source /opt/ros/humble/setup.bash
mkdir -p ~/ws_gazebo_patch/src
cd ~/ws_gazebo_patch/src

# ─── 2. 克隆源码 ───
git clone https://github.com/ros-controls/gazebo_ros2_control.git -b humble

# ─── 3. 复制补丁 ───
cp /mnt/d/Path/tmp/robot_control/src/others/fix_gazebo_ros2_control.patch ~/ws_gazebo_patch/src/gazebo_ros2_control/

# ─── 4. 应用补丁 ───
cd ~/ws_gazebo_patch/src/gazebo_ros2_control
git apply fix_gazebo_ros2_control.patch
echo "补丁应用成功！"

# ─── 5. 安装构建依赖 ───
sudo apt update
sudo apt install -y python3-colcon-common-extensions ros-dev-tools 2>/dev/null
cd ~/ws_gazebo_patch
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

# ─── 6. 编译 ───
cd ~/ws_gazebo_patch
colcon build --packages-up-to gazebo_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "编译完成！"

# ─── 7. 安装（替换原插件）───
sudo cp /opt/ros/humble/lib/libgazebo_ros2_control.so /opt/ros/humble/lib/libgazebo_ros2_control.so.bak
sudo cp ~/ws_gazebo_patch/install/gazebo_ros2_control/lib/libgazebo_ros2_control.so /opt/ros/humble/lib/
echo "安装完成！"

# ─── 8. 验证 ───
ls -la /opt/ros/humble/lib/libgazebo_ros2_control.so
md5sum /opt/ros/humble/lib/libgazebo_ros2_control.so
echo "新旧MD5不一样则替换成功"
