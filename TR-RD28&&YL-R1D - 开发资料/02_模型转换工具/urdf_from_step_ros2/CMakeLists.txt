cmake_minimum_required(VERSION 3.5)
project(urdf_from_step_ros2)

# 查找 ament_cmake（必要）
find_package(ament_cmake REQUIRED)

# 不要调用 ament_python_install_package()
# 所有 Python 安装由 setup.py 负责

# 如果还有 launch 文件或其他资源，可以在 CMake 中安装，但更推荐在 setup.py 中处理
# 除非你有非 Python 文件（如 C++ 库），否则不必在 CMake 中安装任何东西

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()