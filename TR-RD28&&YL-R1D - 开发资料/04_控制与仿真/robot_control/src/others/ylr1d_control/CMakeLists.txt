cmake_minimum_required(VERSION 3.8)
project(ylr1d_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_ROS_INCLUDE_DIRS})

# ── Keyboard teleop node ──────────────────────────────────────────
add_executable(ylr1d_teleop_keyboard
  src/ylr1d_teleop_keyboard.cpp
)
ament_target_dependencies(ylr1d_teleop_keyboard
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

# ── Arm commander node ───────────────────────────────────────────
add_executable(ylr1d_arm_commander
  src/ylr1d_arm_commander.cpp
)
ament_target_dependencies(ylr1d_arm_commander
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
)

# ── PID controller node ─────────────────────────────────────────
add_executable(ylr1d_pid_controller
  src/ylr1d_pid_controller.cpp
)
ament_target_dependencies(ylr1d_pid_controller
  rclcpp
  std_msgs
  sensor_msgs
)

# ── Gazebo joint effort plugin ──────────────────────────────
add_library(ylr1d_joint_effort_plugin SHARED
  src/ylr1d_joint_effort_plugin.cpp
)
ament_target_dependencies(ylr1d_joint_effort_plugin
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(ylr1d_joint_effort_plugin
  ${GAZEBO_LIBRARIES}
  ${GAZEBO_COMMON_LIBRARIES}
)

# ── Install targets ──────────────────────────────────────────────
install(TARGETS
  ylr1d_teleop_keyboard
  ylr1d_arm_commander
  ylr1d_pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ylr1d_joint_effort_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf/    DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
