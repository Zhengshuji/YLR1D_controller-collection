#!/usr/bin/env python3
"""Apply the patch to gazebo_ros2_control_plugin.cpp"""

import sys

source_path = sys.argv[1]

with open(source_path) as f:
    lines = f.readlines()

# Find the target lines
target_start = None
target_end = None
for i, line in enumerate(lines):
    if 'std::string rb_arg = std::string("robot_description:=")' in line:
        target_start = i
    if target_start is not None and i >= target_start + 4:
        target_end = i
        break

if target_start is None:
    print("ERROR: Could not find target code")
    sys.exit(1)

print(f"Found target at lines {target_start}-{target_end}")

# Build new content
replacement = [
    '  // Set robot_description directly as a node parameter instead of passing\n',
    '  // it as a CLI --param argument. The CLI approach fails when the URDF XML\n',
    '  // contains special characters (<?, <, >, ", &) that rcl\'s YAML-based\n',
    '  // parameter parser cannot handle.\n',
    '  {\n',
    '    auto node = std::dynamic_pointer_cast<rclcpp::Node>(impl_->model_nh_);\n',
    '    try {\n',
    '      if (node->has_parameter("robot_description")) {\n',
    '        node->set_parameter(rclcpp::Parameter("robot_description", urdf_string));\n',
    '      } else {\n',
    '        node->declare_parameter("robot_description", urdf_string);\n',
    '      }\n',
    '    } catch (const std::exception & e) {\n',
    '      RCLCPP_ERROR(impl_->model_nh_->get_logger(), "Failed to set robot_description parameter: %s", e.what());\n',
    '    }\n',
    '  }\n',
]

# Remove old lines and insert new ones
new_lines = lines[:target_start-1] + replacement + lines[target_end:]

with open(source_path, 'w') as f:
    f.writelines(new_lines)

print(f"Patch applied: replaced {target_end - target_start + 1} lines with {len(replacement)} lines")
