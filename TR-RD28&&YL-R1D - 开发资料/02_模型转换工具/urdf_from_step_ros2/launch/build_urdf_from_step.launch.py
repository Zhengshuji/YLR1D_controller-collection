#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明命令行参数
    declare_step_file_path = DeclareLaunchArgument(
        'step_file_path',
        default_value='/model/input/YLR1D.STEP',
        description='Path to the input STEP file'
    )

    declare_output_folder_path = DeclareLaunchArgument(
        'output_folder_path',
        default_value='/model/output',
        description='Directory where the URDF and meshes will be saved'
    )

    declare_urdf_package_name = DeclareLaunchArgument(
        'urdf_package_name',
        default_value='YLR1D_urdf',
        description='Name used for the output URDF file (and internal package reference)'
    )

    # 启动节点
    urdf_creator_node = Node(
        package='urdf_from_step_ros2',
        executable='create_urdf.py',          # 对应 scripts/ 中的可执行脚本名
        name='urdf_creator',
        output='screen',
        # 通过 parameters 传递参数给节点（ROS2 参数机制）
        parameters=[{
            'step_file_path': LaunchConfiguration('step_file_path'),
            'output_folder_path': LaunchConfiguration('output_folder_path'),
            'urdf_package_name': LaunchConfiguration('urdf_package_name'),
        }],
        # 如果节点设计为一次性任务而非持续运行，添加 'required' 效果可配合 on_exit 行为，但 ROS2 Node 无 required 参数，可用 EmitEvent 或 OnProcessExit 替代，但通常不必要
    )

    return LaunchDescription([
        declare_step_file_path,
        declare_output_folder_path,
        declare_urdf_package_name,
        urdf_creator_node,
    ])