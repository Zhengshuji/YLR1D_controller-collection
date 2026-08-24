"""Load ros2_control controllers only (assumes Gazebo + robot already running).

Use this when you launched the robot from ylr1d_description and want to
attach controllers separately, or after restarting the controller manager.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controllers = [
        "joint_state_broadcaster",
        "steering_controller",
        "drive_controller",
        "body_controller",
        "left_arm_controller",
        "right_arm_controller",
        "left_gripper_controller",
        "right_gripper_controller",
    ]

    spawners = []
    for ctrl in controllers:
        spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[ctrl],
                output="screen",
            )
        )
    return LaunchDescription(spawners)
