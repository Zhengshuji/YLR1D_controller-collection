"""Launch the ylr1d teleop keyboard and arm commander nodes."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_keyboard = Node(
        package="ylr1d_control",
        executable="ylr1d_teleop_keyboard",
        output="screen",
        prefix="xterm -e",    # run in its own terminal window
    )

    arm_commander = Node(
        package="ylr1d_control",
        executable="ylr1d_arm_commander",
        output="screen",
    )

    return LaunchDescription([
        teleop_keyboard,
        arm_commander,
    ])
