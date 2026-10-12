"""Launch the velocity bridge and wireless-controller reader."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start g1_bridge nodes."""
    return LaunchDescription([
        Node(
            package='g1_bridge',
            executable='cmd_vel_bridge',
            name='g1_cmd_vel_bridge',
            output='screen',
        ),
        Node(
            package='g1_bridge',
            executable='wireless_controller',
            name='g1_wireless_controller',
            output='screen',
        ),
    ])
