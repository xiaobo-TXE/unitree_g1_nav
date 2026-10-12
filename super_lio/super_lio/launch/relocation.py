import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    pkg_livox = get_package_share_directory('livox_ros_driver2')
    # -------- livox config --------
    user_config_path = os.path.join(pkg_livox, 'config', 'MID360_config.json')

    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'livox_360.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'relocation.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    super_lio_node = Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[config_yaml],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # -------- livox driver parameters --------
    livox_ros2_params = [
        {"xfer_format": 1},                 # 0-PointCloud2, 1-customized pointcloud
        {"multi_topic": 0},                # 0-all LiDARs share one topic
        {"data_src": 0},                   # 0-lidar
        {"publish_freq": 10.0},
        {"output_data_type": 0},
        {"frame_id": "livox_frame"},
        {"lvx_file_path": "/home/livox/livox_test.lvx"},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": "livox0000000001"}
    ]

    # -------- livox driver node --------
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag)
    )

    # -------- delayed super_lio start --------
    delayed_super_lio_node = TimerAction(
        period=1.0,
        actions=[super_lio_node]
    )

    map_to_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )

    return LaunchDescription([
        declare_rviz_arg,
        map_to_world_tf,
        livox_driver_node,
        delayed_super_lio_node
        # rviz2_node
    ])