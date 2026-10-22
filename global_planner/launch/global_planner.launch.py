from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('global_planner')
    config_file = os.path.join(package_share, 'config', 'global_planner.yaml')

    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/unitree/nav_ws/src/global_planner/map/trav_map.pcd',
        description='Traversability PCD containing x, y, z and intensity fields',
    )
    map_frame_arg = DeclareLaunchArgument(
        'map_frame', default_value='map', description='Planning frame',
    )

    planner_node = Node(
        package='global_planner',
        executable='global_planner_node',
        name='global_planner',
        output='screen',
        parameters=[
            config_file,
            {
                'map_path': LaunchConfiguration('pcd_file'),
                'map_frame': LaunchConfiguration('map_frame'),
            },
        ],
    )

    return LaunchDescription([pcd_file_arg, map_frame_arg, planner_node])
