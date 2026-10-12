"""Super-LIO with map localization (MapLoc).

IMPORTANT: unlike relocation.py, this launch does NOT publish a static
map -> world transform. map_localization_node broadcasts that transform
dynamically -- it *is* the drift estimate T_M_O -- and a static identity
publisher would fight it, producing a TF tree that flickers between the two.
"""
import os

from ament_index_python.packages import (PackageNotFoundError,
                                          get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')

    # Resolved lazily: replaying a bag (livox:=false) is a first-class use of
    # this launch file and must work on a machine that has no Livox driver
    # installed. Looking the package up unconditionally made the file fail to
    # load at all in that case.
    try:
        pkg_livox = get_package_share_directory('livox_ros_driver2')
        user_config_path = os.path.join(pkg_livox, 'config', 'MID360_config.json')
    except PackageNotFoundError:
        user_config_path = ''

    config_yaml = os.path.join(pkg_super_lio, 'config', 'map_localization.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'map_localization.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false', description='Whether to start RVIZ2')
    declare_livox_arg = DeclareLaunchArgument(
        'livox', default_value='true',
        description='Start the Livox driver (set false when replaying a bag)')
    declare_map_arg = DeclareLaunchArgument(
        'map_dir', default_value='',
        description='Override lio.maploc.map_dir from the command line')
    declare_config_arg = DeclareLaunchArgument(
        'config', default_value='',
        description='Use this params file instead of the installed '
                    'config/map_localization.yaml (absolute path)')

    rviz_flag = LaunchConfiguration('rviz')
    livox_flag = LaunchConfiguration('livox')

    def make_localization_node(context, *_args, **_kwargs):
        """Build the node with the map_dir override applied ONLY if one was given.

        The override cannot be a plain entry in `parameters`. launch_ros writes
        every dict there into its own temporary params file and passes it after
        the yaml, and rclcpp applies params files in order with the last one
        winning -- so an unset `map_dir:=` wrote an EMPTY STRING over the yaml's
        path and the node aborted at startup with "map_dir is empty". There is
        no "empty means leave it alone" rule in ROS 2: an empty string is a
        perfectly good value that overwrites. Deciding here, where the argument
        has actually been resolved to text, is the only way to leave the yaml
        untouched.
        """
        # Which params file, and SAY SO. The node is launched with the copy in
        # install/share, not the one in src/ -- `install(DIRECTORY config/ ...)`
        # copies it at build time -- so editing src/ without rebuilding changes
        # nothing, and editing install/ is undone by the next build. On a robot
        # whose config is its own (sync-real.sh excludes map_localization.yaml)
        # that round trip is pure friction, so `config:=/path/to.yaml` points
        # straight at a file and skips it.
        chosen = LaunchConfiguration('config').perform(context)
        if chosen:
            if not os.path.isfile(chosen):
                raise RuntimeError(
                    'config:={} does not exist'.format(chosen))
            params = [chosen]
        else:
            params = [config_yaml]
        print('[map_localization] params file: {}'.format(params[0]))

        override = LaunchConfiguration('map_dir').perform(context)
        if override:
            params.append({'lio.maploc.map_dir': override})

        node = Node(
            package='super_lio',
            executable='map_localization_node',
            name='map_localization_node',
            # 'both', not 'screen': screen alone makes ros2 launch keep no
            # per-process file, so anything not going through glog (rclcpp's
            # own messages, a crash trace) is lost when the terminal closes.
            output='both',
            parameters=params,
            arguments=['--ros-args', '--log-level', 'info'],
        )
        # Give the driver a moment so the first IMU batch is buffered before the
        # frontend starts its static-initialization window.
        return [TimerAction(period=1.0, actions=[node])]

    livox_ros2_params = [
        {"xfer_format": 1},
        {"multi_topic": 0},
        {"data_src": 0},
        {"publish_freq": 10.0},
        {"output_data_type": 0},
        {"frame_id": "livox_frame"},
        {"lvx_file_path": "/home/livox/livox_test.lvx"},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": "livox0000000001"},
    ]

    # On a non-Livox robot pass livox:=false; the driver would otherwise publish
    # onto a topic nothing reads.
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition=IfCondition(livox_flag),
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='map_localization_rviz',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag),
    )

    return LaunchDescription([
        declare_rviz_arg,
        # declare_livox_arg,
        declare_map_arg,
        declare_config_arg,
        # livox_driver_node,
        OpaqueFunction(function=make_localization_node),
        rviz2_node,
    ])
