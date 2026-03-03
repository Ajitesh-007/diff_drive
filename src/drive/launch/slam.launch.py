import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_drive = get_package_share_directory('drive')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    slam_toolbox_config = os.path.join(
        pkg_drive, 'config', 'slam_toolbox.yaml'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_node,
    ])