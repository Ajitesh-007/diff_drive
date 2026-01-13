import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Package Directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    drive_pkg_dir = get_package_share_directory('drive')

    # 2. Path to Config
    nav2_params_file = os.path.join(drive_pkg_dir, 'config', 'nav2_params.yaml')

    # 3. Include Standard Nav2 Launch
    # We use 'navigation_launch.py' because SLAM is already handling the Map Server
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])