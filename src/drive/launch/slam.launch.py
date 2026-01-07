import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Setup the Launch Configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_name = 'drive'
    
    # 2. Define the argument (We set default to 'true' to fix your timestamp error)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 3. Path to config
    # Ensure your config file is named exactly this in your src/drive/config folder
    slam_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        # Always run this declaration first
        declare_use_sim_time_cmd,
        
        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config_path,
                {'use_sim_time': use_sim_time} # Takes value from the argument above
            ],
        ),
    ])