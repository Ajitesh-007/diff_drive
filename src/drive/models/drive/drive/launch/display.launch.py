import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('drive'),
        'urdf',
        'drive.urdf'
    )
    
    # Read the URDF file contents
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Path to your saved RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('drive'),
        'rviz',
        'view_robot.rviz'
    )

    # --- Nodes ---

    # Publishes TF and robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Interactive joint sliders
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Launch RViz2 with the preloaded configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

