import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_drive = get_package_share_directory('drive')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf_file = os.path.join(pkg_drive, 'urdf', 'drive3.urdf')
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    robot_description_content = robot_description_content.replace('$(find drive)', pkg_drive)

    robot_description = {'robot_description': robot_description_content}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )

    world_file = os.path.join(pkg_drive, 'worlds', 'complex_maze.sdf')

    declare_world = DeclareLaunchArgument(
        'world', default_value=world_file,
    )

    nav2_params_file = os.path.join(pkg_drive, 'config', 'nav2_params.yaml')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=nav2_params_file,
    )

    # Path to the RTAB-Map database saved during mapping
    declare_database = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.join(os.path.expanduser('~'), '.ros', 'rtabmap.db'),
        description='Path to the RTAB-Map database file for localization',
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_drive, '..'),
    )

    # --- Ignition Gazebo ---
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world')],
        }.items(),
    )

    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # --- Spawn robot ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_drive_rover',
        output='screen',
        arguments=[
            '-name', 'drive_rover',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5', '-Y', '0.0',
        ],
    )

    # --- Bridges ---
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # Depth camera bridges
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
    )

    # ros2_control controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- EKF ---
    ekf_config = os.path.join(pkg_drive, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # --- RTAB-Map (LOCALIZATION mode) ---
    rtabmap_config = os.path.join(pkg_drive, 'config', 'rtabmap.yaml')

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'use_sim_time': use_sim_time,
                # Localization mode: do NOT add new data to the map
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
            },
        ],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth_image'),
            ('odom', '/odom'),
        ],
    )

    # --- Nav2 bringup (no AMCL, no map_server — RTAB-Map provides /map + TF) ---
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    # --- RViz2 ---
    rviz_config_file = os.path.join(pkg_drive, 'rviz', 'nav2_config.rviz')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Wait for the controller spawner to exit before starting EKF, RTAB-Map, Nav2, and RViz
    delayed_nodes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[ekf_node, rtabmap_node, nav2_bringup, rviz2_node]
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_params,
        declare_database,
        set_gz_resource_path,
        ignition_gazebo,
        robot_state_publisher_node,
        spawn_robot,
        ros_gz_bridge,
        camera_bridge,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        delayed_nodes,
    ])