import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
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

    robot_description = {'robot_description': robot_description_content}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )

    world_file = os.path.join(pkg_drive, 'worlds', 'complex_maze.sdf')

    declare_world = DeclareLaunchArgument(
        'world', default_value=world_file,
    )

    map_file = os.path.join(pkg_drive, 'maps', 'maze_map1.yaml')

    declare_map = DeclareLaunchArgument(
        'map', default_value=map_file,
        description='Full path to the map yaml file',
    )

    nav2_params_file = os.path.join(pkg_drive, 'config', 'nav2_params.yaml')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=nav2_params_file,
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

    # --- Bridges (delayed 2s to let Gazebo start the clock) ---
    ros_gz_bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
            ),
        ],
    )

    lidar_bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='lidar_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            ),
        ],
    )

    # --- EKF (delayed 4s — needs clock + odom + imu to be flowing) ---
    ekf_config = os.path.join(pkg_drive, 'config', 'ekf.yaml')

    ekf_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    # --- Nav2 bringup (delayed 8s — needs clock + EKF odom->base_link TF) ---
    nav2_bringup = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'use_sim_time': 'true',
                    'autostart': 'true',
                }.items(),
            ),
        ],
    )

    # --- RViz2 (delayed 10s) ---
    rviz_config_file = os.path.join(pkg_drive, 'rviz', 'nav2_config.rviz')

    rviz2_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_map,
        declare_params,
        set_gz_resource_path,
        ignition_gazebo,
        robot_state_publisher_node,
        spawn_robot,
        ros_gz_bridge,
        lidar_bridge,
        ekf_node,
        nav2_bringup,
        rviz2_node,
    ])