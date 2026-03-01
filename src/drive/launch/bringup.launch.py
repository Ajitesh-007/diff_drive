import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_name = 'drive'
    drive_share = get_package_share_directory(pkg_name)

    # 1. Launch Configurations & Arguments
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Mode to run: "mapping" (SLAM) or "navigation" (AMCL)'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(drive_share, 'maps', 'maze_map.yaml'),
        description='Full path to map yaml file to load (only used in navigation mode)'
    )

    # Conditions for mapping vs navigation
    is_mapping = PythonExpression(["'", mode, "' == 'mapping'"])
    is_navigation = PythonExpression(["'", mode, "' == 'navigation'"])

    # 2. PATHS
    gazebo_urdf_file_path = os.path.join(drive_share, 'urdf', 'drive3.urdf')
    ekf_config_path = os.path.join(drive_share, 'config', 'ekf.yaml')
    maze_world_path = "/home/ajitesh/ros2_ws/src/drive/worlds/complex_maze.sdf"
    slam_launch_file_path = os.path.join(drive_share, 'launch', 'slam.launch.py')
    
    # Nav2 paths
    nav2_params_path = os.path.join(drive_share, 'config', 'nav2_params.yaml')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 3. READ URDF
    with open(gazebo_urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 4. ENVIRONMENT SETUP
    set_gl_software = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')

    # 5. IGNITION GAZEBO
    ignition = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', maze_world_path],
        output='screen',
        additional_env={'OGRE_RTT_MODE': 'Copy'}
    )
    
    # 6. ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True 
        }],
    )

    # 7. ROS GZ BRIDGE
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock', 
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/drive/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            ('/model/drive/odometry', '/odom'),
        ],
        output='screen'
    )

    # 8. EKF NODE (Odometry Fusion)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path, 
            {'use_sim_time': True} 
        ],
        remappings=[('/odometry/filtered', '/odom_filtered')]
    )

    # 9. RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 10. SPAWN ROBOT IN GAZEBO
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc, '-name', 'drive', '-x', '0.0', '-y', '0.0', '-z', '0.5'],
    )
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])

    # ===============================
    # CONDITIONAL LAUNCHES
    # ===============================

    # A: MAPPING MODE (SLAM + Basic Nav2 Navigation without AMCL)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file_path),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(is_mapping)
    )

    # Note: In Mapping mode, we use navigation_launch which excludes Map Server and AMCL 
    # because SLAM is acting as the localization and mapping source.
    mapping_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items(),
        condition=IfCondition(is_mapping)
    )

    delayed_mapping_nav = TimerAction(period=10.0, actions=[mapping_nav2_launch])

    # B: NAVIGATION MODE (Map Server + AMCL + Nav2 Navigation)
    # bringup_launch.py brings up Map Server, AMCL, and Navigation
    navigation_full_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path,
            'map': map_yaml,
            'autostart': 'true'
        }.items(),
        condition=IfCondition(is_navigation)
    )

    delayed_full_nav = TimerAction(period=15.0, actions=[navigation_full_launch])

    # ===============================

    return LaunchDescription([
        mode_arg,
        map_arg,
        set_gl_software,
        ignition,
        robot_state_publisher,
        bridge,
        ekf_node,
        delayed_spawn,
        rviz_node,
        # conditional mapping drops here
        slam_launch,
        delayed_mapping_nav,
        # conditional navigation drops here
        delayed_full_nav
    ])