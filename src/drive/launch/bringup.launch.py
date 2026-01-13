import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'drive'
    drive_share = get_package_share_directory(pkg_name)

    # 1. PATHS
    gazebo_urdf_file_path = os.path.join(drive_share, 'urdf', 'drive3.urdf')
    ekf_config_path = os.path.join(drive_share, 'config', 'ekf.yaml')
    maze_world_path = "/home/ajitesh/ros2_ws/src/drive/worlds/complex_maze.sdf"
    slam_launch_file_path = os.path.join(drive_share, 'launch', 'slam.launch.py')
    
    # --- PATH TO NAV2 PARAMS & LAUNCH ---
    nav2_params_path = os.path.join(drive_share, 'config', 'nav2_params.yaml')
    nav2_launch_dir = get_package_share_directory('nav2_bringup')

    # 2. READ URDF
    with open(gazebo_urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. ENVIRONMENT
    set_gl_software = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')

    # 4. IGNITION GAZEBO
    ignition = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', maze_world_path],
        output='screen',
        additional_env={'OGRE_RTT_MODE': 'Copy'}
    )
    
    # 5. ROBOT STATE PUBLISHER
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

    # 6. ROS GZ BRIDGE
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

    # 7. EKF NODE
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

    # 8. RVIZ (Must be in the return list!)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. SLAM LAUNCH
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 10. NAV2 LAUNCH
    # We use 'navigation_launch.py' (not bringup) because SLAM provides the map/localization.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items()
    )

    # 11. SPAWN ROBOT
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc, '-name', 'drive', '-x', '0.0', '-y', '0.0', '-z', '0.5'],
    )

    # DELAYS
    # Spawn robot after 5 seconds to ensure Gazebo is ready
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])
    
    # Start Navigation after 10 seconds to ensure SLAM has built a basic map
    delayed_navigation = TimerAction(period=10.0, actions=[navigation_launch])

    return LaunchDescription([
        set_gl_software,
        ignition,
        robot_state_publisher,
        bridge,
        ekf_node,
        slam_launch,
        delayed_spawn,
        rviz_node,
        delayed_navigation  # <--- Nav2 is launched here
    ])