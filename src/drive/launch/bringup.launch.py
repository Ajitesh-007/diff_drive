import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'drive'
    drive_share = get_package_share_directory(pkg_name)

    # 1. PATHS
    gazebo_urdf_file_path = os.path.join(drive_share, 'urdf', 'drive3.urdf')
    ekf_config_path = os.path.join(drive_share, 'config', 'ekf.yaml')
    maze_world_path = "/home/ajitesh/ros2_ws/src/drive/worlds/complex_maze.sdf"

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
    
    # 5. ROBOT STATE PUBLISHER (UPDATED)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True  # <--- CHANGE 1: Sync with Sim Time
        }],
    )

    # 6. ROS GZ BRIDGE (UPDATED)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # <--- CHANGE 2: Bridge the Clock so ROS knows the Sim Time
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

    # 7. EKF NODE (UPDATED)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path, 
            {'use_sim_time': True} # <--- CHANGE 3: Sync EKF with Sim Time
        ],
        remappings=[('/odometry/filtered', '/odom_filtered')]
    )

    # 8. SPAWN ROBOT
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc, '-name', 'drive', '-x', '0.0', '-y', '0.0', '-z', '0.5'],
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])

    return LaunchDescription([
        set_gl_software,
        ignition,
        robot_state_publisher,
        bridge,
        ekf_node,
        delayed_spawn,
    ])