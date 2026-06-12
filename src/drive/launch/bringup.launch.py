import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_drive = get_package_share_directory('drive')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg_drive, 'urdf', 'drive3.urdf')
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    # Resolve $(find drive) — plain URDF doesn't support xacro substitutions
    robot_description_content = robot_description_content.replace('$(find drive)', pkg_drive)

    robot_description = {'robot_description': robot_description_content}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true',
    )

    world_file = os.path.join(pkg_drive, 'worlds', 'complex_maze.sdf')

    declare_world = DeclareLaunchArgument(
        'world', default_value=world_file,
        description='Full path to the Ignition Gazebo world file',
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_drive, '..'),
    )

    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world')],
        }.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

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

    # Bridge: IMU + clock (cmd_vel/odom/joint_states handled by ros2_control)
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
    # NOTE: Gazebo publishes with Best Effort QoS — ros_gz_bridge mirrors this.
    # rgbd_sync and RTAB-Map must subscribe with Best Effort on these topics.
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

    # EKF: fuse wheel velocity + IMU heading
    # Started first (immediately after controllers) so TF odom→base_link is
    # available before RTAB-Map tries to place point clouds.
    ekf_config = os.path.join(pkg_drive, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # ----------------------------------------------------------------
    # RGBD Sync: subscribes to individual camera topics and publishes a
    # time-synchronised rtabmap_msgs/RGBDImage on /rgbd_image.
    #
    # FIX: add qos_overrides so the node subscribes to camera topics
    # with Best Effort reliability — this matches the QoS Gazebo bridge
    # uses and prevents silent topic drops.
    # ----------------------------------------------------------------
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'approx_sync_max_interval': 0.2,
            'queue_size': 30,
            # FIX: subscriber QoS must match Gazebo bridge (Best Effort / Volatile)
            'qos': 2,            # 1=Reliable, 2=Best Effort
            'qos_camera_info': 2,
        }],
        remappings=[
            ('rgb/image',       '/camera/image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image',     '/camera/depth_image'),
            # output: /rgbd_image  (default, no remap needed)
        ],
    )

    # ----------------------------------------------------------------
    # RTAB-Map (MAPPING mode)
    # subscribe_rgbd=true  → consumes the synced /rgbd_image topic
    # ----------------------------------------------------------------
    rtabmap_config = os.path.join(pkg_drive, 'config', 'rtabmap.yaml')

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('rgbd_image', '/rgbd_image'),   # from rgbd_sync node
            ('odom',       '/odom'),
        ],
        arguments=['--delete_db_on_start'],  # Fresh map each launch for mapping mode
    )

    # RViz2
    rviz_config_file = os.path.join(pkg_drive, 'rviz', 'drive_config.rviz')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ----------------------------------------------------------------
    # Startup sequence after controllers are ready:
    #   t=0 s  → EKF (publishes odom→base_link TF immediately)
    #   t=4 s  → rgbd_sync (camera bridge needs time to establish)
    #   t=7 s  → rtabmap (TF + rgbd_image guaranteed available)
    #   t=7 s  → RViz2
    # ----------------------------------------------------------------
    delayed_nodes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[
                ekf_node,
                TimerAction(period=4.0, actions=[rgbd_sync_node]),
                TimerAction(period=7.0, actions=[rtabmap_node, rviz2_node]),
            ]
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
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