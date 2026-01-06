import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'drive'
    drive_share = get_package_share_directory(pkg_name)

    # 1. DEFINE URDF FILE PATH
    gazebo_urdf_file_path = os.path.join(drive_share, 'urdf', 'drive3.urdf')

    # 2. READ THE URDF FILE
    # We need to read the file content to pass it to the robot_state_publisher
    with open(gazebo_urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. DEFINE THE WORLD PATH
    maze_world_path = "/home/ajitesh/ros2_ws/src/drive/worlds/complex_maze.sdf"

    # 4. GRAPHICS FIX FOR VM
    set_gl_software = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1'
    )

    # 5. START IGNITION GAZEBO
    ignition = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', maze_world_path],
        output='screen',
        additional_env={'OGRE_RTT_MODE': 'Copy'}
    )
    
    # 6. NODE: ROBOT STATE PUBLISHER (CRITICAL FIX)
    # This publishes the TFs (transforms) that fix the "Frame does not exist" error
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 7. NODE: ROS GZ BRIDGE
    # Bridges the sensors (LiDAR/IMU/Joints) from Ignition to ROS 2 topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge LiDAR: Ignition LaserScan -> ROS LaserScan
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            
            # Bridge IMU: Ignition IMU -> ROS Imu
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            
            # Bridge Cmd Vel: ROS Twist -> Ignition Twist (Drive command)
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            
            # Bridge Joint States: Ignition Model -> ROS JointState
            # Note: This topic name might vary based on your world/model name. 
            # If joints don't move in RViz, check "ign topic -l" for the correct topic.
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            
            # Bridge TF: Ignition Pose_V -> ROS TFMessage (Optional, used for ground truth)
            '/model/my_robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
        ],
        output='screen'
    )

    # 8. SPAWN THE ROVER
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,  # Pass the file content directly
            '-name', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
    )

    # Delay spawn slightly to ensure world loads
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        set_gl_software,
        ignition,
        robot_state_publisher, # Fixes "Frame does not exist"
        bridge,                # Fixes "No sensor output"
        delayed_spawn,
    ])