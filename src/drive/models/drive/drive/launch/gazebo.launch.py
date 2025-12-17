import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'drive'
    rover_final_share = get_package_share_directory(pkg_name)

    # 1. DEFINE GAZEBO URDF FILE PATH
    # NOTE: This file (rover_final.urdf) MUST contain absolute file:// paths
    gazebo_urdf_file_path = os.path.join(rover_final_share, 'urdf', 'drive2.urdf')

    # 2. SET VIRTUALIZATION GRAPHICS WORKAROUNDS (Crucial for VM stability)
    set_gl_software = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1' 
    )

    # 3. START IGNITION GAZEBO (FORTRESS)
    ignition = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )
    
    # 4. SPAWN THE ENTITY 
    # Uses the file:// URDF content for guaranteed spawning.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', gazebo_urdf_file_path,  # Uses the file with file:// paths
            '-name', 'drive',
            '-topic','robot_description',
            '-x', '1', '-y', '-5', '-z', '0.0',
            
             
        ],
    )

    # Delay the spawn by 5 seconds to ensure the Gazebo server is ready
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    # 5. ROBOT STATE PUBLISHER (Still useful for bridge/TF even if RViz isn't open)
    # Since you only requested Gazebo, this can be omitted, but it's often essential 
    # for plugins or bridge communication. We will keep it but comment out the content read 
    # as it's not strictly necessary for spawning a model via '-file'. 
    # If you need this running for other ROS nodes, uncomment and load the original URDF.
    # --------------------------------------------------------------------------
    # try:
    #     with open(os.path.join(rover_final_share, 'urdf', 'rover_final.urdf'), 'r') as infp:
    #         robot_description_content_rviz = infp.read()
    # except EnvironmentError:
    #     robot_description_content_rviz = ""
    #
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description_content_rviz}],
    # )
    # --------------------------------------------------------------------------

    return LaunchDescription([
        # Graphics fix
        set_gl_software, 
        
        # Core Nodes
        # robot_state_publisher_node, # Commented out as it's not strictly required for Gazebo-only view

        # Simulation
        ignition,
        delayed_spawn,
    ])
