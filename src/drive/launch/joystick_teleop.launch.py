import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the Joystick Driver (reads hardware)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0', # Ensure this matches your joystick port
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),

        # 2. Start the Teleop Node (converts joy -> cmd_vel)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'require_enable_button': True,
                'enable_button': 0,  # Button index to hold (usually 'A' or 'X')
                'axis_linear.x': 1,  # Vertical axis index (Left stick vertical)
                'scale_linear.x': 1.0,  # Max speed in m/s
                'axis_angular.yaw': 3,  # Horizontal axis index (Right stick horizontal)
                'scale_angular.yaw': 1.5  # Max turn speed in rad/s
            }],
            # Remap is not needed if your robot listens to /cmd_vel
            # If your bridge uses a namespace, uncomment below:
            # remappings=[('/cmd_vel', '/model/drive/cmd_vel')]
        )
    ])