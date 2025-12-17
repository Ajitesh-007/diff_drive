cmake_minimum_required(VERSION 3.5)

project(drive)

find_package(ament_cmake REQUIRED)

# --- Find ROS 2 Control Packages ---
# These are required to access the installed controller configuration and libraries
find_package(ros2_control REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(imu_sensor_broadcaster REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

# --- Find Utility/Gazebo Packages ---
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rviz2 REQUIRED)

# Install directories
install(
	DIRECTORY meshes urdf launch config textures
	DESTINATION share/${PROJECT_NAME})

ament_package()