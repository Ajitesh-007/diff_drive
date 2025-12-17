cmake_minimum_required(VERSION 3.5)

project(drive)

find_package(ament_cmake REQUIRED)

find_package(ros_gz_sim)
find_package(ros_gz_interfaces)
find_package(ros_gz_bridge)
find_package(rviz2)
find_package(robot_state_publisher)

install(
	DIRECTORY meshes urdf launch config textures
	DESTINATION share/${PROJECT_NAME})

ament_package()
