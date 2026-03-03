cmake_minimum_required(VERSION 3.5)

project(drive)

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rviz2 REQUIRED)

install(
  DIRECTORY launch urdf meshes textures config worlds rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()