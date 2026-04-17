cmake_minimum_required(VERSION 3.8)
project(simple_assembly)

find_package(ament_cmake REQUIRED)

# Install all package directories so ROS2 can find meshes, urdf, launch files
install(
  DIRECTORY launch meshes urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
