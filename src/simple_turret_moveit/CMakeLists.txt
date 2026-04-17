cmake_minimum_required(VERSION 3.8)
project(simple_turret_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf srdf meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
