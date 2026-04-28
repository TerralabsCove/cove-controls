cmake_minimum_required(VERSION 3.8)
project(canhat_one_motor)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
