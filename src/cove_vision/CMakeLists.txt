cmake_minimum_required(VERSION 3.8)
project(cove_vision)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    nodes/tag_tf_marker.py
    nodes/robot_description_topic.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
