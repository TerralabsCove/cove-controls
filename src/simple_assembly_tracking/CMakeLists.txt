cmake_minimum_required(VERSION 3.8)
project(simple_assembly_tracking)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    nodes/tag_tracker.py
    nodes/autotune_sweep.py
    nodes/plan_to_tag.py
    nodes/waypoint_runner.py
    nodes/continuous_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
