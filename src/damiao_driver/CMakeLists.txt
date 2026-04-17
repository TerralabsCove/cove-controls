cmake_minimum_required(VERSION 3.8)
project(damiao_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

# --- Standalone ROS2 node (existing) ---
add_executable(damiao_node src/damiao_node.cpp)
ament_target_dependencies(damiao_node
  rclcpp std_msgs sensor_msgs geometry_msgs)

# --- Hardware interface plugin (for ros2_control / MoveIt2) ---
add_library(damiao_hardware_interface SHARED
  src/damiao_hardware_interface.cpp)
ament_target_dependencies(damiao_hardware_interface
  rclcpp rclcpp_lifecycle hardware_interface pluginlib)
target_include_directories(damiao_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

pluginlib_export_plugin_description_file(
  hardware_interface damiao_hardware_interface.xml)

# --- Install ---
install(TARGETS damiao_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS damiao_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(FILES damiao_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
