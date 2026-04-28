cmake_minimum_required(VERSION 3.8)
project(damiao_socketcan_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(damiao_socketcan_core
  src/damiao_socketcan.cpp)
target_include_directories(damiao_socketcan_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(damiao_socketcan_node
  src/damiao_socketcan_node.cpp)
target_link_libraries(damiao_socketcan_node
  damiao_socketcan_core)
ament_target_dependencies(damiao_socketcan_node
  rclcpp std_msgs sensor_msgs std_srvs)

add_library(damiao_socketcan_hardware_interface SHARED
  src/damiao_socketcan_hardware_interface.cpp)
target_link_libraries(damiao_socketcan_hardware_interface
  damiao_socketcan_core)
ament_target_dependencies(damiao_socketcan_hardware_interface
  rclcpp rclcpp_lifecycle hardware_interface pluginlib)
target_include_directories(damiao_socketcan_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

pluginlib_export_plugin_description_file(
  hardware_interface damiao_socketcan_hardware_interface.xml)

install(TARGETS
  damiao_socketcan_core
  EXPORT export_damiao_socketcan_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS damiao_socketcan_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS damiao_socketcan_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(FILES damiao_socketcan_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_damiao_socketcan_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(damiao_socketcan_core)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
