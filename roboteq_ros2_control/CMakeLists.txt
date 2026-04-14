cmake_minimum_required(VERSION 3.8)
project(roboteq_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(
  roboteq_ros2_control_plugin SHARED
  src/epoll_event_loop.cpp
  src/socket_can.cpp
  src/roboteq_hardware_interface.cpp
)

target_include_directories(roboteq_ros2_control_plugin PRIVATE
  include
)

ament_target_dependencies(
  roboteq_ros2_control_plugin
  rclcpp
)

target_compile_features(roboteq_ros2_control_plugin PRIVATE cxx_std_20)

pluginlib_export_plugin_description_file(hardware_interface roboteq_hardware_interface.xml)

install(TARGETS roboteq_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()