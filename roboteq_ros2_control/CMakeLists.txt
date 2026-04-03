cmake_minimum_required(VERSION 3.8)
project(roboteq_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

ament_auto_add_library(
  roboteq_ros2_control_plugin SHARED
  src/epoll_event_loop.cpp
  src/socket_can.cpp
  src/roboteq_hardware_interface.cpp
)

ament_target_dependencies(
  roboteq_ros2_control_plugin
  rclcpp
)

pluginlib_export_plugin_description_file(roboteq_interface roboteq_hardware_interface.xml)

target_compile_features(roboteq_ros2_control_plugin PRIVATE cxx_std_20)

install(TARGETS roboteq_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
