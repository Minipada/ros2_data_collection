cmake_minimum_required(VERSION 3.5)
project(dc_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(dc_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# FIXME nav2_util
# In file included from /root/ros2_data_collection/src/ros2_data_collection/dc_lifecycle_manager/src/lifecycle_manager_client.cpp:8:
#/opt/ros/humble/include/nav2_util/geometry_utils.hpp:25:10: fatal error: tf2_geometry_msgs/tf2_geometry_msgs.hpp: No such file or directory
#   25 | #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)

dc_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

set(dependencies
  bondcpp
  diagnostic_updater
  lifecycle_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(lifecycle_manager
  src/main.cpp
)

target_link_libraries(lifecycle_manager
  ${library_name}
)

ament_target_dependencies(lifecycle_manager
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "dc_lifecycle_manager::LifecycleManager")

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
