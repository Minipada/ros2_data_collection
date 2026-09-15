# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.16)
project(dc_lifecycle_manager)

set(dependencies
  bondcpp
  diagnostic_updater
  dc_common
  lifecycle_msgs
  nav2_msgs
  nav2_ros_common
  nav2_util
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

# ament_target_dependencies was removed in lyrical (#531): dependencies link by exported
# target instead. Message packages have no pkg::pkg umbrella target on lyrical -- their
# C++ typesupport target carries the generated code; dc_common is header-only (include
# dirs wired below).
set(link_dependencies
  bondcpp::bondcpp
  diagnostic_updater::diagnostic_updater
  lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp
  nav2_msgs::nav2_msgs__rosidl_typesupport_cpp
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component_manager
  std_msgs::std_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  tf2_geometry_msgs::tf2_geometry_msgs
)

dc_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

target_include_directories(${library_name} PUBLIC ${dc_common_INCLUDE_DIRS})
target_link_libraries(${library_name} PUBLIC ${link_dependencies})

add_executable(lifecycle_manager
  src/main.cpp
)

target_link_libraries(lifecycle_manager PRIVATE
  ${library_name}
  ${link_dependencies}
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

set(tests
  test_lifecycle_manager_startup
  test_lifecycle_manager_bond
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test ${tests})
    # Bond timeouts and the respawn retry window are wall-clock waits; the default
    # 60 s gtest timeout is not enough for them.
    ament_add_gtest(${PROJECT_NAME}_${test} test/${test}.cpp TIMEOUT 300)
    target_include_directories(${PROJECT_NAME}_${test} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
      $<INSTALL_INTERFACE:include>
    )
    # Plain signature: ament_add_gtest links gtest with the plain form, and CMake
    # refuses to mix signatures on one target (namespaced targets work as plain args).
    target_link_libraries(${PROJECT_NAME}_${test}
      ${library_name}
      ${link_dependencies}
    )
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
