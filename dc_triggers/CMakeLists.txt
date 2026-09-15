# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.16)
project(dc_triggers)

set(dependencies
  dc_core
  dc_interfaces
  dc_util
  lifecycle_msgs
  nav2_ros_common
  nav2_util
  pluginlib
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_search_module(UUID REQUIRED uuid)

# Message packages have no pkg::pkg umbrella target -- their C++ typesupport target
# carries the generated code; dc_core is header-only (include dirs wired below);
# dc_util's export is namespaced.
set(link_dependencies
  dc_interfaces::dc_interfaces__rosidl_typesupport_cpp
  dc_util::base64_library
  lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs__rosidl_typesupport_cpp
)

dc_package()

set(library_name trigger_broadcast_node_core)
set(executable_name trigger_broadcast_node)

include_directories(
  include
  ${UUID_INCLUDE_DIRS}
  ${dc_core_INCLUDE_DIRS}
)

# Trigger plugins
add_library(dc_edge_trigger SHARED plugins/edge_trigger.cpp)
list(APPEND dc_trigger_plugin_libs dc_edge_trigger)

foreach(trigger_plugin ${dc_trigger_plugin_libs})
  target_link_libraries(${trigger_plugin} PUBLIC ${link_dependencies})
  target_compile_definitions(${trigger_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

pluginlib_export_plugin_description_file(dc_core trigger_plugin.xml)

# Library
add_library(${library_name} SHARED
  src/trigger_broadcast_node.cpp
)

target_link_libraries(${library_name} PUBLIC
  ${UUID_LIBRARIES}
  ${link_dependencies}
)

# Executable
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} PRIVATE
  ${library_name}
  ${link_dependencies}
)

rclcpp_components_register_nodes(${library_name} "trigger_broadcast_node::TriggerBroadcastNode")

install(TARGETS ${library_name}
                ${dc_trigger_plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES trigger_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(dc_measurements REQUIRED)
  find_package(nav_msgs REQUIRED)

  # test_trigger_edge_trigger.cpp drives a real TriggerBroadcastNode end to end (lifecycle-node
  # integration test, matching dc_measurements' test_measurement_*.cpp style) and needs
  # dc_measurements' Moving condition plugin -- an existing self-subscribing Condition -- to
  # toggle deterministically without a bespoke test-only plugin.
  ament_add_gtest(${PROJECT_NAME}_test_edge_trigger test/test_trigger_edge_trigger.cpp)
  target_include_directories(${PROJECT_NAME}_test_edge_trigger PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  # dc_measurements exports plain library names, not CMake targets, so its libraries and
  # include dirs come via the variables ament_target_dependencies used to consume (#531).
  target_include_directories(${PROJECT_NAME}_test_edge_trigger PRIVATE
    ${dc_measurements_INCLUDE_DIRS}
  )
  # Plain signature: ament_add_gtest links gtest with the plain form, and CMake refuses
  # to mix signatures on one target (namespaced targets work as plain args).
  target_link_libraries(${PROJECT_NAME}_test_edge_trigger
    ${library_name}
    ${link_dependencies}
    ${dc_measurements_LIBRARIES}
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  ${dc_trigger_plugin_libs}
)
ament_export_dependencies(${dependencies})
ament_package()
