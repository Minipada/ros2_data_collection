cmake_minimum_required(VERSION 3.8)
project(dc_triggers)

set(dependencies
  dc_core
  dc_interfaces
  dc_util
  lifecycle_msgs
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

dc_package()

set(library_name trigger_broadcast_node_core)
set(executable_name trigger_broadcast_node)

include_directories(
  include
  ${UUID_INCLUDE_DIRS}
)

# Trigger plugins
add_library(dc_edge_trigger SHARED plugins/edge_trigger.cpp)
list(APPEND dc_trigger_plugin_libs dc_edge_trigger)

foreach(trigger_plugin ${dc_trigger_plugin_libs})
  ament_target_dependencies(${trigger_plugin} ${dependencies})
  target_compile_definitions(${trigger_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

pluginlib_export_plugin_description_file(dc_core trigger_plugin.xml)

# Library
add_library(${library_name} SHARED
  src/trigger_broadcast_node.cpp
)

target_link_libraries(
  ${library_name}
  ${UUID_LIBRARIES}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# Executable
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(
  ${executable_name}
  ${library_name}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
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
  ament_target_dependencies(${PROJECT_NAME}_test_edge_trigger
    ${dependencies}
    dc_measurements
    nav_msgs
  )
  target_link_libraries(
    ${PROJECT_NAME}_test_edge_trigger
    ${library_name}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  ${dc_trigger_plugin_libs}
)
ament_export_dependencies(${dependencies})
ament_package()
