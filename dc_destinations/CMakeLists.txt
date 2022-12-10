cmake_minimum_required(VERSION 3.8)
project(dc_destinations)

set(dependencies
  ament_index_cpp
  dc_core
  dc_interfaces
  dc_util
  fluent_bit_vendor
  nav2_util
  pluginlib
  rclcpp_components
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(fluent_bit REQUIRED)

dc_package()

set(library_name destination_server_core)
set(executable_name destination_server)

include_directories(include)

# Plugins destinations
add_library(dc_flb_file_destination SHARED plugins/flb_file.cpp)
list(APPEND dc_destination_plugin_libs dc_flb_file_destination)

add_library(dc_flb_http_destination SHARED plugins/flb_http.cpp)
list(APPEND dc_destination_plugin_libs dc_flb_http_destination)

add_library(dc_flb_kinesis_streams_destination SHARED plugins/flb_kinesis_streams.cpp)
list(APPEND dc_destination_plugin_libs dc_flb_kinesis_streams_destination)

add_library(dc_flb_stdout_destination SHARED plugins/flb_stdout.cpp)
list(APPEND dc_destination_plugin_libs dc_flb_stdout_destination)

add_library(dc_rcl_destination SHARED plugins/rcl.cpp)
list(APPEND dc_destination_plugin_libs dc_rcl_destination)

foreach(destination_plugin ${dc_destination_plugin_libs})
  ament_target_dependencies(${destination_plugin} ${dependencies})
  target_link_libraries(
    ${destination_plugin}
    fluent_bit::fluent_bit
  )
  target_compile_definitions(${destination_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

pluginlib_export_plugin_description_file(dc_core destination_plugin.xml)

# Library
add_library(${library_name} SHARED
  src/destination_server.cpp
)

target_link_libraries(
  ${library_name}
  fluent_bit::fluent_bit
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

rclcpp_components_register_nodes(${library_name} "destination_server::DestinationServer")


install(TARGETS ${library_name}
                ${dc_destination_plugin_libs}
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

install(FILES destination_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  ${dc_destination_plugin_libs}
)
ament_export_dependencies(${dependencies})
ament_package()
