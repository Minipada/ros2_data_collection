cmake_minimum_required(VERSION 3.8)
project(dc_measurements)

set(dependencies
  dc_core
  dc_interfaces
  dc_util
  geometry_msgs
  nav2_util
  nlohmann_json
  nlohmann_json_schema_validator
  PkgConfig
  pluginlib
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  tf2
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()
dc_package()

set(library_name measurement_server_core)
set(executable_name measurement_server)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Measurement plugins
add_library(dc_cpu_measurement SHARED
  plugins/measurements/cpu.cpp
  plugins/measurements/system/linux_parser.cpp
  plugins/measurements/system/process.cpp
  plugins/measurements/system/processor.cpp
  plugins/measurements/system/system.cpp
)
list(APPEND dc_measurement_plugin_libs dc_cpu_measurement)

add_library(dc_os_measurement SHARED
  plugins/measurements/os.cpp
  plugins/measurements/system/linux_parser.cpp
  plugins/measurements/system/process.cpp
  plugins/measurements/system/processor.cpp
  plugins/measurements/system/system.cpp
)
list(APPEND dc_measurement_plugin_libs dc_os_measurement)

add_library(dc_uptime_measurement SHARED
  plugins/measurements/uptime.cpp
  plugins/measurements/system/linux_parser.cpp
  plugins/measurements/system/process.cpp
  plugins/measurements/system/processor.cpp
  plugins/measurements/system/system.cpp
)
list(APPEND dc_measurement_plugin_libs dc_uptime_measurement)

add_library(dc_string_stamped_measurement SHARED plugins/measurements/string_stamped.cpp)
list(APPEND dc_measurement_plugin_libs dc_string_stamped_measurement)

foreach(measurement_plugin ${dc_measurement_plugin_libs})
  ament_target_dependencies(${measurement_plugin} ${dependencies})
  target_link_libraries(
    ${measurement_plugin}
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
  )
  target_compile_definitions(${measurement_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

pluginlib_export_plugin_description_file(dc_core measurement_plugin.xml)

# Library
add_library(${library_name} SHARED
  src/measurement_server.cpp
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
  nlohmann_json::nlohmann_json
  nlohmann_json_schema_validator
  ${library_name}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "measurement_server::MeasurementServer")

install(TARGETS ${library_name}
                ${dc_measurement_plugin_libs}
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

install(FILES measurement_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plugins/measurements/
  DESTINATION share/${PROJECT_NAME}/plugins/measurements/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  ${dc_measurement_plugin_libs}
)
ament_export_dependencies(${dependencies})
ament_package()
