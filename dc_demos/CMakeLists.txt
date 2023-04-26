cmake_minimum_required(VERSION 3.5)
project(dc_demos)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(dependencies
  geometry_msgs
  dc_measurements
  dc_simulation
  nlohmann_json
  nlohmann_json_schema_validator
  nav2_bringup
  rclpy
  xacro
)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()
dc_package()

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/qrcodes_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
)

# Measurement plugins
add_library(dc_uptime_custom_measurement SHARED
  plugins/measurements/uptime_custom.cpp
)
list(APPEND dc_measurement_plugin_libs dc_uptime_custom_measurement)

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

install(TARGETS ${library_name}
                ${dc_measurement_plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
