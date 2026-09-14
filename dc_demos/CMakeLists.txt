# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.5)
project(dc_demos)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# nav2_bringup stays a package.xml exec_depend only: it is released into the lyrical
# rosdistro but its resolute .deb publish still lags (#531), so there is no CMake config
# to find at configure time. Nothing here compiles against it -- only the tb3 demo launch
# files include it at runtime, and none of them is CI-exercised. Restore it here once the
# .deb ships.
set(dependencies
  geometry_msgs
  dc_measurements
  dc_simulation
  nlohmann_json
  nlohmann_json_schema_validator
  rclpy
  simulation_interfaces
  xacro
)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()
dc_package()

install(
  DIRECTORY launch params config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables. RENAME drops the .py so the demo is run as
# `ros2 run dc_demos qrcodes_waypoint_follower`, the usual ROS 2 spelling.
install(PROGRAMS
  ${PROJECT_NAME}/qrcodes_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qrcodes_waypoint_follower
)

# tools/sim/scripts/run.sh's DC_SIM_WORLD=small fixture -- see its own module docstring.
install(PROGRAMS
  ${PROJECT_NAME}/qrcodes_small_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qrcodes_small_waypoint_follower
)

include_directories(
  include
)

# Measurement plugins
add_library(dc_uptime_custom_measurement SHARED
  plugins/measurements/uptime_custom.cpp
)
list(APPEND dc_measurement_plugin_libs dc_uptime_custom_measurement)

# ament_target_dependencies was removed in lyrical (#531): dependencies link by exported
# target instead. Message packages have no pkg::pkg umbrella target on lyrical -- their
# C++ typesupport target carries the generated code. The launch/Python-only deps (rclpy,
# xacro, nav2_bringup, dc_simulation) export nothing linkable, and dc_measurements
# exports plain library names, not CMake targets -- linked via the same variables
# ament_target_dependencies used to consume.
foreach(measurement_plugin ${dc_measurement_plugin_libs})
  target_include_directories(${measurement_plugin} PUBLIC
    ${dc_measurements_INCLUDE_DIRS}
  )
  target_link_libraries(${measurement_plugin} PUBLIC
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    simulation_interfaces::simulation_interfaces__rosidl_typesupport_cpp
    ${dc_measurements_LIBRARIES}
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
