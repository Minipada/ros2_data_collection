# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.8)
project(dc_measurements)

set(dependencies
  action_msgs
  controller_manager_msgs
  cv_bridge
  dc_common
  dc_core
  dc_interfaces
  dc_util
  diagnostic_msgs
  geometry_msgs
  lifecycle_msgs
  moveit_msgs
  nav2_msgs
  nav2_util
  nlohmann_json
  nlohmann_json_schema_validator
  PkgConfig
  pluginlib
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  unique_identifier_msgs
  yaml_cpp_vendor
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_search_module(UUID REQUIRED uuid)

# dc_common::WebSocketJsonClient (#390, used by the mission_open_rmf plugin, #391) needs
# std::thread -- linked explicitly rather than assuming a transitive dependency already pulls in
# pthread, same reasoning as dc_common's own websocket_json_client_test.
find_package(Threads REQUIRED)

# ZXing-C++ (#123): only the camera plugin's barcode/QR detection needs it, and it
# exports a plain CMake config (ZXing::ZXing), not an ament package, so it's kept out of
# the shared `dependencies` list/foreach below and linked directly to dc_camera_measurement.
find_package(ZXing REQUIRED)

# cv_bridge only re-exports OpenCV's core/imgproc/imgcodecs; the camera plugin's pose
# estimation (#48) calls cv::solvePnP, which lives in opencv_calib3d.
find_package(OpenCV REQUIRED COMPONENTS calib3d)

# yaml_cpp_vendor (already in `dependencies` above) only guarantees a system yaml-cpp is
# installed via rosdep; it doesn't itself re-export a linkable CMake target, so anything that
# calls into yaml-cpp's `.as<T>()` conversions (which can throw YAML::BadConversion) needs the
# real `yaml-cpp` package found separately for its `yaml-cpp::yaml-cpp` target -- discovered via
# #254's test suite dlopen-ing cmd_vel/map for the first time and hitting `undefined symbol:
# typeinfo for YAML::BadConversion` at pluginlib load time (nlohmann_json/nlohmann_json_schema_validator
# below hit the same class of gap previously, hence their own redundant target_link_libraries).
find_package(yaml-cpp REQUIRED)

# ffmpeg (already a package.xml <depend>, providing headers for ip_camera.cpp's libavformat
# calls) has no ament package/CMake config of its own -- discovered the same way as yaml-cpp
# above, via #254's test suite dlopen-ing ip_camera for the first time and hitting
# `undefined symbol: av_log_set_level` (from libavutil, pulled in transitively by libavformat)
# at pluginlib load time.
pkg_search_module(AVFORMAT REQUIRED libavformat)
pkg_search_module(AVUTIL REQUIRED libavutil)

# fastdds_stats (#392): eProsima's Fast-DDS-statistics-backend has no rosdep/apt key -- it isn't
# released as a binary package for any distro (checked against rosdistro's rosdep/base.yaml and
# jazzy/distribution.yaml) -- and reading Fast DDS's own Statistics Module only means anything
# when the deployment actually runs Fast DDS as its RMW. So this plugin is optional: found here,
# built here; not found, skipped with a clear message rather than failing the whole workspace.
# Install it from source per https://fast-dds-statistics-backend.readthedocs.io (also requires
# Fast DDS itself built with -DFASTDDS_STATISTICS=ON) if this Measurement is wanted; see
# doc/src/dc/measurements/fastdds_stats.md.
find_package(fastdds_statistics_backend QUIET CONFIG)
if(fastdds_statistics_backend_FOUND)
  message(STATUS "fastdds_statistics_backend found: building the fastdds_stats Measurement plugin")
else()
  message(STATUS "fastdds_statistics_backend not found: skipping the fastdds_stats Measurement plugin")
endif()

dc_package()

set(library_name measurement_server_core)
set(executable_name measurement_server)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

# Plugin registration
# One call per plugin owns the registration rules (#485): library target, plugin-libs entry,
# ament deps, link set and export define. DEPS appends plugin-specific libraries.
function(dc_add_measurement_plugin name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES;DEPS")
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "dc_add_measurement_plugin(${name}): unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  add_library(${name} SHARED ${ARG_SOURCES})
  list(APPEND dc_measurement_plugin_libs ${name})
  set(dc_measurement_plugin_libs ${dc_measurement_plugin_libs} PARENT_SCOPE)
  ament_target_dependencies(${name} ${dependencies})
  target_link_libraries(${name}
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
    yaml-cpp::yaml-cpp
    ${UUID_LIBRARIES}
    ${ARG_DEPS}
  )
  target_compile_definitions(${name} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

# Conditions link no Record-side libraries; everything else matches the Measurement rule.
function(dc_add_condition_plugin name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES")
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "dc_add_condition_plugin(${name}): unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  add_library(${name} SHARED ${ARG_SOURCES})
  list(APPEND dc_condition_plugin_libs ${name})
  set(dc_condition_plugin_libs ${dc_condition_plugin_libs} PARENT_SCOPE)
  ament_target_dependencies(${name} ${dependencies})
  target_compile_definitions(${name} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

# Shared by the cpu/memory/os/uptime plugins.
set(system_sources
  plugins/measurements/system/linux_parser.cpp
  plugins/measurements/system/process.cpp
  plugins/measurements/system/processor.cpp
  plugins/measurements/system/system.cpp
)


# Condition plugins
dc_add_condition_plugin(dc_compare_condition SOURCES plugins/conditions/compare.cpp)
dc_add_condition_plugin(dc_moving_condition SOURCES plugins/conditions/moving.cpp)
dc_add_condition_plugin(dc_same_as_previous_condition SOURCES plugins/conditions/same_as_previous.cpp)

# Measurement plugins
dc_add_measurement_plugin(dc_battery_measurement SOURCES plugins/measurements/battery.cpp)
# barcode/QR reading (#123) + cv::solvePnP's calib3d (#48); found and explained above.
dc_add_measurement_plugin(dc_camera_measurement
  SOURCES plugins/measurements/camera.cpp
  DEPS ZXing::ZXing opencv_calib3d
)
dc_add_measurement_plugin(dc_cmd_vel_measurement SOURCES plugins/measurements/cmd_vel.cpp)
dc_add_measurement_plugin(dc_cpu_measurement
  SOURCES plugins/measurements/cpu.cpp ${system_sources}
)
dc_add_measurement_plugin(dc_diagnostics_measurement SOURCES plugins/measurements/diagnostics.cpp)
dc_add_measurement_plugin(dc_dummy_measurement SOURCES plugins/measurements/dummy.cpp)
dc_add_measurement_plugin(dc_distance_traveled_measurement SOURCES plugins/measurements/distance_traveled.cpp)
dc_add_measurement_plugin(dc_fault_measurement SOURCES plugins/measurements/fault.cpp)

if(fastdds_statistics_backend_FOUND)
  dc_add_measurement_plugin(dc_fastdds_stats_measurement
    SOURCES plugins/measurements/fastdds_stats.cpp
    DEPS fastdds_statistics_backend
  )
endif()

dc_add_measurement_plugin(dc_driving_type_measurement SOURCES plugins/measurements/driving_type.cpp)
dc_add_measurement_plugin(dc_intervention_measurement SOURCES plugins/measurements/intervention.cpp)
dc_add_measurement_plugin(dc_ip_camera_measurement
  SOURCES plugins/measurements/ip_camera.cpp
  DEPS ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES}
)
# libav has no CMake target (see the pkg_search_module above), so its dirs are wired by hand.
target_include_directories(dc_ip_camera_measurement PRIVATE
  ${AVFORMAT_INCLUDE_DIRS} ${AVUTIL_INCLUDE_DIRS}
)
target_link_directories(dc_ip_camera_measurement PRIVATE
  ${AVFORMAT_LIBRARY_DIRS} ${AVUTIL_LIBRARY_DIRS}
)

dc_add_measurement_plugin(dc_manipulation_measurement SOURCES plugins/measurements/manipulation.cpp)
dc_add_measurement_plugin(dc_map_measurement SOURCES plugins/measurements/map.cpp)
dc_add_measurement_plugin(dc_mission_nav2_measurement SOURCES plugins/measurements/mission_nav2.cpp)
dc_add_measurement_plugin(dc_mission_nav2_follow_waypoints_measurement SOURCES plugins/measurements/mission_nav2_follow_waypoints.cpp)
dc_add_measurement_plugin(dc_mission_nav2_through_poses_measurement SOURCES plugins/measurements/mission_nav2_through_poses.cpp)
# WebSocketJsonClient (#390) uses std::thread; linked explicitly rather than relying on it
# coming in transitively, same reasoning as dc_common's own websocket_json_client_test.
dc_add_measurement_plugin(dc_mission_open_rmf_measurement
  SOURCES plugins/measurements/mission_open_rmf.cpp
  DEPS Threads::Threads
)
dc_add_measurement_plugin(dc_memory_measurement
  SOURCES plugins/measurements/memory.cpp ${system_sources}
)
dc_add_measurement_plugin(dc_network_measurement SOURCES plugins/measurements/network.cpp)
dc_add_measurement_plugin(dc_os_measurement
  SOURCES plugins/measurements/os.cpp ${system_sources}
)
dc_add_measurement_plugin(dc_permissions_measurement SOURCES plugins/measurements/permissions.cpp)
dc_add_measurement_plugin(dc_position_measurement SOURCES plugins/measurements/position.cpp)
dc_add_measurement_plugin(dc_random_measurement SOURCES plugins/measurements/random.cpp)
dc_add_measurement_plugin(dc_ros2_control_status_measurement SOURCES plugins/measurements/ros2_control_status.cpp)
dc_add_measurement_plugin(dc_serial_interface_measurement SOURCES plugins/measurements/serial_interface.cpp)
dc_add_measurement_plugin(dc_slam_toolbox_quality_measurement SOURCES plugins/measurements/slam_toolbox_quality.cpp)
dc_add_measurement_plugin(dc_speed_measurement SOURCES plugins/measurements/speed.cpp)
dc_add_measurement_plugin(dc_storage_measurement SOURCES plugins/measurements/storage.cpp)
dc_add_measurement_plugin(dc_string_stamped_measurement SOURCES plugins/measurements/string_stamped.cpp)
dc_add_measurement_plugin(dc_tcp_health_measurement SOURCES plugins/measurements/tcp_health.cpp)
dc_add_measurement_plugin(dc_thermal_measurement SOURCES plugins/measurements/thermal.cpp)
dc_add_measurement_plugin(dc_uptime_measurement
  SOURCES plugins/measurements/uptime.cpp ${system_sources}
)

pluginlib_export_plugin_description_file(dc_core measurement_plugin.xml)
pluginlib_export_plugin_description_file(dc_core condition_plugin.xml)

# Library
add_library(${library_name} SHARED
  src/measurement_server.cpp
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
                ${dc_condition_plugin_libs}
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

install(FILES condition_plugin.xml measurement_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plugins/measurements/json
  DESTINATION share/${PROJECT_NAME}/plugins/measurements/
)

set(tests
  test_barcode_rotation
  test_code_pose
  test_incident_releaser
  test_measurement_battery
  test_measurement_buffering
  test_measurement_camera
  test_measurement_cmd_vel
  test_measurement_compare
  test_measurement_diagnostics
  test_measurement_distance_traveled
  test_measurement_driving_type
  test_measurement_dummy
  test_measurement_fault
  test_measurement_gate_condition
  test_measurement_intervention
  test_measurement_ip_camera
  test_measurement_manipulation
  test_measurement_map
  test_measurement_memory
  test_measurement_mission_nav2
  test_measurement_mission_nav2_follow_waypoints
  test_measurement_mission_nav2_through_poses
  test_measurement_mission_open_rmf
  test_measurement_moving
  test_measurement_network
  test_measurement_os
  test_measurement_parameters
  test_measurement_permissions
  test_measurement_position
  test_measurement_random
  test_measurement_ros2_control_status
  test_measurement_same_as_previous
  test_measurement_serial_interface
  test_measurement_shared_flush
  test_measurement_slam_toolbox_quality
  test_measurement_speed
  test_measurement_storage
  test_measurement_tcp_health
  test_measurement_thermal
  test_measurement_uptime
  mission_follow_waypoints_tracker_test
  mission_nav2_tracker_test
  test_mission_nav2_through_poses_core
  test_mission_open_rmf_core
)

# The plugin lib doesn't exist without the library, and this test would only ever report a
# timeout failure -- registered alongside dc_fastdds_stats_measurement itself, not unconditionally.
if(fastdds_statistics_backend_FOUND)
  list(APPEND tests test_measurement_fastdds_stats)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test ${tests})
    ament_add_gtest(${PROJECT_NAME}_${test} test/${test}.cpp)
    target_include_directories(${PROJECT_NAME}_${test} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(${PROJECT_NAME}_${test}
    ${dependencies}
    )
    target_link_libraries(
      ${PROJECT_NAME}_${test}
      nlohmann_json::nlohmann_json
      nlohmann_json_schema_validator
      opencv_calib3d
      ${library_name}
      )
  endforeach()
  # These two encode a QR code (ZXing's writer) to read one back; every other test gets
  # the reader transitively or not at all.
  target_link_libraries(${PROJECT_NAME}_test_code_pose ZXing::ZXing)
  target_link_libraries(${PROJECT_NAME}_test_barcode_rotation ZXing::ZXing)
  # test_measurement_mission_open_rmf's fixture is its own small websocket server (Boost.Beast),
  # standing in for #391's real websocket_url endpoint; needs std::thread like the plugin itself.
  target_link_libraries(${PROJECT_NAME}_test_measurement_mission_open_rmf Threads::Threads)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  ${dc_measurement_plugin_libs}
  ${dc_condition_plugin_libs}
)
ament_export_dependencies(${dependencies})
ament_package()
