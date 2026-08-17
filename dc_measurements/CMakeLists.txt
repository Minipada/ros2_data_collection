cmake_minimum_required(VERSION 3.8)
project(dc_measurements)

set(dependencies
  cv_bridge
  dc_common
  dc_core
  dc_interfaces
  dc_util
  diagnostic_msgs
  geometry_msgs
  lifecycle_msgs
  nav2_util
  nlohmann_json
  nlohmann_json_schema_validator
  PkgConfig
  pluginlib
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  yaml_cpp_vendor
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_search_module(UUID REQUIRED uuid)

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

dc_package()

set(library_name measurement_server_core)
set(executable_name measurement_server)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

# Condition plugins
add_library(dc_bool_equal_condition SHARED plugins/conditions/bool_equal.cpp)
list(APPEND dc_condition_plugin_libs dc_bool_equal_condition)

add_library(dc_double_equal_condition SHARED plugins/conditions/double_equal.cpp)
list(APPEND dc_condition_plugin_libs dc_double_equal_condition)

add_library(dc_double_inferior_condition SHARED plugins/conditions/double_inferior.cpp)
list(APPEND dc_condition_plugin_libs dc_double_inferior_condition)

add_library(dc_double_superior_condition SHARED plugins/conditions/double_superior.cpp)
list(APPEND dc_condition_plugin_libs dc_double_superior_condition)

add_library(dc_exist_condition SHARED plugins/conditions/exist.cpp)
list(APPEND dc_condition_plugin_libs dc_exist_condition)

add_library(dc_integer_equal_condition SHARED plugins/conditions/integer_equal.cpp)
list(APPEND dc_condition_plugin_libs dc_integer_equal_condition)

add_library(dc_integer_inferior_condition SHARED plugins/conditions/integer_inferior.cpp)
list(APPEND dc_condition_plugin_libs dc_integer_inferior_condition)

add_library(dc_integer_superior_condition SHARED plugins/conditions/integer_superior.cpp)
list(APPEND dc_condition_plugin_libs dc_integer_superior_condition)

add_library(dc_list_bool_equal_condition SHARED plugins/conditions/list_bool_equal.cpp)
list(APPEND dc_condition_plugin_libs dc_list_bool_equal_condition)

add_library(dc_list_double_equal_condition SHARED plugins/conditions/list_double_equal.cpp)
list(APPEND dc_condition_plugin_libs dc_list_double_equal_condition)

add_library(dc_list_integer_equal_condition SHARED plugins/conditions/list_integer_equal.cpp)
list(APPEND dc_condition_plugin_libs dc_list_integer_equal_condition)

add_library(dc_list_string_equal_condition SHARED plugins/conditions/list_string_equal.cpp)
list(APPEND dc_condition_plugin_libs dc_list_string_equal_condition)

add_library(dc_moving_condition SHARED plugins/conditions/moving.cpp)
list(APPEND dc_condition_plugin_libs dc_moving_condition)

add_library(dc_same_as_previous_condition SHARED plugins/conditions/same_as_previous.cpp)
list(APPEND dc_condition_plugin_libs dc_same_as_previous_condition)

add_library(dc_string_match_condition SHARED plugins/conditions/string_match.cpp)
list(APPEND dc_condition_plugin_libs dc_string_match_condition)

# Measurement plugins
add_library(dc_camera_measurement SHARED plugins/measurements/camera.cpp)
list(APPEND dc_measurement_plugin_libs dc_camera_measurement)

add_library(dc_cmd_vel_measurement SHARED plugins/measurements/cmd_vel.cpp)
list(APPEND dc_measurement_plugin_libs dc_cmd_vel_measurement)

add_library(dc_cpu_measurement SHARED
  plugins/measurements/cpu.cpp
  plugins/measurements/system/linux_parser.cpp
  plugins/measurements/system/process.cpp
  plugins/measurements/system/processor.cpp
  plugins/measurements/system/system.cpp
)
list(APPEND dc_measurement_plugin_libs dc_cpu_measurement)

add_library(dc_diagnostics_measurement SHARED plugins/measurements/diagnostics.cpp)
list(APPEND dc_measurement_plugin_libs dc_diagnostics_measurement)

add_library(dc_dummy_measurement SHARED plugins/measurements/dummy.cpp)
list(APPEND dc_measurement_plugin_libs dc_dummy_measurement)

add_library(dc_distance_traveled_measurement SHARED plugins/measurements/distance_traveled.cpp)
list(APPEND dc_measurement_plugin_libs dc_distance_traveled_measurement)

add_library(dc_driving_type_measurement SHARED plugins/measurements/driving_type.cpp)
list(APPEND dc_measurement_plugin_libs dc_driving_type_measurement)

add_library(dc_ip_camera_measurement SHARED plugins/measurements/ip_camera.cpp)
list(APPEND dc_measurement_plugin_libs dc_ip_camera_measurement)

add_library(dc_map_measurement SHARED plugins/measurements/map.cpp)
list(APPEND dc_measurement_plugin_libs dc_map_measurement)

add_library(dc_memory_measurement SHARED
  plugins/measurements/memory.cpp
  plugins/measurements/system/linux_parser.cpp
  plugins/measurements/system/process.cpp
  plugins/measurements/system/processor.cpp
  plugins/measurements/system/system.cpp
)
list(APPEND dc_measurement_plugin_libs dc_memory_measurement)

add_library(dc_network_measurement SHARED plugins/measurements/network.cpp)
list(APPEND dc_measurement_plugin_libs dc_network_measurement)

add_library(dc_os_measurement SHARED
  plugins/measurements/os.cpp
  plugins/measurements/system/linux_parser.cpp
  plugins/measurements/system/process.cpp
  plugins/measurements/system/processor.cpp
  plugins/measurements/system/system.cpp
)
list(APPEND dc_measurement_plugin_libs dc_os_measurement)

add_library(dc_permissions_measurement SHARED plugins/measurements/permissions.cpp)
list(APPEND dc_measurement_plugin_libs dc_permissions_measurement)

add_library(dc_position_measurement SHARED plugins/measurements/position.cpp)
list(APPEND dc_measurement_plugin_libs dc_position_measurement)

add_library(dc_random_measurement SHARED plugins/measurements/random.cpp)
list(APPEND dc_measurement_plugin_libs dc_random_measurement)

add_library(dc_serial_interface_measurement SHARED plugins/measurements/serial_interface.cpp)
list(APPEND dc_measurement_plugin_libs dc_serial_interface_measurement)

add_library(dc_speed_measurement SHARED plugins/measurements/speed.cpp)
list(APPEND dc_measurement_plugin_libs dc_speed_measurement)

add_library(dc_storage_measurement SHARED plugins/measurements/storage.cpp)
list(APPEND dc_measurement_plugin_libs dc_storage_measurement)

add_library(dc_string_stamped_measurement SHARED plugins/measurements/string_stamped.cpp)
list(APPEND dc_measurement_plugin_libs dc_string_stamped_measurement)

add_library(dc_tcp_health_measurement SHARED plugins/measurements/tcp_health.cpp)
list(APPEND dc_measurement_plugin_libs dc_tcp_health_measurement)

add_library(dc_thermal_measurement SHARED plugins/measurements/thermal.cpp)
list(APPEND dc_measurement_plugin_libs dc_thermal_measurement)

add_library(dc_uptime_measurement SHARED
  plugins/measurements/uptime.cpp
  plugins/measurements/system/linux_parser.cpp
  plugins/measurements/system/process.cpp
  plugins/measurements/system/processor.cpp
  plugins/measurements/system/system.cpp
)
list(APPEND dc_measurement_plugin_libs dc_uptime_measurement)

foreach(measurement_plugin ${dc_measurement_plugin_libs})
  ament_target_dependencies(${measurement_plugin} ${dependencies})
  target_link_libraries(
    ${measurement_plugin}
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
    yaml-cpp::yaml-cpp
    ${UUID_LIBRARIES}
  )
  target_compile_definitions(${measurement_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()
target_link_libraries(dc_camera_measurement ZXing::ZXing opencv_calib3d)
target_include_directories(dc_ip_camera_measurement PRIVATE ${AVFORMAT_INCLUDE_DIRS} ${AVUTIL_INCLUDE_DIRS})
target_link_directories(dc_ip_camera_measurement PRIVATE ${AVFORMAT_LIBRARY_DIRS} ${AVUTIL_LIBRARY_DIRS})
target_link_libraries(dc_ip_camera_measurement ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES})
foreach(condition_plugin ${dc_condition_plugin_libs})
  ament_target_dependencies(${condition_plugin} ${dependencies})
  target_compile_definitions(${condition_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()
foreach(condition_plugin ${dc_condition_plugin_libs})
  ament_target_dependencies(${condition_plugin} ${dependencies})
  target_compile_definitions(${condition_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

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
  test_code_pose
  test_incident_releaser
  test_measurement_bool_equal
  test_measurement_buffering
  test_measurement_camera
  test_measurement_cmd_vel
  test_measurement_diagnostics
  test_measurement_distance_traveled
  test_measurement_double_equal
  test_measurement_double_inferior
  test_measurement_double_superior
  test_measurement_driving_type
  test_measurement_dummy
  test_measurement_exist
  test_measurement_gate_condition
  test_measurement_integer_equal
  test_measurement_integer_inferior
  test_measurement_integer_superior
  test_measurement_ip_camera
  test_measurement_list_bool_equal
  test_measurement_list_double_equal
  test_measurement_list_integer_equal
  test_measurement_list_string_equal
  test_measurement_map
  test_measurement_memory
  test_measurement_moving
  test_measurement_network
  test_measurement_os
  test_measurement_parameters
  test_measurement_permissions
  test_measurement_position
  test_measurement_random
  test_measurement_same_as_previous
  test_measurement_serial_interface
  test_measurement_speed
  test_measurement_storage
  test_measurement_string_match
  test_measurement_tcp_health
  test_measurement_thermal
  test_measurement_uptime
)

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
  # Only this test encodes a QR code (ZXing's writer) to render one back through the
  # camera model; every other test gets the reader transitively or not at all.
  target_link_libraries(${PROJECT_NAME}_test_code_pose ZXing::ZXing)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  ${dc_measurement_plugin_libs}
  ${dc_condition_plugin_libs}
)
ament_export_dependencies(${dependencies})
ament_package()
