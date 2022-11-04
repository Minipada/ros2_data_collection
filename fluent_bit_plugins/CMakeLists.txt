cmake_minimum_required(VERSION 3.5)
project(fluent_bit_plugins)

set(dependencies
  dc_interfaces
  fluent_bit_vendor
  rcl
  rclc
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(fluent_bit REQUIRED)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/sanitizers-cmake/cmake" ${CMAKE_MODULE_PATH})
find_package(Sanitizers)

# C plugins
list(APPEND c_plugins "in_ros2")

foreach(c_plugin ${c_plugins})
  set(LIBRARY_NAME "flb-${c_plugin}")
  add_library(${LIBRARY_NAME} SHARED "src/c/${c_plugin}.c")
  ament_target_dependencies(${LIBRARY_NAME} ${dependencies})
  add_sanitizers(${LIBRARY_NAME})
  set_target_properties(${LIBRARY_NAME} PROPERTIES PREFIX "")

  list(APPEND c_libs ${LIBRARY_NAME})
endforeach()

include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${c_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(include)
ament_export_library_names(${c_libs})
ament_export_dependencies(${dependencies})
ament_package()
