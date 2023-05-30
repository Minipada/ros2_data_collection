cmake_minimum_required(VERSION 3.5)
project(dc_util)

set(dependencies
  dc_common
  lifecycle_msgs
  nav2_util
  nlohmann_json
  rclcpp
  rclcpp_lifecycle
  yaml_cpp_vendor
)
find_package(Boost REQUIRED)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

dc_package()

set(library_name base64_library)

add_library(${library_name} SHARED
  src/base64.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

ament_export_targets(${library_name} HAS_LIBRARY_TARGET)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
