cmake_minimum_required(VERSION 3.5)
project(dc_util)

set(dependencies
  dc_common
  lifecycle_msgs
  nav2_util
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

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
