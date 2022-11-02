cmake_minimum_required(VERSION 3.8)
project(dc_interfaces)


set(dependencies
  builtin_interfaces
  rosidl_default_generators
  std_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()


rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/StringStamped.msg"
  DEPENDENCIES "${dependencies}"
)

ament_package()
