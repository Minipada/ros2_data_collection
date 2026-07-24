cmake_minimum_required(VERSION 3.5)
project(dc_bringup)

set(dependencies
  dc_common
  dc_measurements
  nav2_lifecycle_manager
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

dc_package()

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/bridge_ready_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bridge_ready_gate)

ament_package()
