cmake_minimum_required(VERSION 3.5)
project(dc_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(aws_robomaker_small_warehouse_world REQUIRED)

install(
  DIRECTORY launch maps models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/dc_simulation.dsv.in")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
