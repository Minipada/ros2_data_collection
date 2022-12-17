cmake_minimum_required(VERSION 3.5)
project(dc_demos)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
