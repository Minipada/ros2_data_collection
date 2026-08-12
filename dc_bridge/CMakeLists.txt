cmake_minimum_required(VERSION 3.16)
project(dc_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dc_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(msgpack-cxx REQUIRED)
find_package(Threads REQUIRED)
# Runtime message introspection for raw / generic-subscription mode (#227): the same
# type support pair `ros2 bag record -a` uses to handle types it was never built against.
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rcpputils REQUIRED)
# The AWS SDK for C++ (s3) comes from the aws_sdk_vendor package (built from source);
# colcon puts its install prefix on CMAKE_PREFIX_PATH so this resolves.
find_package(AWSSDK REQUIRED COMPONENTS s3)

# ROS-independent core: Forwarder/Supervisor/Readiness/TopicConfig/ConfigRenderer/
# vector_binary plus the Uploader logic (ADR-0005). Deliberately aws-free — the S3
# ObjectStore implementation lives in the node target below — so the Uploader is
# unit-tested with an in-memory fake and plain gtest, no cloud dependency.
add_library(dc_bridge_core
  src/forwarder.cpp
  src/readiness.cpp
  src/topic_config.cpp
  src/supervisor.cpp
  src/vector_binary.cpp
  src/render.cpp
  src/raw_config.cpp
  src/uploader/content_type.cpp
  src/uploader/group.cpp
  src/uploader/status.cpp
  src/uploader/multipart.cpp
  src/uploader/intent_queue.cpp
  src/uploader/uploader.cpp
  src/uploader/retention.cpp)
target_include_directories(dc_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dc_bridge_core PUBLIC
  nlohmann_json::nlohmann_json
  tomlplusplus::tomlplusplus
  msgpack-cxx
  Threads::Threads)

# Raw / generic-subscription mode (#227). Its own library rather than a file in the
# executable below, so the introspection→JSON conversion can be gtested against real
# message types (raw_json_test) without pulling in the node, Vector or the AWS SDK. Not
# part of dc_bridge_core: this half genuinely needs rclcpp and the type support
# libraries, which is exactly what the core is defined as not needing.
add_library(dc_bridge_ros
  src/raw_subscriptions.cpp)
target_include_directories(dc_bridge_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Plain signature (no PUBLIC/PRIVATE): ament_target_dependencies below uses the plain
# form of target_link_libraries internally, and CMake refuses to mix the two on one
# target.
target_link_libraries(dc_bridge_ros dc_bridge_core)
ament_target_dependencies(dc_bridge_ros
  rclcpp rosidl_runtime_cpp rosidl_typesupport_introspection_cpp rcpputils)

# The ROS node, plus the aws-sdk-cpp S3 ObjectStore implementation (the only Uploader
# piece that links the SDK).
add_executable(dc_bridge
  src/main.cpp
  src/bridge_node.cpp
  src/uploader/s3_object_store.cpp)
target_link_libraries(dc_bridge dc_bridge_core dc_bridge_ros ${AWSSDK_LINK_LIBRARIES})
ament_target_dependencies(dc_bridge rclcpp dc_interfaces std_msgs std_srvs)

install(TARGETS dc_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # All gtests link only the aws-free core (uploader_test uses an in-memory ObjectStore).
  foreach(t forwarder supervisor render misc uploader intent_queue retention raw_config)
    ament_add_gtest(${t}_test test/${t}_test.cpp)
    target_link_libraries(${t}_test dc_bridge_core)
    target_compile_definitions(${t}_test PRIVATE
      DC_BRIDGE_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/render")
  endforeach()

  # Raw mode against real message types and a real ROS graph (#227) — the one dc_bridge
  # test that needs both. dc_interfaces is a genuinely custom, non-std_msgs package, so
  # the "no compile-time knowledge of the type" claim is exercised end to end: the
  # converter is handed nothing but bytes and a type name. TIMEOUT because the firehose
  # and discovery cases are wall-clock waits on graph events.
  ament_add_gtest(raw_subscriptions_test test/raw_subscriptions_test.cpp TIMEOUT 180)
  target_link_libraries(raw_subscriptions_test dc_bridge_ros)
  ament_target_dependencies(raw_subscriptions_test rclcpp dc_interfaces std_msgs)
endif()

ament_package()
