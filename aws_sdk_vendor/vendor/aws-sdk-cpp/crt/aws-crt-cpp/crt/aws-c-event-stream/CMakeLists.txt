# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.9...3.31)
project (aws-c-event-stream C)

if (NOT IN_SOURCE_BUILD)
    # this is required so we can use aws-c-common's CMake modules
    find_package(aws-c-common REQUIRED)
endif()

include(AwsCFlags)
include(AwsSharedLibSetup)
include(AwsSanitizers)
include(CheckCCompilerFlag)
include(AwsFindPackage)
include(AwsCheckHeaders)
include(GNUInstallDirs)

file(GLOB AWS_EVENT_STREAM_HEADERS
     "include/aws/event-stream/*.h"
)

file(GLOB AWS_EVENT_STREAM_SRC
    "source/*.c"
)

if(WIN32)
     if(MSVC)
         source_group("Header Files\\aws\\event-stream" FILES ${AWS_EVENT_STREAM_HEADERS})
         source_group("Source Files" FILES ${AWS_EVENT_STREAM_SRC})
     endif()
endif()

file(GLOB EVENT_STREAM_HEADERS
    ${AWS_EVENT_STREAM_HEADERS}
)

file(GLOB EVENT_STREAM_SRC
    ${AWS_EVENT_STREAM_SRC}
)

add_library(${PROJECT_NAME} ${EVENT_STREAM_SRC})
aws_set_common_properties(${PROJECT_NAME})
aws_add_sanitizers(${PROJECT_NAME})
aws_prepare_symbol_visibility_args(${PROJECT_NAME} "AWS_EVENT_STREAM")

aws_check_headers(${PROJECT_NAME} ${AWS_EVENT_STREAM_HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0)

aws_use_package(aws-c-io)
aws_use_package(aws-c-common)
aws_use_package(aws-checksums)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_AWS_LIBS})

aws_prepare_shared_lib_exports(${PROJECT_NAME})

install(FILES ${AWS_EVENT_STREAM_HEADERS}
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/event-stream"
    COMPONENT Development)

if (BUILD_SHARED_LIBS)
   set (TARGET_DIR "shared")
else()
   set (TARGET_DIR "static")
endif()

install(EXPORT "${PROJECT_NAME}-targets"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/${TARGET_DIR}/"
    NAMESPACE AWS::
    COMPONENT Development)

configure_file("cmake/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/"
    COMPONENT Development)


include(CTest)
enable_testing()
if (BUILD_TESTING)
    add_subdirectory(tests)
    add_subdirectory(bin)
endif()
