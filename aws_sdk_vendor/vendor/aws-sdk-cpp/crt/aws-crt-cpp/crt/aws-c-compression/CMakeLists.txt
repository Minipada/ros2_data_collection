cmake_minimum_required(VERSION 3.9...3.31)
project(aws-c-compression C)

if (NOT IN_SOURCE_BUILD)
    # this is required so we can use aws-c-common's CMake modules
    find_package(aws-c-common REQUIRED)
endif()

include(AwsCFlags)
include(AwsCheckHeaders)
include(AwsSharedLibSetup)
include(AwsSanitizers)
include(CheckCCompilerFlag)
include(AwsFindPackage)
include(GNUInstallDirs)

file(GLOB AWS_COMPRESSION_HEADERS
        "include/aws/compression/*.h"
        )

file(GLOB AWS_COMPRESSION_PRIV_HEADERS
        "include/aws/compression/private/*.h"
        )

file(GLOB AWS_COMPRESSION_TESTING_HEADERS
        "include/aws/testing/compression/*.h"
        "include/aws/testing/compression/*.inl"
        )

file(GLOB AWS_COMPRESSION_SRC
        "source/*.c"
        )

file(GLOB COMPRESSION_HEADERS
        ${AWS_COMPRESSION_HEADERS}
        ${AWS_COMPRESSION_PRIV_HEADERS}
        ${AWS_COMPRESSION_TESTING_HEADERS}
        )

file(GLOB COMPRESSION_SRC
        ${AWS_COMPRESSION_SRC}
        )

add_library(${PROJECT_NAME} ${LIBTYPE} ${COMPRESSION_HEADERS} ${COMPRESSION_SRC})
aws_set_common_properties(${PROJECT_NAME})
aws_prepare_symbol_visibility_args(${PROJECT_NAME} "AWS_COMPRESSION")

aws_add_sanitizers(${PROJECT_NAME})

# We are not ABI stable yet
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

aws_use_package(aws-c-common)
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_AWS_LIBS})

aws_prepare_shared_lib_exports(${PROJECT_NAME})

aws_check_headers(${PROJECT_NAME} ${AWS_COMPRESSION_HEADERS})
install(FILES ${AWS_COMPRESSION_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/compression")
install(FILES ${AWS_COMPRESSION_TESTING_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/testing/compression")

if (BUILD_SHARED_LIBS)
   set (TARGET_DIR "shared")
else()
   set (TARGET_DIR "static")
endif()

install(EXPORT "${PROJECT_NAME}-targets"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/${TARGET_DIR}"
        NAMESPACE AWS::
        COMPONENT Development)

configure_file("cmake/${PROJECT_NAME}-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/"
        COMPONENT Development)

option(BUILD_HUFFMAN_GENERATOR "Whether or not to build the aws-c-common-huffman-generator tool" OFF)
if (BUILD_HUFFMAN_GENERATOR)
        add_subdirectory(source/huffman_generator)
endif()

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif()
