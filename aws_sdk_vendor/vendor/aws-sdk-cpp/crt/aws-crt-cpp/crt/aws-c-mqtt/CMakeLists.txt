cmake_minimum_required(VERSION 3.9...3.31)
project(aws-c-mqtt C)

option(ASSERT_LOCK_HELD "Enable ASSERT_SYNCED_DATA_LOCK_HELD for checking thread issue" OFF)
if (ASSERT_LOCK_HELD)
    add_definitions(-DASSERT_LOCK_HELD)
endif()

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

file(GLOB AWS_MQTT_HEADERS
        "include/aws/mqtt/*.h"
)

file(GLOB AWS_MQTT5_HEADERS
        "include/aws/mqtt/v5/*.h"
)

file(GLOB AWS_MQTT_RR_HEADERS
        "include/aws/mqtt/request-response/*.h"
)

file(GLOB AWS_MQTT_PRIV_HEADERS
        "include/aws/mqtt/private/*.h"
        "include/aws/mqtt/private/request-response/*.h"
        "include/aws/mqtt/private/v5/*.h"
)

file(GLOB AWS_MQTT_PRIV_EXPOSED_HEADERS
        "include/aws/mqtt/private/mqtt_client_test_helper.h"
)

file(GLOB AWS_MQTT_SRC
        "source/*.c"
        "source/v5/*.c"
        "source/request-response/*.c"
)

file(GLOB MQTT_HEADERS
        ${AWS_MQTT_HEADERS}
        ${AWS_MQTT_PRIV_HEADERS}
)

file(GLOB AWS_MQTT5_HEADERS
        ${AWS_MQTT5_HEADERS}
)

file(GLOB MQTT_SRC
        ${AWS_MQTT_SRC}
)

add_library(${PROJECT_NAME} ${MQTT_HEADERS} ${MQTT_SRC})
aws_set_common_properties(${PROJECT_NAME})
aws_prepare_symbol_visibility_args(${PROJECT_NAME} "AWS_MQTT")
aws_check_headers(${PROJECT_NAME} ${AWS_MQTT_HEADERS} ${AWS_MQTT5_HEADERS})

aws_add_sanitizers(${PROJECT_NAME})

# We are not ABI stable yet
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

aws_use_package(aws-c-http)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_AWS_LIBS})
aws_prepare_shared_lib_exports(${PROJECT_NAME})

install(FILES ${AWS_MQTT_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/mqtt" COMPONENT Development)
install(FILES ${AWS_MQTT5_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/mqtt/v5" COMPONENT Development)
install(FILES ${AWS_MQTT_RR_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/mqtt/request-response" COMPONENT Development)
install(FILES ${AWS_MQTT_TESTING_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/testing/mqtt" COMPONENT Development)
install(FILES ${AWS_MQTT_PRIV_EXPOSED_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/mqtt/private" COMPONENT Development)

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

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
    if (NOT CMAKE_CROSSCOMPILING )
        add_subdirectory(bin/elastipubsub)
        add_subdirectory(bin/elastipubsub5)
        add_subdirectory(bin/elastishadow)
        add_subdirectory(bin/mqtt5canary)
    endif()
endif ()
