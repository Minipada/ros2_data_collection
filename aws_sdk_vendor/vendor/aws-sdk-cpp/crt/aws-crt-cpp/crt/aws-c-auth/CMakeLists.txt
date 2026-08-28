
cmake_minimum_required(VERSION 3.9...3.31)
project(aws-c-auth C)

if (NOT IN_SOURCE_BUILD)
    # this is required so we can use aws-c-common's CMake modules
    find_package(aws-c-common REQUIRED)
endif()

include(AwsCFlags)
include(AwsCheckHeaders)
include(AwsSharedLibSetup)
include(AwsSanitizers)
include(AwsFindPackage)
include(GNUInstallDirs)

option(BUILD_RELOCATABLE_BINARIES
        "Build Relocatable Binaries, this will turn off features that will fail on older kernels than used for the build."
        OFF)

file(GLOB AWS_AUTH_ROOT_HEADERS
        "include/aws/auth/*.h"
        )

file(GLOB AWS_AUTH_PRIVATE_HEADERS
        "include/aws/auth/private/*.h"
        )

file(GLOB AWS_AUTH_ROOT_SRC
        "source/*.c"
        )

if (WIN32)
    if (MSVC)
        source_group("Header Files\\aws\\auth" FILES ${AWS_AUTH_HEADERS})
        source_group("Source Files" FILES ${AWS_AUTH_SRC})
    endif ()
endif()

file(GLOB AUTH_HEADERS
        ${AWS_AUTH_ROOT_HEADERS}
        ${AWS_AUTH_PRIVATE_HEADERS}
        )

file(GLOB AUTH_SRC
        ${AWS_AUTH_ROOT_SRC}
        )

add_library(${PROJECT_NAME} ${LIBTYPE} ${AUTH_HEADERS} ${AUTH_SRC})
aws_set_common_properties(${PROJECT_NAME})
aws_prepare_symbol_visibility_args(${PROJECT_NAME} "AWS_AUTH")
aws_check_headers(${PROJECT_NAME} ${AWS_AUTH_ROOT_HEADERS})

aws_add_sanitizers(${PROJECT_NAME})

# We are not ABI stable yet
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0)

target_compile_definitions(${PROJECT_NAME} PRIVATE -DCJSON_HIDE_SYMBOLS)

if (BUILD_RELOCATABLE_BINARIES)
    target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCOMPAT_MODE")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

aws_use_package(aws-c-sdkutils)
aws_use_package(aws-c-cal)
aws_use_package(aws-c-http)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_AWS_LIBS})

aws_prepare_shared_lib_exports(${PROJECT_NAME})

install(FILES ${AWS_AUTH_ROOT_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/auth" COMPONENT Development)

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
if (BUILD_TESTING)
    add_subdirectory(tests)
endif()
