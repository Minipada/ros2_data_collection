cmake_minimum_required(VERSION 3.9...3.31)

option(BUILD_DEPS "Builds aws common runtime dependencies as part of build. Turn off if you want to control your dependency chain." ON)
option(BYO_CRYPTO "Don't build a tls implementation or link against a crypto interface. This feature is only for unix builds currently" OFF)
option(USE_OPENSSL "Set this if you want to use your system's OpenSSL 1.0.2/1.1.1 compatible libcrypto" OFF)

# Let aws-iot-device-sdk-cpp-v2 report its own version in MQTT connections (instead of reporting aws-crt-cpp's version).
option(AWS_IOT_SDK_VERSION "Set the version reported by Aws::Iot::MqttClientConnectionConfigBuilder")

# Tests require environment variables setup in order to run properly.
# See https://github.com/awslabs/aws-crt-builder/blob/main/builder/actions/setup_cross_ci_crt_environment.py
# for how environment variables are setup.
# NOTE: Some environment variables use Mosquitto or Proxy servers, which are assumed to be installed
# locally if running the testing outside of CI/CD.
option(ENABLE_PROXY_INTEGRATION_TESTS "Whether or not to build and run the proxy integration tests that rely on a proxy server installed and running locally" OFF)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(AwsGetVersion)
aws_get_version(SIMPLE_VERSION FULL_VERSION GIT_HASH)
message(STATUS "AWS CRT C++ ${FULL_VERSION}")

project("aws-crt-cpp"
    LANGUAGES CXX C
    VERSION ${SIMPLE_VERSION})

include(CTest)
include(GNUInstallDirs)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 11)
endif()

if(NOT CMAKE_BUILD_TYPE)
    # setting this breaks C++ builds in visualc++, so don't do it.
    if(NOT WIN32)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo")
    endif()
endif()

set(GENERATED_ROOT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(GENERATED_INCLUDE_DIR "${GENERATED_ROOT_DIR}/include")
set(GENERATED_CONFIG_HEADER "${GENERATED_INCLUDE_DIR}/aws/crt/Config.h")
configure_file(include/aws/crt/Config.h.in ${GENERATED_CONFIG_HEADER} @ONLY)

if(BUILD_DEPS)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/crt/aws-c-common/cmake")

    include(AwsFindPackage)

    set(IN_SOURCE_BUILD ON)
    set(BUILD_TESTING_PREV ${BUILD_TESTING})
    set(BUILD_TESTING OFF)
    add_subdirectory(crt/aws-c-common)

    if(UNIX AND NOT APPLE AND NOT BYO_CRYPTO)
        if(NOT USE_OPENSSL)
            include(AwsPrebuildDependency)

            set(AWSLC_CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

            # temporarily disable certain warnings as errors for the aws-lc build
            if(NOT MSVC)
                check_c_compiler_flag(-Wno-stringop-overflow HAS_WNO_STRINGOP_OVERFLOW)

                if(HAS_WNO_STRINGOP_OVERFLOW)
                    set(AWSLC_CMAKE_C_FLAGS "${AWSLC_CMAKE_C_FLAGS} -Wno-stringop-overflow")
                endif()

                check_c_compiler_flag(-Wno-array-parameter HAS_WNO_ARRAY_PARAMETER)

                if(HAS_WNO_ARRAY_PARAMETER)
                    set(AWSLC_CMAKE_C_FLAGS "${AWSLC_CMAKE_C_FLAGS} -Wno-array-parameter")
                endif()
            endif()

            # s2n-tls uses libcrypto during its configuration, so we need to prebuild aws-lc.
            aws_prebuild_dependency(
                DEPENDENCY_NAME AWSLC
                SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/crt/aws-lc
                CMAKE_ARGUMENTS
                    -DDISABLE_GO=ON
                    -DDISABLE_PERL=ON
                    -DBUILD_LIBSSL=OFF
                    -DBUILD_TESTING=OFF
                    -DCMAKE_C_FLAGS=${AWSLC_CMAKE_C_FLAGS}
            )
        endif()

        set(UNSAFE_TREAT_WARNINGS_AS_ERRORS OFF CACHE BOOL "Disable warnings-as-errors when building S2N")
        add_subdirectory(crt/s2n)
    endif()

    add_subdirectory(crt/aws-c-sdkutils)
    add_subdirectory(crt/aws-c-io)
    add_subdirectory(crt/aws-c-cal)
    add_subdirectory(crt/aws-c-compression)
    add_subdirectory(crt/aws-c-http)
    add_subdirectory(crt/aws-c-auth)
    add_subdirectory(crt/aws-c-mqtt)
    add_subdirectory(crt/aws-checksums)
    add_subdirectory(crt/aws-c-event-stream)
    add_subdirectory(crt/aws-c-s3)
    set(BUILD_TESTING ${BUILD_TESTING_PREV})
else()
    # this is required so we can use aws-c-common's CMake modules
    find_package(aws-c-common REQUIRED)

    include(AwsFindPackage)
    set(IN_SOURCE_BUILD OFF)
endif()

include(AwsCFlags)
include(AwsSharedLibSetup)

file(GLOB AWS_CRT_HEADERS
    "include/aws/crt/*.h"
    ${GENERATED_CONFIG_HEADER}
)

file(GLOB AWS_CRT_AUTH_HEADERS
    "include/aws/crt/auth/*.h"
)

file(GLOB AWS_CRT_CHECKSUM_HEADERS
    "include/aws/crt/checksum/*.h"
)

file(GLOB AWS_CRT_CRYPTO_HEADERS
    "include/aws/crt/crypto/*.h"
)

file(GLOB AWS_CRT_IO_HEADERS
    "include/aws/crt/io/*.h"
)

file(GLOB AWS_CRT_IOT_HEADERS
    "include/aws/iot/*.h"
)

file(GLOB AWS_CRT_MQTT_HEADERS
    "include/aws/crt/mqtt/*.h"
)

file(GLOB AWS_CRT_HTTP_HEADERS
    "include/aws/crt/http/*.h"
)

file(GLOB AWS_CRT_ENDPOINT_HEADERS
    "include/aws/crt/endpoints/*.h"
)

file(GLOB AWS_CRT_CBOR_HEADERS
    "include/aws/crt/cbor/*.h"
)

file(GLOB AWS_CRT_PUBLIC_HEADERS
    ${AWS_CRT_HEADERS}
    ${AWS_CRT_AUTH_HEADERS}
    ${AWS_CRT_CHECKSUM_HEADERS}
    ${AWS_CRT_CRYPTO_HEADERS}
    ${AWS_CRT_IO_HEADERS}
    ${AWS_CRT_IOT_HEADERS}
    ${AWS_CRT_MQTT_HEADERS}
    ${AWS_CRT_HTTP_HEADERS}
    ${AWS_CRT_ENDPOINT_HEADERS}
    ${AWS_CRT_CBOR_HEADERS}
)

if(BUILD_DEPS)
    include(AwsCheckHeaders)
    aws_check_headers(${PROJECT_NAME} IS_CXX ${AWS_CRT_PUBLIC_HEADERS})
endif()

file(GLOB AWS_CRT_CPP_HEADERS
    ${AWS_CRT_PUBLIC_HEADERS}
)

file(GLOB AWS_CRT_SRC
    "source/*.cpp"
)

file(GLOB AWS_CRT_AUTH_SRC
    "source/auth/*.cpp"
)

file(GLOB AWS_CRT_CHECKSUM_SRC
    "source/checksum/*.cpp"
)

file(GLOB AWS_CRT_CRYPTO_SRC
    "source/crypto/*.cpp"
)

file(GLOB AWS_CRT_IO_SRC
    "source/io/*.cpp"
)

file(GLOB AWS_CRT_IOT_SRC
    "source/iot/*.cpp"
)

file(GLOB AWS_CRT_MQTT_SRC
    "source/mqtt/*.cpp"
)

file(GLOB AWS_CRT_HTTP_SRC
    "source/http/*.cpp"
)

file(GLOB AWS_CRT_ENDPOINTS_SRC
    "source/endpoints/*.cpp"
)

file(GLOB AWS_CRT_CBOR_SRC
    "source/cbor/*.cpp"
)

file(GLOB AWS_CRT_CPP_SRC
    ${AWS_CRT_SRC}
    ${AWS_CRT_AUTH_SRC}
    ${AWS_CRT_CHECKSUM_SRC}
    ${AWS_CRT_CRYPTO_SRC}
    ${AWS_CRT_IO_SRC}
    ${AWS_CRT_IOT_SRC}
    ${AWS_CRT_MQTT_SRC}
    ${AWS_CRT_HTTP_SRC}
    ${AWS_CRT_ENDPOINTS_SRC}
    ${AWS_CRT_CBOR_SRC}
)

if(WIN32)
    if(MSVC)
        source_group("Header Files\\aws\\crt" FILES ${AWS_CRT_HEADERS})
        source_group("Header Files\\aws\\crt\\auth" FILES ${AWS_CRT_AUTH_HEADERS})
        source_group("Header Files\\aws\\crt\\checksum" FILES ${AWS_CRT_CHECKSUM_HEADERS})
        source_group("Header Files\\aws\\crt\\crypto" FILES ${AWS_CRT_CRYPTO_HEADERS})
        source_group("Header Files\\aws\\crt\\io" FILES ${AWS_CRT_IO_HEADERS})
        source_group("Header Files\\aws\\iot" FILES ${AWS_CRT_IOT_HEADERS})
        source_group("Header Files\\aws\\crt\\mqtt" FILES ${AWS_CRT_MQTT_HEADERS})
        source_group("Header Files\\aws\\crt\\http" FILES ${AWS_CRT_HTTP_HEADERS})
        source_group("Header Files\\aws\\crt\\endpoints" FILES ${AWS_CRT_ENDPOINT_HEADERS})
        source_group("Header Files\\aws\\crt\\cbor" FILES ${AWS_CRT_CBOR_HEADERS})

        source_group("Source Files" FILES ${AWS_CRT_SRC})
        source_group("Source Files\\auth" FILES ${AWS_CRT_AUTH_SRC})
        source_group("Source Files\\checksum" FILES ${AWS_CRT_CHECKSUM_SRC})
        source_group("Source Files\\crypto" FILES ${AWS_CRT_CRYPTO_SRC})
        source_group("Source Files\\io" FILES ${AWS_CRT_IO_SRC})
        source_group("Source Files\\iot" FILES ${AWS_CRT_IOT_SRC})
        source_group("Source Files\\mqtt" FILES ${AWS_CRT_MQTT_SRC})
        source_group("Source Files\\http" FILES ${AWS_CRT_HTTP_SRC})
        source_group("Source Files\\endpoints" FILES ${AWS_CRT_ENDPOINTS_SRC})
        source_group("Source Files\\cbor" FILES ${AWS_CRT_CBOR_SRC})
    endif()
endif()

add_library(${PROJECT_NAME} ${AWS_CRT_CPP_HEADERS} ${AWS_CRT_CPP_SRC})

target_compile_definitions(${PROJECT_NAME} PRIVATE -DCJSON_HIDE_SYMBOLS)

if(AWS_IOT_SDK_VERSION)
    target_compile_definitions(${PROJECT_NAME} PRIVATE "-DAWS_IOT_SDK_VERSION=\"${AWS_IOT_SDK_VERSION}\"")
endif()

if(BUILD_SHARED_LIBS)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DAWS_CRT_CPP_USE_IMPORT_EXPORT)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DAWS_CRT_CPP_EXPORTS)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD ${CMAKE_CXX_STANDARD})

# Hide symbols by default
# Except where it causes problems, and the situation is weird enough that it's not worth investigating further.
#
# We've seen people set _GLIBCXX_USE_CXX11_ABI=0 which forces GCC to use it's pre-C++11 string implementation,
# which leads to crashes on shared-lib builds. Search every variant of CXX_FLAGS to see if it's set.
string(FIND "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${CMAKE_CXX_FLAGS_MINSIZEREL}"
       "-D_GLIBCXX_USE_CXX11_ABI=0" found_ancient_abi_flag)
if(found_ancient_abi_flag GREATER -1)
    message(WARNING "_GLIBCXX_USE_CXX11_ABI=0 is set. Making all symbols visible to prevent weird crashes")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0")
    # Ancient GCC leads to crashes in shared-lib builds
    # see: https://github.com/awslabs/aws-crt-cpp/pull/675
    message(WARNING "Ancient compiler (${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}). Making all symbols visible to prevent weird crashes")
else()
    set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
endif()

aws_prepare_symbol_visibility_args(${PROJECT_NAME} "AWS_CRT_CPP")

# set runtime library
if(MSVC)
    if(AWS_STATIC_MSVC_RUNTIME_LIBRARY OR STATIC_CRT)
        target_compile_options(${PROJECT_NAME} PRIVATE "/MT$<$<CONFIG:Debug>:d>")
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE "/MD$<$<CONFIG:Debug>:d>")
    endif()
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:DEBUG_BUILD>)

# set extra warning flags
if(AWS_WARNINGS_ARE_ERRORS)
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX /wd4068)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wno-long-long -pedantic -Werror)
    endif()
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>)

aws_use_package(aws-c-http)
aws_use_package(aws-c-mqtt)
aws_use_package(aws-c-cal)
aws_use_package(aws-c-auth)
aws_use_package(aws-c-common)
aws_use_package(aws-c-io)
aws_use_package(aws-checksums)
aws_use_package(aws-c-event-stream)
aws_use_package(aws-c-s3)

include(AwsSanitizers)
aws_add_sanitizers(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_AWS_LIBS})

install(FILES ${AWS_CRT_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt" COMPONENT Development)
install(FILES ${AWS_CRT_AUTH_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt/auth" COMPONENT Development)
install(FILES ${AWS_CRT_CHECKSUM_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt/checksum" COMPONENT Development)
install(FILES ${AWS_CRT_CRYPTO_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt/crypto" COMPONENT Development)
install(FILES ${AWS_CRT_IO_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt/io" COMPONENT Development)
install(FILES ${AWS_CRT_IOT_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/iot" COMPONENT Development)
install(FILES ${AWS_CRT_MQTT_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt/mqtt" COMPONENT Development)
install(FILES ${AWS_CRT_HTTP_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt/http" COMPONENT Development)
install(FILES ${AWS_CRT_ENDPOINT_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt/endpoints" COMPONENT Development)
install(FILES ${AWS_CRT_CBOR_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/crt/cbor" COMPONENT Development)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Runtime
)

if(BUILD_SHARED_LIBS)
    set(TARGET_DIR "shared")
else()
    set(TARGET_DIR "static")
endif()

install(EXPORT "${PROJECT_NAME}-targets"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/${TARGET_DIR}"
    NAMESPACE AWS::
    COMPONENT Development)

configure_file("cmake/${PROJECT_NAME}-config.cmake"
    "${GENERATED_ROOT_DIR}/${PROJECT_NAME}-config.cmake"
    @ONLY)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${GENERATED_ROOT_DIR}/${PROJECT_NAME}-config-version.cmake"
    COMPATIBILITY ExactVersion
)

install(FILES "${GENERATED_ROOT_DIR}/${PROJECT_NAME}-config.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/"
    COMPONENT Development)

install(FILES "${GENERATED_ROOT_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/"
    COMPONENT Development)

if(NOT CMAKE_CROSSCOMPILING)
    if(BUILD_TESTING)
        add_subdirectory(tests)

        if(NOT BYO_CRYPTO)
            add_subdirectory(bin/elasticurl_cpp)
            add_subdirectory(bin/mqtt5_app)
            add_subdirectory(bin/mqtt5_canary)
        endif()
    endif()
endif()
