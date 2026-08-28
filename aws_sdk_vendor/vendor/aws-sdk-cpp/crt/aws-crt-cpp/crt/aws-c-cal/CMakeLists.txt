# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.9...3.31)

project(aws-c-cal LANGUAGES C VERSION 0.1.0)

set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

option(BYO_CRYPTO "Set this if you want to provide your own cryptography implementation. This will cause the defaults to not be compiled." OFF)
option(USE_OPENSSL "Set this if you want to use your system's OpenSSL 1.0.2/1.1.1 compatible libcrypto" OFF)
option(AWS_USE_CRYPTO_SHARED_LIBS "Force c-cal to use shared libs in Findcrypto" OFF)
option(AWS_USE_LIBCRYPTO_TO_SUPPORT_ED25519_EVERYWHERE "Experimental feature to support ED25519 keygen on platforms that do not support it in os libs (i.e. win/mac)" OFF)

if (NOT IN_SOURCE_BUILD)
    # this is required so we can use aws-c-common's CMake modules
    find_package(aws-c-common REQUIRED)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/modules")

include(AwsCFlags)
include(AwsCheckHeaders)
include(AwsSharedLibSetup)
include(AwsSanitizers)
include(AwsFindPackage)
include(GNUInstallDirs)

file(GLOB AWS_CAL_HEADERS
    "include/aws/cal/*.h"
)

file(GLOB AWS_CAL_SRC
    "source/*.c"
)

if (WIN32)

    if (NOT BYO_CRYPTO)
        file(GLOB AWS_CAL_OS_SRC
            "source/windows/*.c"
        )

        if (AWS_USE_LIBCRYPTO_TO_SUPPORT_ED25519_EVERYWHERE)
            list(APPEND AWS_CAL_OS_SRC "source/shared/ed25519.c")
            list(APPEND AWS_CAL_OS_SRC "source/shared/lccrypto_common.c")
        else()
            list(APPEND AWS_CAL_OS_SRC "source/shared/ed25519_noop.c")
        endif()

        if (AWS_SUPPORT_WIN7)
            set(PLATFORM_LIBS bcrypt)
        else()
            set(PLATFORM_LIBS ncrypt)
        endif()
    endif()

    if (MSVC)
        source_group("Header Files\\aws\\cal" FILES ${AWS_CAL_HEADERS})
        source_group("Source Files" FILES ${AWS_CAL_SRC})
        source_group("Source Files\\windows" FILES ${AWS_CAL_OS_SRC})
    endif ()

elseif (APPLE)
    if (NOT BYO_CRYPTO)
        file(GLOB AWS_CAL_OS_SRC
            "source/darwin/*.c"
        )

        if (AWS_USE_LIBCRYPTO_TO_SUPPORT_ED25519_EVERYWHERE)
            list(APPEND AWS_CAL_OS_SRC "source/shared/ed25519.c")
            list(APPEND AWS_CAL_OS_SRC "source/shared/lccrypto_common.c")
        else()
            list(APPEND AWS_CAL_OS_SRC "source/shared/ed25519_noop.c")
        endif()

        find_library(SECURITY_LIB Security)
        if (NOT SECURITY_LIB)
           message(FATAL_ERROR "Security Framework not found")
        endif ()


        find_library(COREFOUNDATION_LIB CoreFoundation)
        if(NOT COREFOUNDATION_LIB)
           message(FATAL_ERROR "CoreFoundation Framework not found")
        endif()

        list(APPEND PLATFORM_LIBS "-framework Security -framework CoreFoundation")
    endif()
else ()
    if (NOT BYO_CRYPTO)
        file(GLOB AWS_CAL_OS_SRC
            "source/unix/*.c"
            "source/shared/ed25519.c"
            "source/shared/lccrypto_common.c"
        )
    endif()
endif()

if (NOT BYO_CRYPTO)
    if ((NOT WIN32 AND NOT APPLE) OR AWS_USE_LIBCRYPTO_TO_SUPPORT_ED25519_EVERYWHERE)
        if (USE_OPENSSL AND NOT ANDROID)
            find_package(OpenSSL REQUIRED)
            find_package(Threads REQUIRED)
            list(APPEND PLATFORM_LIBS OpenSSL::Crypto Threads::Threads)
            message(STATUS "Using libcrypto from system: ${OPENSSL_CRYPTO_LIBRARY}")
        elseif(NOT USE_OPENSSL AND IN_SOURCE_BUILD)
            if (TARGET crypto)
                message(STATUS "Using libcrypto from AWS-LC")
                list(APPEND PLATFORM_LIBS  crypto)
            elseif(AWSLC_PREBUILT)
                message(STATUS "Using prebuilt libcrypto from AWS-LC")
                find_package(crypto REQUIRED)
                list(APPEND PLATFORM_LIBS  AWS::crypto)
            else()
                message(FATAL_ERROR "Target crypto is not defined, failed to find libcrypto.")
            endif()
        else()
            #  note aws_use_package() does this for you, except it appends to the public link targets
            # which we probably don't want for this case where we want the crypto dependency private
            if (IN_SOURCE_BUILD)
                list(APPEND PLATFORM_LIBS crypto)
            else()
                find_package(crypto REQUIRED)
                list(APPEND PLATFORM_LIBS AWS::crypto)
            endif()
        endif()
    endif()
endif()

file(GLOB CAL_HEADERS
    ${AWS_CAL_HEADERS}
)

file(GLOB CAL_SRC
    ${AWS_CAL_SRC}
    ${AWS_CAL_OS_SRC}
)

add_library(${PROJECT_NAME} ${CAL_SRC})
aws_set_common_properties(${PROJECT_NAME} NO_WEXTRA)
aws_prepare_symbol_visibility_args(${PROJECT_NAME} "AWS_CAL")
aws_add_sanitizers(${PROJECT_NAME})

aws_use_package(aws-c-common)
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_AWS_LIBS} ${PLATFORM_LIBS})

if (BYO_CRYPTO)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DBYO_CRYPTO)
endif()

if (AWS_USE_LIBCRYPTO_TO_SUPPORT_ED25519_EVERYWHERE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DAWS_USE_LIBCRYPTO_TO_SUPPORT_ED25519_EVERYWHERE)
endif()

# Our ABI is not yet stable
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
# When we install, the generated header will be at the INSTALL_INTERFACE:include location,
# but at build time we need to explicitly include this here
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${GENERATED_INCLUDE_DIR}>)

aws_prepare_shared_lib_exports(${PROJECT_NAME})

configure_file("cmake/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  @ONLY)

aws_check_headers(${PROJECT_NAME} ${AWS_CAL_HEADERS})
install(FILES ${AWS_CAL_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/cal" COMPONENT Development)

if (BUILD_SHARED_LIBS)
   set (TARGET_DIR "shared")
else()
   set (TARGET_DIR "static")
endif()

install(EXPORT "${PROJECT_NAME}-targets"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/${TARGET_DIR}/"
    NAMESPACE AWS::
    COMPONENT Development)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
    COMPONENT Development)

list(APPEND EXPORT_MODULES
    "cmake/modules/Findcrypto.cmake"
    )

install(FILES ${EXPORT_MODULES}
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/modules"
        COMPONENT Development)

if (NOT CMAKE_CROSSCOMPILING AND NOT BYO_CRYPTO)
    include(CTest)
    if (BUILD_TESTING)
        add_subdirectory(bin/sha256_profile)
        add_subdirectory(bin/produce_x_platform_fuzz_corpus)
        add_subdirectory(bin/run_x_platform_fuzz_corpus)
        add_subdirectory(tests)
    endif()
endif()
