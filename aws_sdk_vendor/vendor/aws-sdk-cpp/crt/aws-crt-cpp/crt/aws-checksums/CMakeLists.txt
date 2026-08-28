cmake_minimum_required(VERSION 3.9...3.31)

option(STATIC_CRT "Windows specific option that to specify static/dynamic run-time library" OFF)

project (aws-checksums C)

if (NOT IN_SOURCE_BUILD)
    # this is required so we can use aws-c-common's CMake modules
    find_package(aws-c-common REQUIRED)
endif()

include(AwsSIMD)
include(AwsCFlags)
include(AwsCheckHeaders)
include(AwsSharedLibSetup)
include(AwsSanitizers)
include(CheckCCompilerFlag)
include(AwsFindPackage)
include(AwsFeatureTests)
include(GNUInstallDirs)

file(GLOB AWS_CHECKSUMS_HEADERS
     "include/aws/checksums/*.h"
)

file(GLOB AWS_CHECKSUMS_PRIV_HEADERS
     "include/aws/checksums/private/*.h"
)

file(GLOB AWS_CHECKSUMS_SRC
    "source/*.c"
)

if(MSVC)
     source_group("Header Files\\aws\\checksums" FILES ${AWS_CHECKSUMS_HEADERS})
     source_group("Source Files" FILES ${AWS_CHECKSUMS_SRC})
endif()

file(GLOB CHECKSUMS_COMBINED_HEADERS
    ${AWS_CHECKSUMS_HEADERS}
    ${AWS_CHECKSUMS_PRIV_HEADERS}
)

file(GLOB CHECKSUMS_COMBINED_SRC
    ${AWS_CHECKSUMS_SRC}
    ${AWS_CHECKSUMS_PLATFORM_SOURCE}
)

add_library(${PROJECT_NAME} ${CHECKSUMS_COMBINED_HEADERS} ${CHECKSUMS_COMBINED_SRC})

aws_set_common_properties(${PROJECT_NAME})
aws_prepare_symbol_visibility_args(${PROJECT_NAME} "AWS_CHECKSUMS")
aws_check_headers(${PROJECT_NAME} ${AWS_CHECKSUMS_HEADERS})

aws_add_sanitizers(${PROJECT_NAME})

# We are not ABI stable yet
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0)

if (USE_CPU_EXTENSIONS)
    if (AWS_ARCH_INTEL)
        file (GLOB AWS_ARCH_INTEL_SRC
                "source/intel/*.c"
                )

        if (MSVC)
            file(GLOB AWS_ARCH_INTRIN_SRC
                    "source/intel/intrin/*.c"
                )

            source_group("Source Files\\intel" FILES ${AWS_ARCH_INTEL_SRC})
            source_group("Source Files\\intel\\intrin" FILES ${AWS_ARCH_INTRIN_SRC})
        else()
            if (AWS_HAVE_GCC_INLINE_ASM)
                simd_append_source_and_features(${PROJECT_NAME} "source/intel/asm/crc32c_sse42_asm.c" ${AWS_SSE4_2_FLAG})
            endif()
        endif()


        set(UBER_FILE_FLAGS "")
        if (AWS_HAVE_AVX512_INTRINSICS)
            list(APPEND UBER_FILE_FLAGS ${AWS_AVX512_FLAG})
            list(APPEND UBER_FILE_FLAGS ${AWS_AVX512vL_FLAG})
            list(APPEND UBER_FILE_FLAGS ${AWS_AVX2_FLAG})
            simd_append_source_and_features(${PROJECT_NAME} "source/intel/intrin/crc64nvme_avx512.c" ${AWS_AVX512_FLAG} ${AWS_AVX512vL_FLAG} ${AWS_AVX2_FLAG} ${AWS_CLMUL_FLAG} ${AWS_SSE4_2_FLAG})

        endif()

        if (AWS_HAVE_CLMUL)
            list(APPEND UBER_FILE_FLAGS ${AWS_CLMUL_FLAG})
        endif()

        list(APPEND UBER_FILE_FLAGS "${AWS_SSE4_2_FLAG}")

       # this file routes all of the implementations together based on available cpu features. It gets built regardless
       # of which flags exist. The c file sorts it out.
       simd_append_source_and_features(${PROJECT_NAME} "source/intel/intrin/crc32c_sse42_avx512.c" ${UBER_FILE_FLAGS})

        if (AWS_HAVE_CLMUL)
            simd_append_source_and_features(${PROJECT_NAME} "source/intel/intrin/crc64nvme_clmul.c" ${AWS_AVX2_FLAG} ${AWS_CLMUL_FLAG} ${AWS_SSE4_2_FLAG})
        endif()


    elseif(AWS_ARCH_ARM64 OR (AWS_ARCH_ARM32 AND AWS_HAVE_ARM32_CRC))
        simd_append_source_and_features(${PROJECT_NAME} "source/arm/crc32c_arm.c" ${AWS_ARMv8_1_FLAG})
        simd_append_source_and_features(${PROJECT_NAME} "source/arm/crc64_arm.c" ${AWS_ARMv8_1_FLAG})

        if (MSVC)
            file(GLOB AWS_ARCH_SRC
                    "source/arm/*.c"
                    )
            source_group("Source Files\\arm" FILES ${AWS_ARCH_SRC})
        endif()
    endif()
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

aws_use_package(aws-c-common)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_AWS_LIBS})
aws_prepare_shared_lib_exports(${PROJECT_NAME})

install(FILES ${AWS_CHECKSUMS_HEADERS} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aws/checksums" COMPONENT Development)

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
    add_subdirectory(bin/benchmark)
endif ()
