# This can be used by downstream consumers (most likely CRT libraries)
# to build and embed aws-lc as libcrypto into the final library/binary
# This will create a crypto lib that is findable via find_package(LibCrypto)
# and is compatible with s2n and aws-c-cal

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/aws-lc)
execute_process(
    COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR}
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
    -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DBUILD_TESTING=OFF
    -DBUILD_LIBSSL=OFF
    -DDISABLE_GO=ON # disables codegen
    -DDISABLE_PERL=ON # disables codegen
    -DBUILD_SHARED_LIBS=${BUILD_SHARED_LIBS}
    ${CMAKE_CURRENT_SOURCE_DIR}/crt/aws-lc
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/aws-lc
    RESULT_VARIABLE BUILD_AWSLC_EXIT_CODE
)
if (NOT ${BUILD_AWSLC_EXIT_CODE} EQUAL 0)
    message(FATAL_ERROR "Failed to configure aws-lc")
endif()
execute_process(
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/aws-lc --config ${CMAKE_BUILD_TYPE} --target install
    RESULT_VARIABLE BUILD_AWSLC_EXIT_CODE
)
if (NOT ${BUILD_AWSLC_EXIT_CODE} EQUAL 0)
    message(FATAL_ERROR "Failed to build aws-lc")
endif()
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_BINARY_DIR}/aws-lc")
