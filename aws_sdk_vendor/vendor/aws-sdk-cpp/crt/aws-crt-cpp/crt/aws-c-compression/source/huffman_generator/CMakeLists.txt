file(GLOB GENERATIR_SRC "generator.c")

set(GENERATOR_BINARY_NAME ${CMAKE_PROJECT_NAME}-huffman-generator)

add_executable(${GENERATOR_BINARY_NAME} ${GENERATIR_SRC})
aws_set_common_properties(${GENERATOR_BINARY_NAME})
aws_add_sanitizers(${GENERATOR_BINARY_NAME})
target_include_directories(${GENERATOR_BINARY_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        )

if (MSVC)
    target_compile_definitions(${GENERATOR_BINARY_NAME} PRIVATE "-D_CRT_SECURE_NO_WARNINGS")
endif ()


install(
        TARGETS ${GENERATOR_BINARY_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
