include(CTest)
include(AwsTestHarness)
include(AwsLibFuzzer)

enable_testing()

file(GLOB TEST_SRC "*.c")
file(GLOB TEST_HDRS "*.h")
file(GLOB TESTS ${TEST_HDRS} ${TEST_SRC})

add_test_case(library_init)

add_test_case(huffman_symbol_encoder)
add_test_case(huffman_encoder)
add_test_case(huffman_encoder_all_code_points)
add_test_case(huffman_encoder_partial_output)
add_test_case(huffman_encoder_exact_output)

add_test_case(huffman_symbol_decoder)
add_test_case(huffman_decoder)
add_test_case(huffman_decoder_all_code_points)
add_test_case(huffman_decoder_partial_input)
add_test_case(huffman_decoder_partial_output)
add_test_case(huffman_decoder_allow_growth)

add_test_case(huffman_transitive)
add_test_case(huffman_transitive_even_bytes)
add_test_case(huffman_transitive_all_code_points)
add_test_case(huffman_transitive_chunked)

generate_test_driver(${PROJECT_NAME}-tests)
if(MSVC)
    target_compile_definitions(${PROJECT_NAME}-tests PRIVATE "-D_CRT_SECURE_NO_WARNINGS")
endif()

file(GLOB FUZZ_TESTS "fuzz/*.c")
aws_add_fuzz_tests("${FUZZ_TESTS}" "test_huffman_static.c" "")
