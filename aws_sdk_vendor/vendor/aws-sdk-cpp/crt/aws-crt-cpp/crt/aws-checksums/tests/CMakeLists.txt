include(AwsLibFuzzer)
include(AwsTestHarness)
enable_testing()

file(GLOB TEST_HDRS "*.h")
file(GLOB TEST_SRC "*.c")

file(GLOB TESTS ${TEST_HDRS} ${TEST_SRC})

add_test_case(test_crc32c)
add_test_case(test_crc32)
add_test_case(test_crc32c_init)
add_test_case(test_crc32_init)
add_test_case(test_large_buffer_crc32)
add_test_case(test_crc64nvme)
add_test_case(test_crc64nvme_init)
add_test_case(test_large_buffer_crc64)

generate_test_driver(${PROJECT_NAME}-tests)
