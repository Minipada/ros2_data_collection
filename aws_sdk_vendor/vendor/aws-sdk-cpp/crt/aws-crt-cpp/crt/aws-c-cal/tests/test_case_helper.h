#ifndef AWS_CAL_TEST_CASE_HELPER_H
#define AWS_CAL_TEST_CASE_HELPER_H
/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/cal/cal.h>
#include <aws/cal/hash.h>
#include <aws/cal/hmac.h>
#include <aws/common/environment.h>
#include <aws/common/string.h>

#if !defined(BYO_CRYPTO) && !defined(AWS_OS_APPLE) && !defined(AWS_OS_WINDOWS)
#    include <openssl/crypto.h>
#endif

/**
 * If $AWS_TEST_FIPS env-var is set, turn on FIPS mode.
 * Then do normal aws_cal_library_init()
 */
static inline void aws_cal_library_test_init(struct aws_allocator *allocator) {
    struct aws_string *key_AWS_TEST_FIPS = aws_string_new_from_c_str(allocator, "AWS_TEST_FIPS");
    struct aws_string *val_AWS_TEST_FIPS = NULL;
    aws_get_environment_value(allocator, key_AWS_TEST_FIPS, &val_AWS_TEST_FIPS);
    bool is_fips_desired = val_AWS_TEST_FIPS != NULL;
    aws_string_destroy(key_AWS_TEST_FIPS);
    aws_string_destroy(val_AWS_TEST_FIPS);

    if (is_fips_desired) {
#if defined(OPENSSL_IS_AWSLC)
        AWS_FATAL_ASSERT(FIPS_mode_set(1) == 1 && "FIPS_mode_set(1) must succeed");
#else
        AWS_FATAL_ASSERT(!is_fips_desired && "AWS_TEST_FIPS is currently only supported with AWS-LC");
#endif
    }

    aws_cal_library_init(allocator);
}

static inline int s_verify_hmac_test_case(
    struct aws_allocator *allocator,
    struct aws_byte_cursor *input,
    struct aws_byte_cursor *secret,
    struct aws_byte_cursor *expected,
    aws_hmac_new_fn *new_fn) {

    aws_cal_library_test_init(allocator);

    /* test all possible segmentation lengths from 1 byte at a time to the entire
     * input. Using a do-while so that we still do 1 pass on 0-length input */
    size_t advance_i = 1;
    do {
        uint8_t output[128] = {0};
        struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output, AWS_ARRAY_SIZE(output));

        struct aws_hmac *hmac = new_fn(allocator, secret);
        ASSERT_NOT_NULL(hmac);

        struct aws_byte_cursor input_cpy = *input;

        while (input_cpy.len) {
            size_t max_advance = aws_min_size(input_cpy.len, advance_i);
            struct aws_byte_cursor segment = aws_byte_cursor_from_array(input_cpy.ptr, max_advance);
            ASSERT_SUCCESS(aws_hmac_update(hmac, &segment));
            aws_byte_cursor_advance(&input_cpy, max_advance);
        }

        size_t truncation_size = expected->len;

        ASSERT_SUCCESS(aws_hmac_finalize(hmac, &output_buf, truncation_size));
        ASSERT_BIN_ARRAYS_EQUALS(expected->ptr, expected->len, output_buf.buffer, output_buf.len);

        aws_hmac_destroy(hmac);
    } while (++advance_i <= input->len);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

static inline int s_verify_hash_test_case(
    struct aws_allocator *allocator,
    struct aws_byte_cursor *input,
    struct aws_byte_cursor *expected,
    aws_hash_new_fn *new_fn) {

    aws_cal_library_test_init(allocator);

    /* test all possible segmentation lengths from 1 byte at a time to the entire
     * input. Using a do-while so that we still do 1 pass on 0-length input */
    size_t advance_i = 1;
    do {
        uint8_t output[128] = {0};
        struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output, AWS_ARRAY_SIZE(output));

        struct aws_hash *hash = new_fn(allocator);
        ASSERT_NOT_NULL(hash);

        struct aws_byte_cursor input_cpy = *input;

        while (input_cpy.len) {
            size_t max_advance = aws_min_size(input_cpy.len, advance_i);
            struct aws_byte_cursor segment = aws_byte_cursor_from_array(input_cpy.ptr, max_advance);
            ASSERT_SUCCESS(aws_hash_update(hash, &segment));
            aws_byte_cursor_advance(&input_cpy, max_advance);
        }

        size_t truncation_size = expected->len;

        ASSERT_SUCCESS(aws_hash_finalize(hash, &output_buf, truncation_size));
        ASSERT_BIN_ARRAYS_EQUALS(expected->ptr, expected->len, output_buf.buffer, output_buf.len);

        aws_hash_destroy(hash);
    } while (++advance_i <= input->len);

    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

#endif /*AWS_CAL_TEST_CASE_HELPER_H*/
