/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/testing/aws_test_harness.h>

#include <aws/auth/credentials.h>
#include <aws/auth/private/key_derivation.h>
#include <aws/cal/ecc.h>
#include <aws/common/encoding.h>
#include <aws/common/string.h>

struct aws_be_add_one_test {
    uint8_t *input;
    size_t input_length;
    uint8_t *expected_output;
    size_t expected_output_length;
};

static uint8_t add_one_input_1[] = {0x00, 0x00, 0x00};
static uint8_t add_one_expected_output_1[] = {0x00, 0x00, 0x01};
static uint8_t add_one_input_2[] = {0x00, 0x00, 0xFF};
static uint8_t add_one_expected_output_2[] = {0x00, 0x01, 0x00};
static uint8_t add_one_input_3[] = {0x00, 0xFF, 0xFF};
static uint8_t add_one_expected_output_3[] = {0x01, 0x00, 0x00};
static uint8_t add_one_input_4[] = {0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t add_one_expected_output_4[] = {0x00, 0x00, 0x00, 0x00};

static struct aws_be_add_one_test s_be_add_one_test_cases[] = {
    {
        .input = add_one_input_1,
        .input_length = AWS_ARRAY_SIZE(add_one_input_1),
        .expected_output = add_one_expected_output_1,
        .expected_output_length = AWS_ARRAY_SIZE(add_one_expected_output_1),
    },
    {
        .input = add_one_input_2,
        .input_length = AWS_ARRAY_SIZE(add_one_input_2),
        .expected_output = add_one_expected_output_2,
        .expected_output_length = AWS_ARRAY_SIZE(add_one_expected_output_2),
    },
    {
        .input = add_one_input_3,
        .input_length = AWS_ARRAY_SIZE(add_one_input_3),
        .expected_output = add_one_expected_output_3,
        .expected_output_length = AWS_ARRAY_SIZE(add_one_expected_output_3),
    },
    {
        .input = add_one_input_4,
        .input_length = AWS_ARRAY_SIZE(add_one_input_4),
        .expected_output = add_one_expected_output_4,
        .expected_output_length = AWS_ARRAY_SIZE(add_one_expected_output_4),
    },
};

static int s_be_sequence_add_one(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_be_add_one_test_cases); ++i) {
        struct aws_be_add_one_test *test_case = &s_be_add_one_test_cases[i];

        struct aws_byte_buf input = {
            .len = test_case->input_length,
            .buffer = test_case->input,
            .capacity = test_case->input_length,
            .allocator = NULL,
        };

        aws_be_bytes_add_one_constant_time(&input);

        ASSERT_BIN_ARRAYS_EQUALS(
            test_case->expected_output, test_case->expected_output_length, input.buffer, input.len);
    }

    return 0;
}

AWS_TEST_CASE(be_sequence_add_one, s_be_sequence_add_one);

struct aws_be_compare_test {
    uint8_t *lhs;
    size_t lhs_length;
    uint8_t *rhs;
    size_t rhs_length;
    int expected_return_value;
    int expected_result;
};

static uint8_t compare_lhs_bad[] = {0x00, 0x00, 0x00};
static uint8_t compare_rhs_bad[] = {0x00, 0x00, 0x01, 0xFF};

static uint8_t compare_lhs_1[] = {0x00, 0x00, 0x00};
static uint8_t compare_rhs_1[] = {0x00, 0x00, 0x01};
static uint8_t compare_lhs_2[] = {0xAB, 0xCD, 0x80, 0xFF, 0x01, 0x0A};
static uint8_t compare_rhs_2[] = {0xAB, 0xCD, 0x80, 0xFF, 0x01, 0x0A};
static uint8_t compare_lhs_3[] = {0xFF, 0xCD, 0x80, 0xFF, 0x01, 0x0A};
static uint8_t compare_rhs_3[] = {0xFE, 0xCD, 0x80, 0xFF, 0x01, 0x0A};

static struct aws_be_compare_test s_be_compare_test_cases[] = {
    /*
     * Failure cases
     */
    {
        .lhs = compare_lhs_bad,
        .lhs_length = AWS_ARRAY_SIZE(compare_lhs_bad),
        .rhs = compare_rhs_bad,
        .rhs_length = AWS_ARRAY_SIZE(compare_rhs_bad),
        .expected_return_value = AWS_OP_ERR,
        .expected_result = 0,
    },

    /*
     * Success cases
     */
    {
        .lhs = compare_lhs_1,
        .lhs_length = AWS_ARRAY_SIZE(compare_lhs_1),
        .rhs = compare_rhs_1,
        .rhs_length = AWS_ARRAY_SIZE(compare_rhs_1),
        .expected_return_value = AWS_OP_SUCCESS,
        .expected_result = -1,
    },
    {
        .lhs = compare_lhs_2,
        .lhs_length = AWS_ARRAY_SIZE(compare_lhs_2),
        .rhs = compare_rhs_2,
        .rhs_length = AWS_ARRAY_SIZE(compare_rhs_2),
        .expected_return_value = AWS_OP_SUCCESS,
        .expected_result = 0,
    },
    {
        .lhs = compare_lhs_3,
        .lhs_length = AWS_ARRAY_SIZE(compare_lhs_3),
        .rhs = compare_rhs_3,
        .rhs_length = AWS_ARRAY_SIZE(compare_rhs_3),
        .expected_return_value = AWS_OP_SUCCESS,
        .expected_result = 1,
    },
};

static int s_be_sequence_compare(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    (void)allocator;

    for (size_t i = 0; i < AWS_ARRAY_SIZE(s_be_compare_test_cases); ++i) {
        struct aws_be_compare_test *test_case = &s_be_compare_test_cases[i];

        struct aws_byte_buf lhs = {
            .len = test_case->lhs_length,
            .buffer = test_case->lhs,
            .capacity = test_case->lhs_length,
            .allocator = NULL,
        };

        struct aws_byte_buf rhs = {
            .len = test_case->rhs_length,
            .buffer = test_case->rhs,
            .capacity = test_case->rhs_length,
            .allocator = NULL,
        };

        int comparison_result = 0;
        int result = aws_be_bytes_compare_constant_time(&lhs, &rhs, &comparison_result);

        ASSERT_INT_EQUALS(test_case->expected_return_value, result);
        if (result == AWS_OP_SUCCESS) {
            ASSERT_INT_EQUALS(test_case->expected_result, comparison_result);
        }

        int swapped_comparison_result = 0;
        int swapped_result = aws_be_bytes_compare_constant_time(&rhs, &lhs, &swapped_comparison_result);
        ASSERT_INT_EQUALS(test_case->expected_return_value, swapped_result);
        if (swapped_result == AWS_OP_SUCCESS) {
            ASSERT_INT_EQUALS(-test_case->expected_result, swapped_comparison_result);
        }
    }

    return 0;
}

AWS_TEST_CASE(be_sequence_compare, s_be_sequence_compare);

AWS_STATIC_STRING_FROM_LITERAL(s_ecc_derive_fixed_access_key_id_test_value, "AKISORANDOMAASORANDOM");
AWS_STATIC_STRING_FROM_LITERAL(
    s_ecc_derive_fixed_secret_access_key_test_value,
    "q+jcrXGc+0zWN6uzclKVhvMmUsIfRPa4rlRandom");

/*
 * Values derived in synchronicity with Golang and IAM implementations
 */
#ifndef __APPLE__
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_fixed_pub_x,
    "15d242ceebf8d8169fd6a8b5a746c41140414c3b07579038da06af89190fffcb");
AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_fixed_pub_y,
    "0515242cedd82e94799482e4c0514b505afccf2c0c98d6a553bf539f424c5ec0");
#endif /* __APPLE__ */

AWS_STATIC_STRING_FROM_LITERAL(
    s_expected_fixed_private_key,
    "7fd3bd010c0d9c292141c2b77bfbde1042c92e6836fff749d1269ec890fca1bd");

static int s_verify_fixed_ecc_key_public(struct aws_ecc_key_pair *key, struct aws_allocator *allocator) {
#ifdef __APPLE__
    (void)key;
    (void)allocator;
#else
    aws_ecc_key_pair_derive_public_key(key);

    struct aws_byte_cursor pub_x_cursor;
    AWS_ZERO_STRUCT(pub_x_cursor);
    struct aws_byte_cursor pub_y_cursor;
    AWS_ZERO_STRUCT(pub_y_cursor);

    aws_ecc_key_pair_get_public_key(key, &pub_x_cursor, &pub_y_cursor);

    struct aws_byte_buf pub_coord_x;
    ASSERT_SUCCESS(aws_byte_buf_init(&pub_coord_x, allocator, 128));

    ASSERT_SUCCESS(aws_hex_encode(&pub_x_cursor, &pub_coord_x));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_fixed_pub_x->bytes, s_expected_fixed_pub_x->len, pub_coord_x.buffer, pub_coord_x.len);

    struct aws_byte_buf pub_coord_y;
    ASSERT_SUCCESS(aws_byte_buf_init(&pub_coord_y, allocator, 128));

    ASSERT_SUCCESS(aws_hex_encode(&pub_y_cursor, &pub_coord_y));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_fixed_pub_y->bytes, s_expected_fixed_pub_y->len, pub_coord_y.buffer, pub_coord_y.len);

    aws_byte_buf_clean_up(&pub_coord_x);
    aws_byte_buf_clean_up(&pub_coord_y);
#endif /* __APPLE__ */

    return AWS_OP_SUCCESS;
}

static int s_verify_fixed_ecc_key_private(struct aws_ecc_key_pair *key, struct aws_allocator *allocator) {
    struct aws_byte_cursor private_key_cursor;
    AWS_ZERO_STRUCT(private_key_cursor);

    aws_ecc_key_pair_get_private_key(key, &private_key_cursor);

    struct aws_byte_buf private_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&private_buf, allocator, 128));

    ASSERT_SUCCESS(aws_hex_encode(&private_key_cursor, &private_buf));
    ASSERT_BIN_ARRAYS_EQUALS(
        s_expected_fixed_private_key->bytes, s_expected_fixed_private_key->len, private_buf.buffer, private_buf.len);

    aws_byte_buf_clean_up(&private_buf);

    return AWS_OP_SUCCESS;
}

static int s_credentials_derive_ecc_key_fixed(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct aws_credentials *creds = aws_credentials_new_from_string(
        allocator,
        s_ecc_derive_fixed_access_key_id_test_value,
        s_ecc_derive_fixed_secret_access_key_test_value,
        NULL,
        UINT64_MAX);

    struct aws_ecc_key_pair *derived_key = aws_ecc_key_pair_new_ecdsa_p256_key_from_aws_credentials(allocator, creds);
    ASSERT_TRUE(derived_key != NULL);

    ASSERT_SUCCESS(s_verify_fixed_ecc_key_public(derived_key, allocator));
    ASSERT_SUCCESS(s_verify_fixed_ecc_key_private(derived_key, allocator));

    aws_ecc_key_pair_release(derived_key);
    aws_credentials_release(creds);

    aws_auth_library_clean_up();

    return 0;
}

AWS_TEST_CASE(credentials_derive_ecc_key_fixed, s_credentials_derive_ecc_key_fixed);

static int s_credentials_new_ecc_fixed(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct aws_credentials *creds = aws_credentials_new_from_string(
        allocator,
        s_ecc_derive_fixed_access_key_id_test_value,
        s_ecc_derive_fixed_secret_access_key_test_value,
        NULL,
        UINT64_MAX);

    struct aws_credentials *derived_credentials = aws_credentials_new_ecc_from_aws_credentials(allocator, creds);
    ASSERT_TRUE(derived_credentials != NULL);

    struct aws_ecc_key_pair *derived_key = aws_credentials_get_ecc_key_pair(derived_credentials);

    ASSERT_SUCCESS(s_verify_fixed_ecc_key_public(derived_key, allocator));
    ASSERT_SUCCESS(s_verify_fixed_ecc_key_private(derived_key, allocator));

    aws_credentials_release(derived_credentials);
    aws_credentials_release(creds);

    aws_auth_library_clean_up();

    return 0;
}

AWS_TEST_CASE(credentials_new_ecc_fixed, s_credentials_new_ecc_fixed);

AWS_STATIC_STRING_FROM_LITERAL(
    s_ecc_derive_long_access_key_id_test_value,
    "AKISORANDOMAASORANDOMFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFf");
AWS_STATIC_STRING_FROM_LITERAL(
    s_ecc_derive_long_secret_access_key_test_value,
    "q+jcrXGc+0zWN6uzclKVhvMmUsIfRPa4rlRandom");

static int s_credentials_derive_ecc_key_long_access_key(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_auth_library_init(allocator);

    struct aws_credentials *creds = aws_credentials_new_from_string(
        allocator,
        s_ecc_derive_long_access_key_id_test_value,
        s_ecc_derive_long_secret_access_key_test_value,
        NULL,
        UINT64_MAX);

    struct aws_ecc_key_pair *derived_key = aws_ecc_key_pair_new_ecdsa_p256_key_from_aws_credentials(allocator, creds);
    ASSERT_TRUE(derived_key != NULL);

    aws_ecc_key_pair_release(derived_key);
    aws_credentials_release(creds);

    aws_auth_library_clean_up();

    return 0;
}

AWS_TEST_CASE(credentials_derive_ecc_key_long_access_key, s_credentials_derive_ecc_key_long_access_key);
