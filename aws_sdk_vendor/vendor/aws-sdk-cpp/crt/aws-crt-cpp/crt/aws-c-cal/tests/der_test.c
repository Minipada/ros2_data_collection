/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/cal/cal.h>
#include <aws/cal/private/der.h>

#include <aws/testing/aws_test_harness.h>

#include "test_case_helper.h"

/* clang-format off */
/* note that this int is unsigned, with the high bit set, so needs to be encoded specially */
static uint8_t s_bigint[] = {
        0x8f, 0xe2, 0x41, 0x2a, 0x08, 0xe8, 0x51, 0xa8, 0x8c, 0xb3, 0xe8, 0x53, 0xe7, 0xd5, 0x49, 0x50, 0xb3, 0x27, 0x8a,
        0x2b, 0xcb, 0xea, 0xb5, 0x42, 0x73, 0xea, 0x02, 0x57, 0xcc, 0x65, 0x33, 0xee, 0x88, 0x20, 0x61, 0xa1, 0x17, 0x56,
        0xc1, 0x24, 0x18, 0xe3, 0xa8, 0x08, 0xd3, 0xbe, 0xd9, 0x31, 0xf3, 0x37, 0x0b, 0x94, 0xb8, 0xcc, 0x43, 0x08, 0x0b,
        0x70, 0x24, 0xf7, 0x9c, 0xb1, 0x8d, 0x5d, 0xd6, 0x6d, 0x82, 0xd0, 0x54, 0x09, 0x84, 0xf8, 0x9f, 0x97, 0x01, 0x75,
        0x05, 0x9c, 0x89, 0xd4, 0xd5, 0xc9, 0x1e, 0xc9, 0x13, 0xd7, 0x2a, 0x6b, 0x30, 0x91, 0x19, 0xd6, 0xd4, 0x42, 0xe0,
        0xc4, 0x9d, 0x7c, 0x92, 0x71, 0xe1, 0xb2, 0x2f, 0x5c, 0x8d, 0xee, 0xf0, 0xf1, 0x17, 0x1e, 0xd2, 0x5f, 0x31, 0x5b,
        0xb1, 0x9c, 0xbc, 0x20, 0x55, 0xbf, 0x3a, 0x37, 0x42, 0x45, 0x75, 0xdc, 0x90, 0x65,
};

static uint8_t s_bigint_zero[] = { 0x00 };

static uint8_t s_encoded_bigint[] = {
        0x02 /* INTEGER */,
        0x81 /* 1 byte length */,
        0x81 /* 0x81 bytes */,
        0x00 /* unsigned */,
        0x8f, 0xe2, 0x41, 0x2a, 0x08, 0xe8, 0x51, 0xa8, 0x8c, 0xb3, 0xe8, 0x53, 0xe7, 0xd5, 0x49, 0x50, 0xb3, 0x27, 0x8a,
        0x2b, 0xcb, 0xea, 0xb5, 0x42, 0x73, 0xea, 0x02, 0x57, 0xcc, 0x65, 0x33, 0xee, 0x88, 0x20, 0x61, 0xa1, 0x17, 0x56,
        0xc1, 0x24, 0x18, 0xe3, 0xa8, 0x08, 0xd3, 0xbe, 0xd9, 0x31, 0xf3, 0x37, 0x0b, 0x94, 0xb8, 0xcc, 0x43, 0x08, 0x0b,
        0x70, 0x24, 0xf7, 0x9c, 0xb1, 0x8d, 0x5d, 0xd6, 0x6d, 0x82, 0xd0, 0x54, 0x09, 0x84, 0xf8, 0x9f, 0x97, 0x01, 0x75,
        0x05, 0x9c, 0x89, 0xd4, 0xd5, 0xc9, 0x1e, 0xc9, 0x13, 0xd7, 0x2a, 0x6b, 0x30, 0x91, 0x19, 0xd6, 0xd4, 0x42, 0xe0,
        0xc4, 0x9d, 0x7c, 0x92, 0x71, 0xe1, 0xb2, 0x2f, 0x5c, 0x8d, 0xee, 0xf0, 0xf1, 0x17, 0x1e, 0xd2, 0x5f, 0x31, 0x5b,
        0xb1, 0x9c, 0xbc, 0x20, 0x55, 0xbf, 0x3a, 0x37, 0x42, 0x45, 0x75, 0xdc, 0x90, 0x65,
};

static uint8_t s_encoded_bigint_zero[] = {
        0x02 /* INTEGER */,
        0x01 /* 1 byte length */,
        0x00 /* unsigned */,
};

const uint8_t s_encoded_true[] = {0x01, 0x01, 0xff};
const uint8_t s_encoded_false[] = {0x01, 0x01, 0x00};

const uint8_t s_encoded_null[] = {0x05, 0x00};

static uint8_t s_bit_string[] = {
        0x47, 0xeb, 0x99, 0x5a, 0xdf, 0x9e, 0x70, 0x0d,  0xfb, 0xa7, 0x31, 0x32, 0xc1, 0x5f, 0x5c, 0x24,
        0xc2, 0xe0, 0xbf, 0xc6, 0x24, 0xaf, 0x15, 0x66,  0x0e, 0xb8, 0x6a, 0x2e, 0xab, 0x2b, 0xc4, 0x97,
        0x1f, 0xe3, 0xcb, 0xdc, 0x63, 0xa5, 0x25, 0xec,  0xc7, 0xb4, 0x28, 0x61, 0x66, 0x36, 0xa1, 0x31,
        0x1b, 0xbf, 0xdd, 0xd0, 0xfc, 0xbf, 0x17, 0x94,  0x90, 0x1d, 0xe5, 0x5e, 0xc7, 0x11, 0x5e, 0xc9,
        0x55, 0x9f, 0xeb, 0xa3, 0x3e, 0x14, 0xc7, 0x99,  0xa6, 0xcb, 0xba, 0xa1, 0x46, 0x0f, 0x39, 0xd4,
        0x44, 0xc4, 0xc8, 0x4b, 0x76, 0x0e, 0x20, 0x5d,  0x6d, 0xa9, 0x34, 0x9e, 0xd4, 0xd5, 0x87, 0x42,
        0xeb, 0x24, 0x26, 0x51, 0x14, 0x90, 0xb4, 0x0f,  0x06, 0x5e, 0x52, 0x88, 0x32, 0x7a, 0x95, 0x20,
        0xa0, 0xfd, 0xf7, 0xe5, 0x7d, 0x60, 0xdd, 0x72,  0x68, 0x9b, 0xf5, 0x7b, 0x05, 0x8f, 0x6d, 0x1e,
};

static uint8_t s_encoded_bit_string[] = {
        0x03, /* BIT_STRING */
        0x81, /* 1 byte length */
        0x81, /* 0x81 bytes */
        0x00, /* 0 trailing unused bits */
        0x47, 0xeb, 0x99, 0x5a, 0xdf, 0x9e, 0x70, 0x0d,  0xfb, 0xa7, 0x31, 0x32, 0xc1, 0x5f, 0x5c, 0x24,
        0xc2, 0xe0, 0xbf, 0xc6, 0x24, 0xaf, 0x15, 0x66,  0x0e, 0xb8, 0x6a, 0x2e, 0xab, 0x2b, 0xc4, 0x97,
        0x1f, 0xe3, 0xcb, 0xdc, 0x63, 0xa5, 0x25, 0xec,  0xc7, 0xb4, 0x28, 0x61, 0x66, 0x36, 0xa1, 0x31,
        0x1b, 0xbf, 0xdd, 0xd0, 0xfc, 0xbf, 0x17, 0x94,  0x90, 0x1d, 0xe5, 0x5e, 0xc7, 0x11, 0x5e, 0xc9,
        0x55, 0x9f, 0xeb, 0xa3, 0x3e, 0x14, 0xc7, 0x99,  0xa6, 0xcb, 0xba, 0xa1, 0x46, 0x0f, 0x39, 0xd4,
        0x44, 0xc4, 0xc8, 0x4b, 0x76, 0x0e, 0x20, 0x5d,  0x6d, 0xa9, 0x34, 0x9e, 0xd4, 0xd5, 0x87, 0x42,
        0xeb, 0x24, 0x26, 0x51, 0x14, 0x90, 0xb4, 0x0f,  0x06, 0x5e, 0x52, 0x88, 0x32, 0x7a, 0x95, 0x20,
        0xa0, 0xfd, 0xf7, 0xe5, 0x7d, 0x60, 0xdd, 0x72,  0x68, 0x9b, 0xf5, 0x7b, 0x05, 0x8f, 0x6d, 0x1e,
};

static uint8_t s_octet_string[] = {
        0x38, 0x10, 0x60, 0xe2, 0x70, 0x69, 0x91, 0x4a,
        0x8b, 0xb5, 0x22, 0x57, 0x2a, 0x62, 0xef, 0xde,
        0x15, 0x7d, 0x59, 0xd6, 0x4e, 0x20, 0x9a, 0x45,
        0x2b, 0xe3, 0xfd, 0xfc, 0x68, 0xba, 0xaf, 0xbf,
        0x9c, 0x17, 0xb0, 0x8e, 0x6d, 0xc4, 0x29, 0x1e,
        0xe3, 0x21, 0xac, 0xbb, 0x5a, 0x8a, 0xc9, 0x67,
        0x0a, 0xd4, 0x45, 0x93, 0x10, 0xc0, 0x26, 0xeb,
        0x0a, 0x83, 0xc2, 0xb1, 0x40, 0x87, 0x36, 0xf7,
        0xa0, 0x26, 0xda, 0xb9, 0xbb, 0x46, 0x73, 0x88,
        0x7a, 0x67, 0xb9, 0xe6, 0xb3, 0x6f, 0xea, 0x59,
        0x28, 0x8a, 0xd3, 0x92, 0x72, 0xf6, 0x7b, 0x89,
        0xa0, 0xd8, 0x2d, 0x9e, 0x40, 0xeb, 0x1e, 0xbb,
        0x6e, 0xae, 0xf0, 0x5a, 0xed, 0x16, 0xc9, 0xe3,
        0x27, 0x59, 0x37, 0x8f, 0xf3, 0x4a, 0x98, 0x60,
        0xf8, 0xfb, 0xa7, 0x0a, 0xee, 0x1b, 0x6e, 0x91,
        0x95, 0x96, 0xcf, 0x0d, 0x56, 0xac, 0xab, 0x35,
};

static uint8_t s_encoded_octet_string[] = {
        0x04, /* OCTET_STRING */
        0x81, /* 1 byte length */
        0x80, /* 0x80 bytes */
        0x38, 0x10, 0x60, 0xe2, 0x70, 0x69, 0x91, 0x4a,
        0x8b, 0xb5, 0x22, 0x57, 0x2a, 0x62, 0xef, 0xde,
        0x15, 0x7d, 0x59, 0xd6, 0x4e, 0x20, 0x9a, 0x45,
        0x2b, 0xe3, 0xfd, 0xfc, 0x68, 0xba, 0xaf, 0xbf,
        0x9c, 0x17, 0xb0, 0x8e, 0x6d, 0xc4, 0x29, 0x1e,
        0xe3, 0x21, 0xac, 0xbb, 0x5a, 0x8a, 0xc9, 0x67,
        0x0a, 0xd4, 0x45, 0x93, 0x10, 0xc0, 0x26, 0xeb,
        0x0a, 0x83, 0xc2, 0xb1, 0x40, 0x87, 0x36, 0xf7,
        0xa0, 0x26, 0xda, 0xb9, 0xbb, 0x46, 0x73, 0x88,
        0x7a, 0x67, 0xb9, 0xe6, 0xb3, 0x6f, 0xea, 0x59,
        0x28, 0x8a, 0xd3, 0x92, 0x72, 0xf6, 0x7b, 0x89,
        0xa0, 0xd8, 0x2d, 0x9e, 0x40, 0xeb, 0x1e, 0xbb,
        0x6e, 0xae, 0xf0, 0x5a, 0xed, 0x16, 0xc9, 0xe3,
        0x27, 0x59, 0x37, 0x8f, 0xf3, 0x4a, 0x98, 0x60,
        0xf8, 0xfb, 0xa7, 0x0a, 0xee, 0x1b, 0x6e, 0x91,
        0x95, 0x96, 0xcf, 0x0d, 0x56, 0xac, 0xab, 0x35,
};

/* SEQUENCE [BOOLEAN true, BOOLEAN false] */
static uint8_t s_encoded_sequence[] = {
        0x30, /* SEQUENCE */
        0x06, /* 6 bytes */
        0x01, 0x01, 0xff, /* BOOLEAN true */
        0x01, 0x01, 0x00, /* BOOLEAN false */
};

/* SET [BOOLEAN true, BOOLEAN false] */
static uint8_t s_encoded_set[] = {
        0x31, /* SET */
        0x06, /* 6 bytes */
        0x01, 0x01, 0xff, /* BOOLEAN true */
        0x01, 0x01, 0x00, /* BOOLEAN false */
        0x0a, /* trailing newline */
};

static uint8_t s_encoded_key_pair[] = {
        0x30, 0x74, /* SEQUENCE, 116 bytes */
        0x02, 0x01, 0x01, /* INTEGER, 1 byte, value: 1 */
        0x04, 0x20, /* OCTET_STRING, 32 bytes */
        0x9d, 0x6d, 0x10, 0x36, 0xbe, 0x66, 0x10, 0xeb, 0x8c, 0x66, 0xe6, 0x39, 0xa3, 0x1e, 0x47, 0xbc,
        0x46, 0x6f, 0x46, 0x70, 0x59, 0x36, 0x32, 0x84, 0x46, 0x0c, 0x97, 0xb8, 0xda, 0x00, 0x19, 0xe2,
        0xa0, 0x07, /* context-defined container 0, 7 bytes */
        0x06, 0x05, 0x2b, 0x81, 0x04, 0x00, 0x0a, /* OID, 5 bytes */
        0xa1, 0x44, /* context-defined container 1, 68 bytes */
        0x03, 0x42, /* BIT_STRING, 66 bytes */
        0x00, 0x04, 0xd1, 0xcf, 0x9c, 0x8a, 0xb4, 0x76, 0x58, 0x70, 0xd9, 0x35, 0x1c, 0xdc, 0x88, 0xbb,
        0x43, 0x19, 0x77, 0xe4, 0xde, 0xba, 0xda, 0x81, 0x58, 0x54, 0x92, 0x93, 0x8d, 0x85, 0xce, 0xf9,
        0x04, 0xf3, 0x8e, 0x86, 0x95, 0x46, 0xa3, 0x43, 0xdd, 0x67, 0x8c, 0x8e, 0xb5, 0xf4, 0x33, 0x8e,
        0x95, 0x4a, 0x93, 0x96, 0xcf, 0xe4, 0x8f, 0x32, 0x78, 0x88, 0xe8, 0x5a, 0xde, 0x59, 0x3f, 0x63,
        0xaf, 0xf2,
        0x0a, /* trailing newline */
};
/* clang-format on */

static int s_der_encode_integer(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_der_encoder *encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    struct aws_byte_cursor bigint_cur = aws_byte_cursor_from_array(s_bigint, AWS_ARRAY_SIZE(s_bigint));
    ASSERT_SUCCESS(aws_der_encoder_write_unsigned_integer(encoder, bigint_cur));
    struct aws_byte_cursor encoded;
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));

    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_bigint, AWS_ARRAY_SIZE(s_encoded_bigint), encoded.ptr, encoded.len);
    aws_der_encoder_destroy(encoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_encode_integer, s_der_encode_integer)

static int s_der_encode_integer_zero(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_der_encoder *encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    struct aws_byte_cursor bigint_cur = aws_byte_cursor_from_array(s_bigint_zero, AWS_ARRAY_SIZE(s_bigint_zero));
    ASSERT_SUCCESS(aws_der_encoder_write_unsigned_integer(encoder, bigint_cur));
    struct aws_byte_cursor encoded;
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));

    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_bigint_zero, AWS_ARRAY_SIZE(s_encoded_bigint_zero), encoded.ptr, encoded.len);
    aws_der_encoder_destroy(encoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_encode_integer_zero, s_der_encode_integer_zero)

static int s_der_encode_boolean(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    bool flag = true;
    struct aws_der_encoder *encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    ASSERT_SUCCESS(aws_der_encoder_write_boolean(encoder, flag));
    struct aws_byte_cursor encoded;
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));

    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_true, AWS_ARRAY_SIZE(s_encoded_true), encoded.ptr, encoded.len);
    aws_der_encoder_destroy(encoder);

    flag = false;
    encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    ASSERT_SUCCESS(aws_der_encoder_write_boolean(encoder, flag));
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));
    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_false, AWS_ARRAY_SIZE(s_encoded_false), encoded.ptr, encoded.len);
    aws_der_encoder_destroy(encoder);

    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_encode_boolean, s_der_encode_boolean)

static int s_der_encode_null(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_der_encoder *encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    ASSERT_SUCCESS(aws_der_encoder_write_null(encoder));
    struct aws_byte_cursor encoded;
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));

    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_null, AWS_ARRAY_SIZE(s_encoded_null), encoded.ptr, encoded.len);

    aws_der_encoder_destroy(encoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_encode_null, s_der_encode_null)

static int s_der_encode_bit_string(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_der_encoder *encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    struct aws_byte_cursor bit_string = aws_byte_cursor_from_array(s_bit_string, AWS_ARRAY_SIZE(s_bit_string));
    ASSERT_SUCCESS(aws_der_encoder_write_bit_string(encoder, bit_string));
    struct aws_byte_cursor encoded;
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));

    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_bit_string, AWS_ARRAY_SIZE(s_encoded_bit_string), encoded.ptr, encoded.len);
    aws_der_encoder_destroy(encoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_encode_bit_string, s_der_encode_bit_string)

static int s_der_encode_octet_string(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_der_encoder *encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    struct aws_byte_cursor octet_string = aws_byte_cursor_from_array(s_octet_string, AWS_ARRAY_SIZE(s_octet_string));
    ASSERT_SUCCESS(aws_der_encoder_write_octet_string(encoder, octet_string));
    struct aws_byte_cursor encoded;
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));

    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_octet_string, AWS_ARRAY_SIZE(s_encoded_octet_string), encoded.ptr, encoded.len);
    aws_der_encoder_destroy(encoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_encode_octet_string, s_der_encode_octet_string)

static int s_der_encode_sequence(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_der_encoder *encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    ASSERT_SUCCESS(aws_der_encoder_begin_sequence(encoder));
    ASSERT_SUCCESS(aws_der_encoder_write_boolean(encoder, true));
    ASSERT_SUCCESS(aws_der_encoder_write_boolean(encoder, false));
    ASSERT_SUCCESS(aws_der_encoder_end_sequence(encoder));
    struct aws_byte_cursor encoded;
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));

    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_sequence, AWS_ARRAY_SIZE(s_encoded_sequence), encoded.ptr, encoded.len);
    aws_der_encoder_destroy(encoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_encode_sequence, s_der_encode_sequence)

static int s_der_encode_set(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_der_encoder *encoder = aws_der_encoder_new(allocator, 1024);
    ASSERT_NOT_NULL(encoder);
    ASSERT_SUCCESS(aws_der_encoder_begin_set(encoder));
    ASSERT_SUCCESS(aws_der_encoder_write_boolean(encoder, true));
    ASSERT_SUCCESS(aws_der_encoder_write_boolean(encoder, false));
    ASSERT_SUCCESS(aws_der_encoder_end_set(encoder));
    struct aws_byte_cursor encoded;
    ASSERT_SUCCESS(aws_der_encoder_get_contents(encoder, &encoded));

    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_set, AWS_ARRAY_SIZE(s_encoded_set) - 1, encoded.ptr, encoded.len);
    aws_der_encoder_destroy(encoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_encode_set, s_der_encode_set)

static int s_der_decode_integer(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    const size_t encoded_size = AWS_ARRAY_SIZE(s_encoded_bigint);
    const size_t decoded_size = AWS_ARRAY_SIZE(s_bigint);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_bigint, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);
    ASSERT_TRUE(aws_der_decoder_next(decoder));

    ASSERT_INT_EQUALS(AWS_DER_INTEGER, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(decoded_size, aws_der_decoder_tlv_length(decoder));
    struct aws_byte_cursor decoded;
    ASSERT_SUCCESS(aws_der_decoder_tlv_unsigned_integer(decoder, &decoded));
    ASSERT_BIN_ARRAYS_EQUALS(s_bigint, decoded_size, decoded.ptr, decoded.len);
    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_integer, s_der_decode_integer)

static int s_der_decode_integer_zero(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    const size_t encoded_size = AWS_ARRAY_SIZE(s_encoded_bigint_zero);
    const size_t decoded_size = AWS_ARRAY_SIZE(s_bigint_zero);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_bigint_zero, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);
    ASSERT_TRUE(aws_der_decoder_next(decoder));

    ASSERT_INT_EQUALS(AWS_DER_INTEGER, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(decoded_size, aws_der_decoder_tlv_length(decoder));
    struct aws_byte_cursor decoded;
    ASSERT_SUCCESS(aws_der_decoder_tlv_unsigned_integer(decoder, &decoded));
    ASSERT_BIN_ARRAYS_EQUALS(s_bigint_zero, decoded_size, decoded.ptr, decoded.len);
    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_integer_zero, s_der_decode_integer_zero)

static int s_der_decode_boolean(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    bool flag = false;
    const size_t encoded_size = AWS_ARRAY_SIZE(s_encoded_true);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_true, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);

    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_BOOLEAN, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(1, aws_der_decoder_tlv_length(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_boolean(decoder, &flag));
    ASSERT_TRUE(flag);
    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);

    input = aws_byte_cursor_from_array(s_encoded_false, encoded_size);
    decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_BOOLEAN, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(1, aws_der_decoder_tlv_length(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_boolean(decoder, &flag));
    ASSERT_FALSE(flag);
    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_boolean, s_der_decode_boolean)

static int s_der_decode_null(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_null, AWS_ARRAY_SIZE(s_encoded_null));
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_NULL, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(0, aws_der_decoder_tlv_length(decoder));
    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_null, s_der_decode_null)

static int s_der_decode_bit_string(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    const size_t encoded_size = AWS_ARRAY_SIZE(s_encoded_bit_string);
    const size_t decoded_size = AWS_ARRAY_SIZE(s_bit_string);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_bit_string, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);
    ASSERT_TRUE(aws_der_decoder_next(decoder));

    ASSERT_INT_EQUALS(AWS_DER_BIT_STRING, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(decoded_size, aws_der_decoder_tlv_length(decoder));
    struct aws_byte_cursor decoded;
    ASSERT_SUCCESS(aws_der_decoder_tlv_string(decoder, &decoded));
    ASSERT_BIN_ARRAYS_EQUALS(s_bit_string, decoded_size, decoded.ptr, decoded.len);
    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_bit_string, s_der_decode_bit_string)

static int s_der_decode_octet_string(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    const size_t encoded_size = AWS_ARRAY_SIZE(s_encoded_octet_string);
    const size_t decoded_size = AWS_ARRAY_SIZE(s_bit_string);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_octet_string, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);
    ASSERT_TRUE(aws_der_decoder_next(decoder));

    ASSERT_INT_EQUALS(AWS_DER_OCTET_STRING, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(decoded_size, aws_der_decoder_tlv_length(decoder));
    struct aws_byte_cursor decoded;
    ASSERT_SUCCESS(aws_der_decoder_tlv_string(decoder, &decoded));
    ASSERT_BIN_ARRAYS_EQUALS(s_octet_string, decoded_size, decoded.ptr, decoded.len);
    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_octet_string, s_der_decode_octet_string)

static int s_der_decode_sequence(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    const size_t encoded_size = AWS_ARRAY_SIZE(s_encoded_sequence);
    const size_t decoded_size = AWS_ARRAY_SIZE(s_encoded_true) + AWS_ARRAY_SIZE(s_encoded_false);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_sequence, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);

    /* Verify SEQUENCE */
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_SEQUENCE, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(decoded_size, aws_der_decoder_tlv_length(decoder));
    ASSERT_INT_EQUALS(2, aws_der_decoder_tlv_count(decoder));

    /* Verify true, then false */
    bool decoded_flag = false;
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_BOOLEAN, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(1, aws_der_decoder_tlv_length(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_boolean(decoder, &decoded_flag));
    ASSERT_TRUE(decoded_flag);

    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_BOOLEAN, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(1, aws_der_decoder_tlv_length(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_boolean(decoder, &decoded_flag));
    ASSERT_FALSE(decoded_flag);

    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_sequence, s_der_decode_sequence)

static int s_der_decode_set(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    const size_t encoded_size = AWS_ARRAY_SIZE(s_encoded_set);
    const size_t decoded_size = AWS_ARRAY_SIZE(s_encoded_true) + AWS_ARRAY_SIZE(s_encoded_false);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_set, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);

    /* Verify SET */
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_SET, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(decoded_size, aws_der_decoder_tlv_length(decoder));
    ASSERT_INT_EQUALS(2, aws_der_decoder_tlv_count(decoder));

    /* Verify true, then false */
    bool decoded_flag = false;
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_BOOLEAN, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(1, aws_der_decoder_tlv_length(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_boolean(decoder, &decoded_flag));
    ASSERT_TRUE(decoded_flag);

    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_BOOLEAN, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(1, aws_der_decoder_tlv_length(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_boolean(decoder, &decoded_flag));
    ASSERT_FALSE(decoded_flag);

    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_set, s_der_decode_set)

static int s_der_decode_key_pair(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    const size_t encoded_size = AWS_ARRAY_SIZE(s_encoded_key_pair);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(s_encoded_key_pair, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);

    /* SEQUENCE */
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_SEQUENCE, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(4, aws_der_decoder_tlv_count(decoder));

    /* INTEGER 1 */
    struct aws_byte_cursor integer;
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_INTEGER, aws_der_decoder_tlv_type(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_unsigned_integer(decoder, &integer));
    ASSERT_BIN_ARRAYS_EQUALS("\x01", 1, integer.ptr, integer.len);

    /* 32 byte private key */
    struct aws_byte_cursor private_key;
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_OCTET_STRING, aws_der_decoder_tlv_type(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_string(decoder, &private_key));
    ASSERT_INT_EQUALS(32, private_key.len);

    /* container */
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_TRUE(aws_der_decoder_tlv_type(decoder) & (AWS_DER_CLASS_CONTEXT | AWS_DER_FORM_CONSTRUCTED));
    ASSERT_INT_EQUALS(7, aws_der_decoder_tlv_length(decoder));
    ASSERT_INT_EQUALS(1, aws_der_decoder_tlv_count(decoder));

    /* 5 byte OID */
    struct aws_byte_cursor oid;
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_OBJECT_IDENTIFIER, aws_der_decoder_tlv_type(decoder));
    ASSERT_INT_EQUALS(5, aws_der_decoder_tlv_length(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_blob(decoder, &oid));
    ASSERT_BIN_ARRAYS_EQUALS("\x2b\x81\x04\x00\x0a", 5, oid.ptr, oid.len);

    /* container */
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_TRUE(aws_der_decoder_tlv_type(decoder) & (AWS_DER_CLASS_CONTEXT | AWS_DER_FORM_CONSTRUCTED));
    ASSERT_INT_EQUALS(68, aws_der_decoder_tlv_length(decoder));
    ASSERT_INT_EQUALS(1, aws_der_decoder_tlv_count(decoder));

    /* 64 byte public key */
    struct aws_byte_cursor public_key;
    ASSERT_TRUE(aws_der_decoder_next(decoder));
    ASSERT_INT_EQUALS(AWS_DER_BIT_STRING, aws_der_decoder_tlv_type(decoder));
    ASSERT_SUCCESS(aws_der_decoder_tlv_string(decoder, &public_key));
    ASSERT_INT_EQUALS(65, public_key.len);

    ASSERT_FALSE(aws_der_decoder_next(decoder));
    aws_der_decoder_destroy(decoder);
    aws_cal_library_clean_up();
    return 0;
}

AWS_TEST_CASE(der_decode_key_pair, s_der_decode_key_pair)

static int s_der_decode_negative_int(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t negative_der[] = {0x02 /*int*/, 0x01 /*len 1*/, 0xfd /*-3*/};

    const size_t encoded_size = AWS_ARRAY_SIZE(negative_der);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(negative_der, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NULL(decoder);

    ASSERT_INT_EQUALS(AWS_ERROR_CAL_DER_UNSUPPORTED_NEGATIVE_INT, aws_last_error());
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();
    return 0;
}
AWS_TEST_CASE(der_decode_negative_int, s_der_decode_negative_int)

static int s_der_decode_positive_int(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t negative_der[] = {0x02 /*int*/, 0x02 /*len 2*/, 0x00, 0xfd /*253*/};

    const size_t encoded_size = AWS_ARRAY_SIZE(negative_der);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(negative_der, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);

    ASSERT_TRUE(aws_der_decoder_next(decoder));
    struct aws_byte_cursor cur;
    ASSERT_SUCCESS(aws_der_decoder_tlv_unsigned_integer(decoder, &cur));
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();
    return 0;
}
AWS_TEST_CASE(der_decode_positive_int, s_der_decode_positive_int)

static int s_der_decode_zero_int(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t negative_der[] = {0x02 /*int*/, 0x01 /*len 2*/, 0x00 /*0*/};

    const size_t encoded_size = AWS_ARRAY_SIZE(negative_der);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(negative_der, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NOT_NULL(decoder);

    ASSERT_TRUE(aws_der_decoder_next(decoder));
    struct aws_byte_cursor cur;
    ASSERT_SUCCESS(aws_der_decoder_tlv_unsigned_integer(decoder, &cur));
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();
    return 0;
}
AWS_TEST_CASE(der_decode_zero_int, s_der_decode_zero_int)

static int s_der_decode_bad_length(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t negative_der[] = {0x02 /*int*/, 0x09 /*len 9*/, 0x00 /*0*/};

    const size_t encoded_size = AWS_ARRAY_SIZE(negative_der);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(negative_der, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NULL(decoder);

    ASSERT_INT_EQUALS(AWS_ERROR_CAL_MALFORMED_ASN1_ENCOUNTERED, aws_last_error());
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();
    return 0;
}
AWS_TEST_CASE(der_decode_bad_length, s_der_decode_bad_length)

static int s_der_decode_zero_length_int(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t negative_der[] = {0x02 /*int*/, 0x00 /*len 9*/, 0x00 /*0*/};

    const size_t encoded_size = AWS_ARRAY_SIZE(negative_der);
    struct aws_byte_cursor input = aws_byte_cursor_from_array(negative_der, encoded_size);
    struct aws_der_decoder *decoder = aws_der_decoder_new(allocator, input);
    ASSERT_NULL(decoder);

    ASSERT_INT_EQUALS(AWS_ERROR_CAL_MALFORMED_ASN1_ENCOUNTERED, aws_last_error());
    aws_der_decoder_destroy(decoder);

    aws_cal_library_clean_up();
    return 0;
}
AWS_TEST_CASE(der_decode_zero_length_int, s_der_decode_zero_length_int)
