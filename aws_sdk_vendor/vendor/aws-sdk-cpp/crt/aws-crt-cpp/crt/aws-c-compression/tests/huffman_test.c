/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/compression/private/huffman_testing.h>
#include <aws/testing/aws_test_harness.h>

#include <aws/compression/huffman.h>

/* Exported by generated file */
struct aws_huffman_symbol_coder *test_get_coder(void);

static struct huffman_test_code_point s_code_points[] = {
#include "test_huffman_static_table.def"
};
enum { NUM_CODE_POINTS = sizeof(s_code_points) / sizeof(s_code_points[0]) };

/* Useful data for testing */
static const char s_url_string[] = "www.example.com";
enum { URL_STRING_LEN = sizeof(s_url_string) - 1 };

static uint8_t s_encoded_url[] = {0x9e, 0x79, 0xeb, 0x9b, 0x04, 0xb3, 0x5a, 0x94, 0xd5, 0xe0, 0x4c, 0xdf};
enum { ENCODED_URL_LEN = sizeof(s_encoded_url) };

static const char s_all_codes[] = " !\"#$%&'()*+,-./"
                                  "0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ["
                                  "\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
enum { ALL_CODES_LEN = sizeof(s_all_codes) - 1 };
static uint8_t s_encoded_codes[] = {
    0x26, 0x9b, 0xa7, 0x69, 0xfa, 0x86, 0xa3, 0xa9, 0x56, 0xd4, 0xf5, 0x4d, 0x57, 0x56, 0xb9, 0xc4, 0x57, 0xd5,
    0xf5, 0x8d, 0x67, 0x5a, 0xd6, 0xf5, 0xcd, 0x77, 0x5e, 0xd7, 0xf6, 0x0d, 0x87, 0x62, 0xd8, 0xf6, 0x4d, 0x97,
    0x66, 0xba, 0xd9, 0xf6, 0x8b, 0xbc, 0x4e, 0x2b, 0x17, 0x8c, 0xc6, 0xe3, 0xaf, 0x36, 0x9d, 0xab, 0x1f, 0x90,
    0xda, 0xf6, 0xcc, 0x8e, 0xdb, 0xb7, 0x6d, 0xf7, 0xbb, 0x86, 0x4a, 0xfb, 0x71, 0xc9, 0xee, 0x5b, 0x9e, 0xe9,
    0xba, 0xee, 0xdb, 0xbe, 0xf0, 0x5b, 0x10, 0x42, 0x68, 0xac, 0xc6, 0x7b, 0xf9, 0x25, 0x99, 0x09, 0xb5, 0x94,
    0x52, 0xd8, 0xdc, 0x09, 0xf0, 0x68, 0xde, 0x77, 0xad, 0xef, 0x7c, 0xdf, 0x7f};
enum { ENCODED_CODES_LEN = sizeof(s_encoded_codes) };

static const size_t s_step_sizes[] = {1, 2, 4, 8, 16, 32, 64, 128};
enum { NUM_STEP_SIZES = sizeof(s_step_sizes) / sizeof(s_step_sizes[0]) };

AWS_TEST_CASE(huffman_symbol_encoder, test_huffman_symbol_encoder)
static int test_huffman_symbol_encoder(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding each character */

    struct aws_huffman_symbol_coder *coder = test_get_coder();

    for (size_t i = 0; i < NUM_CODE_POINTS; ++i) {
        struct huffman_test_code_point *value = &s_code_points[i];

        struct aws_huffman_code code = coder->encode(value->symbol, NULL);

        ASSERT_UINT_EQUALS(value->code.pattern, code.pattern);
        ASSERT_UINT_EQUALS(value->code.num_bits, code.num_bits);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_encoder, test_huffman_encoder)
static int test_huffman_encoder(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding a short url */

    uint8_t output_buffer[ENCODED_URL_LEN + 1];
    AWS_ZERO_ARRAY(output_buffer);
    struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output_buffer, ENCODED_URL_LEN);

    struct aws_huffman_symbol_coder *coder = test_get_coder();
    struct aws_huffman_encoder encoder;
    aws_huffman_encoder_init(&encoder, coder);

    struct aws_byte_cursor to_encode = aws_byte_cursor_from_array(s_url_string, URL_STRING_LEN);
    const size_t encoded_length = aws_huffman_get_encoded_length(&encoder, to_encode);
    ASSERT_UINT_EQUALS(ENCODED_URL_LEN, encoded_length);
    int result = aws_huffman_encode(&encoder, &to_encode, &output_buf);
    ASSERT_SUCCESS(result);

    ASSERT_UINT_EQUALS(ENCODED_URL_LEN, output_buf.len);
    ASSERT_UINT_EQUALS(0, output_buffer[ENCODED_URL_LEN]);
    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_url, ENCODED_URL_LEN, output_buf.buffer, output_buf.len);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_encoder_all_code_points, test_huffman_encoder_all_code_points)
static int test_huffman_encoder_all_code_points(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding a sequence of all character values expressable as
     * characters */

    uint8_t output_buffer[ENCODED_CODES_LEN + 1];
    AWS_ZERO_ARRAY(output_buffer);
    struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output_buffer, ENCODED_CODES_LEN);

    struct aws_huffman_symbol_coder *coder = test_get_coder();
    struct aws_huffman_encoder encoder;
    aws_huffman_encoder_init(&encoder, coder);

    struct aws_byte_cursor to_encode = aws_byte_cursor_from_array(s_all_codes, ALL_CODES_LEN);
    const size_t encoded_length = aws_huffman_get_encoded_length(&encoder, to_encode);
    ASSERT_UINT_EQUALS(ENCODED_CODES_LEN, encoded_length);
    int result = aws_huffman_encode(&encoder, &to_encode, &output_buf);
    ASSERT_SUCCESS(result);

    ASSERT_UINT_EQUALS(ENCODED_CODES_LEN, output_buf.len);
    ASSERT_UINT_EQUALS(0, output_buffer[ENCODED_CODES_LEN]);
    ASSERT_BIN_ARRAYS_EQUALS(s_encoded_codes, ENCODED_CODES_LEN, output_buf.buffer, output_buf.len);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_encoder_partial_output, test_huffman_encoder_partial_output)
static int test_huffman_encoder_partial_output(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding when the output buffer size is limited */

    struct aws_huffman_encoder encoder;
    aws_huffman_encoder_init(&encoder, test_get_coder());

    uint8_t output_buffer[ENCODED_CODES_LEN];

    for (size_t i = 0; i < NUM_STEP_SIZES; ++i) {
        const size_t step_size = s_step_sizes[i];

        aws_huffman_encoder_reset(&encoder);

        struct aws_byte_cursor to_encode = aws_byte_cursor_from_array(s_all_codes, ALL_CODES_LEN);
        struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output_buffer, (size_t)-1);
        output_buf.capacity = 0;
        AWS_ZERO_ARRAY(output_buffer);

        do {
            output_buf.capacity += step_size;
            if (output_buf.capacity > ENCODED_CODES_LEN) {
                output_buf.capacity = ENCODED_CODES_LEN;
            }

            const size_t previous_output_len = output_buf.len;

            int result = aws_huffman_encode(&encoder, &to_encode, &output_buf);

            ASSERT_TRUE(output_buf.len > previous_output_len);
            ASSERT_BIN_ARRAYS_EQUALS(s_encoded_codes, output_buf.len, output_buf.buffer, output_buf.len);

            if (output_buf.len == ENCODED_CODES_LEN) {
                ASSERT_SUCCESS(result);
            } else {
                ASSERT_UINT_EQUALS(AWS_ERROR_SHORT_BUFFER, aws_last_error());
                aws_reset_error();
            }
        } while (output_buf.len < ENCODED_CODES_LEN);

        ASSERT_UINT_EQUALS(ENCODED_CODES_LEN, output_buf.len);

        ASSERT_BIN_ARRAYS_EQUALS(s_encoded_codes, ENCODED_CODES_LEN, output_buf.buffer, output_buf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_encoder_exact_output, test_huffman_encoder_exact_output)
static int test_huffman_encoder_exact_output(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding when the output buffer size is exactly the necessary size */
    struct aws_huffman_encoder encoder;
    aws_huffman_encoder_init(&encoder, test_get_coder());

    uint8_t output_buffer[2];
    struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output_buffer, 2);

    /* Encode a character that uses 8 bits into a 1 byte buffer */
    struct aws_byte_cursor to_encode = aws_byte_cursor_from_array("?", 1);
    uint8_t expected_1byte[] = {0xba};
    output_buf.capacity = 1;
    ASSERT_SUCCESS(aws_huffman_encode(&encoder, &to_encode, &output_buf));
    ASSERT_BIN_ARRAYS_EQUALS(expected_1byte, 1, output_buf.buffer, output_buf.len);

    /* Encode 2 characters that sum to 16 bits, into a 2 byte buffer
     * y: 101000
     * z: 1101111001
     * combined: 1010001101111001 == 0xa379 */
    to_encode = aws_byte_cursor_from_array("yz", 2);
    uint8_t expected_2byte[] = {0xa3, 0x79};
    output_buf.capacity = 2;
    aws_byte_buf_reset(&output_buf, true /*zero*/);
    ASSERT_SUCCESS(aws_huffman_encode(&encoder, &to_encode, &output_buf));
    ASSERT_BIN_ARRAYS_EQUALS(expected_2byte, 2, output_buf.buffer, output_buf.len);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_symbol_decoder, test_huffman_symbol_decoder)
static int test_huffman_symbol_decoder(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test decoding each character */

    struct aws_huffman_symbol_coder *coder = test_get_coder();

    for (size_t i = 0; i < NUM_CODE_POINTS; ++i) {
        struct huffman_test_code_point *value = &s_code_points[i];

        uint32_t bit_pattern = value->code.pattern << (32 - value->code.num_bits);

        uint8_t out;
        size_t bits_read = coder->decode(bit_pattern, &out, NULL);

        ASSERT_UINT_EQUALS(value->symbol, out);
        ASSERT_UINT_EQUALS(value->code.num_bits, bits_read);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_decoder, test_huffman_decoder)
static int test_huffman_decoder(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test decoding a short url */

    char output_buffer[URL_STRING_LEN + 1];
    AWS_ZERO_ARRAY(output_buffer);
    struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output_buffer, URL_STRING_LEN);

    struct aws_huffman_symbol_coder *coder = test_get_coder();
    struct aws_huffman_decoder decoder;
    aws_huffman_decoder_init(&decoder, coder);

    struct aws_byte_cursor to_decode = aws_byte_cursor_from_array(s_encoded_url, ENCODED_URL_LEN);
    int result = aws_huffman_decode(&decoder, &to_decode, &output_buf);

    ASSERT_SUCCESS(result);
    ASSERT_UINT_EQUALS(URL_STRING_LEN, output_buf.len);
    ASSERT_UINT_EQUALS(0, to_decode.len);
    ASSERT_UINT_EQUALS(output_buffer[URL_STRING_LEN], 0);
    ASSERT_BIN_ARRAYS_EQUALS(s_url_string, URL_STRING_LEN, output_buf.buffer, output_buf.len);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_decoder_all_code_points, test_huffman_decoder_all_code_points)
static int test_huffman_decoder_all_code_points(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test decoding a sequence of all character values expressable as
     * characters */

    char output_buffer[ALL_CODES_LEN + 1];
    AWS_ZERO_ARRAY(output_buffer);
    struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output_buffer, ALL_CODES_LEN);

    struct aws_huffman_symbol_coder *coder = test_get_coder();
    struct aws_huffman_decoder decoder;
    aws_huffman_decoder_init(&decoder, coder);

    struct aws_byte_cursor to_decode = aws_byte_cursor_from_array(s_encoded_codes, ENCODED_CODES_LEN);
    int result = aws_huffman_decode(&decoder, &to_decode, &output_buf);

    ASSERT_SUCCESS(result);
    ASSERT_UINT_EQUALS(ALL_CODES_LEN, output_buf.len);
    ASSERT_UINT_EQUALS(0, to_decode.len);
    ASSERT_UINT_EQUALS(output_buffer[ALL_CODES_LEN], 0);
    ASSERT_BIN_ARRAYS_EQUALS(s_all_codes, ALL_CODES_LEN, output_buf.buffer, output_buf.len);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_decoder_partial_input, test_huffman_decoder_partial_input)
static int test_huffman_decoder_partial_input(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test decoding a buffer in chunks */

    struct aws_huffman_decoder decoder;
    aws_huffman_decoder_init(&decoder, test_get_coder());

    char output_buffer[150];

    for (size_t i = 0; i < NUM_STEP_SIZES; ++i) {
        const size_t step_size = s_step_sizes[i];

        aws_huffman_decoder_reset(&decoder);

        struct aws_byte_cursor to_decode = aws_byte_cursor_from_array(s_encoded_codes, ENCODED_CODES_LEN);
        struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output_buffer, ALL_CODES_LEN);
        AWS_ZERO_ARRAY(output_buffer);

        do {
            const size_t chunk_size = step_size < to_decode.len ? step_size : to_decode.len;
            struct aws_byte_cursor to_decode_chunk = aws_byte_cursor_advance(&to_decode, chunk_size);

            int result = aws_huffman_decode(&decoder, &to_decode_chunk, &output_buf);

            ASSERT_UINT_EQUALS(0, to_decode_chunk.len);
            ASSERT_BIN_ARRAYS_EQUALS(s_all_codes, output_buf.len, output_buf.buffer, output_buf.len);

            if (output_buf.len == ALL_CODES_LEN) {
                ASSERT_SUCCESS(result);
            }
        } while (output_buf.len < ALL_CODES_LEN);

        ASSERT_UINT_EQUALS(ALL_CODES_LEN, output_buf.len);
        ASSERT_BIN_ARRAYS_EQUALS(s_all_codes, ALL_CODES_LEN, output_buf.buffer, output_buf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_decoder_partial_output, test_huffman_decoder_partial_output)
static int test_huffman_decoder_partial_output(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test decoding when the output buffer size is limited */

    struct aws_huffman_decoder decoder;
    aws_huffman_decoder_init(&decoder, test_get_coder());

    char output_buffer[150];

    for (size_t i = 0; i < NUM_STEP_SIZES; ++i) {
        const size_t step_size = s_step_sizes[i];

        aws_huffman_decoder_reset(&decoder);

        struct aws_byte_cursor to_decode = aws_byte_cursor_from_array(s_encoded_codes, ENCODED_CODES_LEN);
        struct aws_byte_buf output_buf = aws_byte_buf_from_empty_array(output_buffer, (size_t)-1);
        output_buf.capacity = 0; /* Can't set above because it sets buffer to 0 */
        AWS_ZERO_ARRAY(output_buffer);

        do {
            output_buf.capacity += step_size;
            if (output_buf.capacity > ALL_CODES_LEN) {
                output_buf.capacity = ALL_CODES_LEN;
            }

            const size_t previous_output_size = output_buf.len;

            int result = aws_huffman_decode(&decoder, &to_decode, &output_buf);

            ASSERT_TRUE(output_buf.len > previous_output_size);
            ASSERT_BIN_ARRAYS_EQUALS(s_all_codes, output_buf.len, output_buf.buffer, output_buf.len);

            if (output_buf.len == ALL_CODES_LEN) {
                ASSERT_SUCCESS(result);
            } else {
                ASSERT_UINT_EQUALS(AWS_ERROR_SHORT_BUFFER, aws_last_error());
                aws_reset_error();
            }
        } while (output_buf.len < ALL_CODES_LEN);

        ASSERT_UINT_EQUALS(ALL_CODES_LEN, output_buf.len);
        ASSERT_BIN_ARRAYS_EQUALS(s_all_codes, ALL_CODES_LEN, output_buf.buffer, output_buf.len);
    }

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_decoder_allow_growth, test_huffman_decoder_allow_growth)
static int test_huffman_decoder_allow_growth(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    /* Test that decoder will grow output buffer if allow-growth is set */

    struct aws_huffman_decoder decoder;
    aws_huffman_decoder_init(&decoder, test_get_coder());
    aws_huffman_decoder_allow_growth(&decoder, true);

    struct aws_byte_buf output_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&output_buf, allocator, 1 /* way too small */));

    struct aws_byte_cursor to_decode = aws_byte_cursor_from_array(s_encoded_url, ENCODED_URL_LEN);
    ASSERT_SUCCESS(aws_huffman_decode(&decoder, &to_decode, &output_buf));

    ASSERT_UINT_EQUALS(0, to_decode.len);
    ASSERT_BIN_ARRAYS_EQUALS(s_url_string, URL_STRING_LEN, output_buf.buffer, output_buf.len);

    aws_byte_buf_clean_up(&output_buf);
    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_transitive, test_huffman_transitive)
static int test_huffman_transitive(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding a short url and immediately decoding it */

    const char *error_message = NULL;
    int result =
        huffman_test_transitive(test_get_coder(), s_url_string, URL_STRING_LEN, ENCODED_URL_LEN, &error_message);
    ASSERT_SUCCESS(result, error_message);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_transitive_even_bytes, test_huffman_transitive_even_bytes)
static int test_huffman_transitive_even_bytes(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding a string that encodes to a multiple of 8 bits */

    const char *error_message = NULL;
    int result = huffman_test_transitive(test_get_coder(), "cdfh", 4, 3, &error_message);
    ASSERT_SUCCESS(result, error_message);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_transitive_all_code_points, test_huffman_transitive_all_code_points)
static int test_huffman_transitive_all_code_points(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding a sequence of all character values expressable as
     * characters and immediately decoding it */

    const char *error_message = NULL;
    int result =
        huffman_test_transitive(test_get_coder(), s_all_codes, ALL_CODES_LEN, ENCODED_CODES_LEN, &error_message);
    ASSERT_SUCCESS(result, error_message);

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(huffman_transitive_chunked, test_huffman_transitive_chunked)
static int test_huffman_transitive_chunked(struct aws_allocator *allocator, void *ctx) {
    (void)allocator;
    (void)ctx;
    /* Test encoding a sequence of all character values expressable as
     * characters and immediately decoding it */

    for (size_t i = 0; i < NUM_STEP_SIZES; ++i) {
        const size_t step_size = s_step_sizes[i];

        const char *error_message = NULL;
        int result = huffman_test_transitive_chunked(
            test_get_coder(), s_all_codes, ALL_CODES_LEN, ENCODED_CODES_LEN, step_size, &error_message);
        ASSERT_SUCCESS(result, error_message);
    }

    return AWS_OP_SUCCESS;
}
