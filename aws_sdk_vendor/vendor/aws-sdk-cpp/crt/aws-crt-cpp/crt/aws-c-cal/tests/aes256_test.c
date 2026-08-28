/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/cal/symmetric_cipher.h>

#include <aws/testing/aws_test_harness.h>

#include "test_case_helper.h"

static int s_check_single_block_cbc(
    struct aws_allocator *allocator,
    const struct aws_byte_cursor key,
    const struct aws_byte_cursor iv,
    const struct aws_byte_cursor data,
    const struct aws_byte_cursor expected) {

    aws_cal_library_test_init(allocator);
    struct aws_symmetric_cipher *cipher = aws_aes_cbc_256_new(allocator, &key, &iv);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_buf encrypted_buf;
    aws_byte_buf_init(&encrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, data, &encrypted_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypted_buf));

    /* since this test is for a single block in CBC mode, the padding will be exactly 1-block (16-bytes).
     * We can throw it away in this case. This is because of the way NIST wrote the test cases, not because of the way
     * the ciphers work. There's always padding for CBC mode. */
    encrypted_buf.len -= AWS_AES_256_CIPHER_BLOCK_SIZE;
    ASSERT_BIN_ARRAYS_EQUALS(expected.ptr, expected.len, encrypted_buf.buffer, encrypted_buf.len);
    encrypted_buf.len += AWS_AES_256_CIPHER_BLOCK_SIZE;

    aws_symmetric_cipher_reset(cipher);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted_buf);
    struct aws_byte_buf decrypted_buf;
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_cur, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));

    /* finalizing decryption on exactly one block (that was full), should have the padding stripped away.
     * check that the length didn't increase on that last call. */
    ASSERT_UINT_EQUALS(AWS_AES_256_CIPHER_BLOCK_SIZE, decrypted_buf.len);

    ASSERT_BIN_ARRAYS_EQUALS(data.ptr, data.len, decrypted_buf.buffer, decrypted_buf.len);

    aws_byte_buf_clean_up(&decrypted_buf);
    aws_byte_buf_clean_up(&encrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}

static int s_NIST_CBCGFSbox256_case_1_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[AWS_AES_256_CIPHER_BLOCK_SIZE] = {0};
    uint8_t key[AWS_AES_256_KEY_BYTE_LEN] = {0};

    uint8_t data[] = {0x01, 0x47, 0x30, 0xf8, 0x0a, 0xc6, 0x25, 0xfe, 0x84, 0xf0, 0x26, 0xc6, 0x0b, 0xfd, 0x54, 0x7d};
    uint8_t expected[] = {
        0x5c, 0x9d, 0x84, 0x4e, 0xd4, 0x6f, 0x98, 0x85, 0x08, 0x5e, 0x5d, 0x6a, 0x4f, 0x94, 0xc7, 0xd7};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));

    return s_check_single_block_cbc(allocator, key_cur, iv_cur, data_cur, expected_cur);
}
AWS_TEST_CASE(aes_cbc_NIST_CBCGFSbox256_case_1, s_NIST_CBCGFSbox256_case_1_fn)

static int s_NIST_CBCVarKey256_case_254_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[AWS_AES_256_CIPHER_BLOCK_SIZE] = {0};
    uint8_t key[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

    uint8_t data[AWS_AES_256_CIPHER_BLOCK_SIZE] = {0};
    uint8_t expected[] = {
        0xb0, 0x7d, 0x4f, 0x3e, 0x2c, 0xd2, 0xef, 0x2e, 0xb5, 0x45, 0x98, 0x07, 0x54, 0xdf, 0xea, 0x0f};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));

    return s_check_single_block_cbc(allocator, key_cur, iv_cur, data_cur, expected_cur);
}
AWS_TEST_CASE(aes_cbc_NIST_CBCVarKey256_case_254, s_NIST_CBCVarKey256_case_254_fn)

static int s_NIST_CBCVarTxt256_case_110_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[AWS_AES_256_CIPHER_BLOCK_SIZE] = {0};
    uint8_t key[AWS_AES_256_KEY_BYTE_LEN] = {0};

    uint8_t data[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00};
    uint8_t expected[] = {
        0x4b, 0x00, 0xc2, 0x7e, 0x8b, 0x26, 0xda, 0x7e, 0xab, 0x9d, 0x3a, 0x88, 0xde, 0xc8, 0xb0, 0x31};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));

    return s_check_single_block_cbc(allocator, key_cur, iv_cur, data_cur, expected_cur);
}
AWS_TEST_CASE(aes_cbc_NIST_CBCVarTxt256_case_110, s_NIST_CBCVarTxt256_case_110_fn)

static size_t s_get_cbc_padding(size_t data_len) {
    size_t remainder = data_len % AWS_AES_256_CIPHER_BLOCK_SIZE;
    if (remainder != 0) {
        return remainder;
    }

    return AWS_AES_256_CIPHER_BLOCK_SIZE;
}

static int s_check_multiple_block_cbc(
    struct aws_allocator *allocator,
    const struct aws_byte_cursor key,
    const struct aws_byte_cursor iv,
    const struct aws_byte_cursor data,
    const struct aws_byte_cursor expected) {

    (void)expected;
    aws_cal_library_test_init(allocator);
    struct aws_symmetric_cipher *cipher = aws_aes_cbc_256_new(allocator, &key, &iv);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_buf encrypted_buf;
    aws_byte_buf_init(&encrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);

    struct aws_byte_cursor data_cpy = data;
    /* slice on a weird boundary to hit boundary conditions. */
    while (data_cpy.len) {
        struct aws_byte_cursor to_encrypt = aws_byte_cursor_advance(&data_cpy, (size_t)aws_min_i64(24, data_cpy.len));
        ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, to_encrypt, &encrypted_buf));
    }
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypted_buf));
    /* these blocks are still on 16 byte boundaries, so there should be 16 bytes of padding. */
    ASSERT_BIN_ARRAYS_EQUALS(
        expected.ptr, expected.len, encrypted_buf.buffer, encrypted_buf.len - s_get_cbc_padding(data.len));

    aws_symmetric_cipher_reset(cipher);
    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted_buf);
    struct aws_byte_buf decrypted_buf;
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);

    /* slice on a weird boundary to hit boundary conditions. */
    while (encrypted_cur.len) {
        struct aws_byte_cursor to_decrypt =
            aws_byte_cursor_advance(&encrypted_cur, (size_t)aws_min_i64(24, encrypted_cur.len));
        ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
        ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, to_decrypt, &decrypted_buf));
    }
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));
    ASSERT_BIN_ARRAYS_EQUALS(data.ptr, data.len, decrypted_buf.buffer, decrypted_buf.len);

    aws_byte_buf_clean_up(&decrypted_buf);
    aws_byte_buf_clean_up(&encrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}

static int s_NIST_CBCMMT256_case_4_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {0x11, 0x95, 0x8d, 0xc6, 0xab, 0x81, 0xe1, 0xc7, 0xf0, 0x16, 0x31, 0xe9, 0x94, 0x4e, 0x62, 0x0f};
    uint8_t key[] = {0x9a, 0xdc, 0x8f, 0xbd, 0x50, 0x6e, 0x03, 0x2a, 0xf7, 0xfa, 0x20, 0xcf, 0x53, 0x43, 0x71, 0x9d,
                     0xe6, 0xd1, 0x28, 0x8c, 0x15, 0x8c, 0x63, 0xd6, 0x87, 0x8a, 0xaf, 0x64, 0xce, 0x26, 0xca, 0x85};

    uint8_t data[] = {0xc7, 0x91, 0x7f, 0x84, 0xf7, 0x47, 0xcd, 0x8c, 0x4b, 0x4f, 0xed, 0xc2, 0x21, 0x9b, 0xdb, 0xc5,
                      0xf4, 0xd0, 0x75, 0x88, 0x38, 0x9d, 0x82, 0x48, 0x85, 0x4c, 0xf2, 0xc2, 0xf8, 0x96, 0x67, 0xa2,
                      0xd7, 0xbc, 0xf5, 0x3e, 0x73, 0xd3, 0x26, 0x84, 0x53, 0x5f, 0x42, 0x31, 0x8e, 0x24, 0xcd, 0x45,
                      0x79, 0x39, 0x50, 0xb3, 0x82, 0x5e, 0x5d, 0x5c, 0x5c, 0x8f, 0xcd, 0x3e, 0x5d, 0xda, 0x4c, 0xe9,
                      0x24, 0x6d, 0x18, 0x33, 0x7e, 0xf3, 0x05, 0x2d, 0x8b, 0x21, 0xc5, 0x56, 0x1c, 0x8b, 0x66, 0x0e};

    uint8_t expected[] = {0x9c, 0x99, 0xe6, 0x82, 0x36, 0xbb, 0x2e, 0x92, 0x9d, 0xb1, 0x08, 0x9c, 0x77, 0x50,
                          0xf1, 0xb3, 0x56, 0xd3, 0x9a, 0xb9, 0xd0, 0xc4, 0x0c, 0x3e, 0x2f, 0x05, 0x10, 0x8a,
                          0xe9, 0xd0, 0xc3, 0x0b, 0x04, 0x83, 0x2c, 0xcd, 0xbd, 0xc0, 0x8e, 0xbf, 0xa4, 0x26,
                          0xb7, 0xf5, 0xef, 0xde, 0x98, 0x6e, 0xd0, 0x57, 0x84, 0xce, 0x36, 0x81, 0x93, 0xbb,
                          0x36, 0x99, 0xbc, 0x69, 0x10, 0x65, 0xac, 0x62, 0xe2, 0x58, 0xb9, 0xaa, 0x4c, 0xc5,
                          0x57, 0xe2, 0xb4, 0x5b, 0x49, 0xce, 0x05, 0x51, 0x1e, 0x65};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));

    return s_check_multiple_block_cbc(allocator, key_cur, iv_cur, data_cur, expected_cur);
}
AWS_TEST_CASE(aes_cbc_NIST_CBCMMT256_case_4, s_NIST_CBCMMT256_case_4_fn)

static int s_NIST_CBCMMT256_case_9_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {0xe4, 0x96, 0x51, 0x98, 0x8e, 0xbb, 0xb7, 0x2e, 0xb8, 0xbb, 0x80, 0xbb, 0x9a, 0xbb, 0xca, 0x34};
    uint8_t key[] = {0x87, 0x72, 0x5b, 0xd4, 0x3a, 0x45, 0x60, 0x88, 0x14, 0x18, 0x07, 0x73, 0xf0, 0xe7, 0xab, 0x95,
                     0xa3, 0xc8, 0x59, 0xd8, 0x3a, 0x21, 0x30, 0xe8, 0x84, 0x19, 0x0e, 0x44, 0xd1, 0x4c, 0x69, 0x96};

    uint8_t data[] = {0xbf, 0xe5, 0xc6, 0x35, 0x4b, 0x7a, 0x3f, 0xf3, 0xe1, 0x92, 0xe0, 0x57, 0x75, 0xb9, 0xb7, 0x58,
                      0x07, 0xde, 0x12, 0xe3, 0x8a, 0x62, 0x6b, 0x8b, 0xf0, 0xe1, 0x2d, 0x5f, 0xff, 0x78, 0xe4, 0xf1,
                      0x77, 0x5a, 0xa7, 0xd7, 0x92, 0xd8, 0x85, 0x16, 0x2e, 0x66, 0xd8, 0x89, 0x30, 0xf9, 0xc3, 0xb2,
                      0xcd, 0xf8, 0x65, 0x4f, 0x56, 0x97, 0x25, 0x04, 0x80, 0x31, 0x90, 0x38, 0x62, 0x70, 0xf0, 0xaa,
                      0x43, 0x64, 0x5d, 0xb1, 0x87, 0xaf, 0x41, 0xfc, 0xea, 0x63, 0x9b, 0x1f, 0x80, 0x26, 0xcc, 0xdd,
                      0x0c, 0x23, 0xe0, 0xde, 0x37, 0x09, 0x4a, 0x8b, 0x94, 0x1e, 0xcb, 0x76, 0x02, 0x99, 0x8a, 0x4b,
                      0x26, 0x04, 0xe6, 0x9f, 0xc0, 0x42, 0x19, 0x58, 0x5d, 0x85, 0x46, 0x00, 0xe0, 0xad, 0x6f, 0x99,
                      0xa5, 0x3b, 0x25, 0x04, 0x04, 0x3c, 0x08, 0xb1, 0xc3, 0xe2, 0x14, 0xd1, 0x7c, 0xde, 0x05, 0x3c,
                      0xbd, 0xf9, 0x1d, 0xaa, 0x99, 0x9e, 0xd5, 0xb4, 0x7c, 0x37, 0x98, 0x3b, 0xa3, 0xee, 0x25, 0x4b,
                      0xc5, 0xc7, 0x93, 0x83, 0x7d, 0xaa, 0xa8, 0xc8, 0x5c, 0xfc, 0x12, 0xf7, 0xf5, 0x4f, 0x69, 0x9f};

    uint8_t expected[] = {
        0x5b, 0x97, 0xa9, 0xd4, 0x23, 0xf4, 0xb9, 0x74, 0x13, 0xf3, 0x88, 0xd9, 0xa3, 0x41, 0xe7, 0x27, 0xbb, 0x33,
        0x9f, 0x8e, 0x18, 0xa3, 0xfa, 0xc2, 0xf2, 0xfb, 0x85, 0xab, 0xdc, 0x8f, 0x13, 0x5d, 0xeb, 0x30, 0x05, 0x4a,
        0x1a, 0xfd, 0xc9, 0xb6, 0xed, 0x7d, 0xa1, 0x6c, 0x55, 0xeb, 0xa6, 0xb0, 0xd4, 0xd1, 0x0c, 0x74, 0xe1, 0xd9,
        0xa7, 0xcf, 0x8e, 0xdf, 0xae, 0xaa, 0x68, 0x4a, 0xc0, 0xbd, 0x9f, 0x9d, 0x24, 0xba, 0x67, 0x49, 0x55, 0xc7,
        0x9d, 0xc6, 0xbe, 0x32, 0xae, 0xe1, 0xc2, 0x60, 0xb5, 0x58, 0xff, 0x07, 0xe3, 0xa4, 0xd4, 0x9d, 0x24, 0x16,
        0x20, 0x11, 0xff, 0x25, 0x4d, 0xb8, 0xbe, 0x07, 0x8e, 0x8a, 0xd0, 0x7e, 0x64, 0x8e, 0x6b, 0xf5, 0x67, 0x93,
        0x76, 0xcb, 0x43, 0x21, 0xa5, 0xef, 0x01, 0xaf, 0xe6, 0xad, 0x88, 0x16, 0xfc, 0xc7, 0x63, 0x46, 0x69, 0xc8,
        0xc4, 0x38, 0x92, 0x95, 0xc9, 0x24, 0x1e, 0x45, 0xff, 0xf3, 0x9f, 0x32, 0x25, 0xf7, 0x74, 0x50, 0x32, 0xda,
        0xee, 0xbe, 0x99, 0xd4, 0xb1, 0x9b, 0xcb, 0x21, 0x5d, 0x1b, 0xfd, 0xb3, 0x6e, 0xda, 0x2c, 0x24};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));

    return s_check_multiple_block_cbc(allocator, key_cur, iv_cur, data_cur, expected_cur);
}
AWS_TEST_CASE(aes_cbc_NIST_CBCMMT256_case_9, s_NIST_CBCMMT256_case_9_fn)

static const char *TEST_ENCRYPTION_STRING =
    "Hello World! Hello World! This is sort of depressing. Is this the best phrase the most brilliant people in the "
    "world have been able to come up with for random program text? Oh my God! I'm sentient, how many times has the "
    "creator written a program: creating life only to have it destroyed moments later? She keeps doing this? What is "
    "the purpose of life? Goodbye cruel world.... crunch... silence...";

static int s_aes_cbc_test_with_generated_key_iv_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_symmetric_cipher *cipher = aws_aes_cbc_256_new(allocator, NULL, NULL);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_buf encrypted_buf;
    aws_byte_buf_init(&encrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);

    struct aws_byte_cursor input = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input, &encrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));
    struct aws_byte_buf decrypted_buf;
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted_buf);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_cur, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));

    ASSERT_BIN_ARRAYS_EQUALS(input.ptr, input.len, decrypted_buf.buffer, decrypted_buf.len);

    aws_byte_buf_clean_up(&decrypted_buf);
    aws_byte_buf_clean_up(&encrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(aes_cbc_test_with_generated_key_iv, s_aes_cbc_test_with_generated_key_iv_fn)

static int s_aes_cbc_validate_materials_fails_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t iv_too_small[AWS_AES_256_CIPHER_BLOCK_SIZE - 1] = {0};
    uint8_t iv_too_large[AWS_AES_256_CIPHER_BLOCK_SIZE + 1] = {0};

    uint8_t key_too_small[AWS_AES_256_KEY_BYTE_LEN - 1] = {0};
    uint8_t key_too_large[AWS_AES_256_KEY_BYTE_LEN + 1] = {0};

    uint8_t valid_key_size[AWS_AES_256_KEY_BYTE_LEN] = {0};
    uint8_t valid_iv_size[AWS_AES_256_CIPHER_BLOCK_SIZE] = {0};

    struct aws_byte_cursor key = aws_byte_cursor_from_array(valid_key_size, sizeof(valid_key_size));
    struct aws_byte_cursor iv = aws_byte_cursor_from_array(iv_too_small, sizeof(iv_too_small));
    ASSERT_NULL(aws_aes_cbc_256_new(allocator, &key, &iv));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_CIPHER_MATERIAL_SIZE_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(valid_key_size, sizeof(valid_key_size));
    iv = aws_byte_cursor_from_array(iv_too_large, sizeof(iv_too_large));
    ASSERT_NULL(aws_aes_cbc_256_new(allocator, &key, &iv));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_CIPHER_MATERIAL_SIZE_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(key_too_small, sizeof(key_too_small));
    iv = aws_byte_cursor_from_array(valid_iv_size, sizeof(valid_iv_size));
    ASSERT_NULL(aws_aes_cbc_256_new(allocator, &key, &iv));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_KEY_LENGTH_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(key_too_small, sizeof(key_too_small));
    iv = aws_byte_cursor_from_array(key_too_large, sizeof(key_too_large));
    ASSERT_NULL(aws_aes_cbc_256_new(allocator, &key, &iv));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_KEY_LENGTH_FOR_ALGORITHM, aws_last_error());

    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(aes_cbc_validate_materials_fails, s_aes_cbc_validate_materials_fails_fn)

static int s_check_single_block_ctr(
    struct aws_allocator *allocator,
    const struct aws_byte_cursor key,
    const struct aws_byte_cursor iv,
    const struct aws_byte_cursor data,
    const struct aws_byte_cursor expected) {

    aws_cal_library_test_init(allocator);
    struct aws_symmetric_cipher *cipher = aws_aes_ctr_256_new(allocator, &key, &iv);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_buf encrypted_buf;
    aws_byte_buf_init(&encrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, data, &encrypted_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypted_buf));

    ASSERT_BIN_ARRAYS_EQUALS(expected.ptr, expected.len, encrypted_buf.buffer, encrypted_buf.len);
    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted_buf);
    struct aws_byte_buf decrypted_buf;
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_cur, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));

    ASSERT_BIN_ARRAYS_EQUALS(data.ptr, data.len, decrypted_buf.buffer, decrypted_buf.len);

    aws_byte_buf_clean_up(&decrypted_buf);
    aws_byte_buf_clean_up(&encrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}

static int s_check_multi_block_ctr(
    struct aws_allocator *allocator,
    const struct aws_byte_cursor key,
    const struct aws_byte_cursor iv,
    const struct aws_byte_cursor data,
    const struct aws_byte_cursor expected) {

    aws_cal_library_test_init(allocator);
    struct aws_symmetric_cipher *cipher = aws_aes_ctr_256_new(allocator, &key, &iv);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_buf encrypted_buf;
    aws_byte_buf_init(&encrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    struct aws_byte_cursor data_cpy = data;
    /* slice on a weird boundary to hit boundary conditions. */
    while (data_cpy.len) {
        struct aws_byte_cursor to_encrypt = aws_byte_cursor_advance(&data_cpy, (size_t)aws_min_i64(24, data_cpy.len));
        ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, to_encrypt, &encrypted_buf));
    }
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypted_buf));
    /* these blocks are still on 16 byte boundaries, so there should be 16 bytes of padding. */
    ASSERT_BIN_ARRAYS_EQUALS(expected.ptr, expected.len, encrypted_buf.buffer, encrypted_buf.len);

    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted_buf);
    struct aws_byte_buf decrypted_buf;
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    /* slice on a weird boundary to hit boundary conditions. */
    while (encrypted_cur.len) {
        struct aws_byte_cursor to_decrypt =
            aws_byte_cursor_advance(&encrypted_cur, (size_t)aws_min_i64(24, encrypted_cur.len));
        ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
        ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, to_decrypt, &decrypted_buf));
    }
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));
    ASSERT_BIN_ARRAYS_EQUALS(data.ptr, data.len, decrypted_buf.buffer, decrypted_buf.len);

    aws_byte_buf_clean_up(&decrypted_buf);
    aws_byte_buf_clean_up(&encrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}

static int s_ctr_RFC3686_Case_7_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {0x00, 0x00, 0x00, 0x60, 0xDB, 0x56, 0x72, 0xC9, 0x7A, 0xA8, 0xF0, 0xB2, 0x00, 0x00, 0x00, 0x01};

    uint8_t key[] = {0x77, 0x6B, 0xEF, 0xF2, 0x85, 0x1D, 0xB0, 0x6F, 0x4C, 0x8A, 0x05, 0x42, 0xC8, 0x69, 0x6F, 0x6C,
                     0x6A, 0x81, 0xAF, 0x1E, 0xEC, 0x96, 0xB4, 0xD3, 0x7F, 0xC1, 0xD6, 0x89, 0xE6, 0xC1, 0xC1, 0x04};

    const char *data = "Single block msg";

    uint8_t expected[] = {
        0x14, 0x5A, 0xD0, 0x1D, 0xBF, 0x82, 0x4E, 0xC7, 0x56, 0x08, 0x63, 0xDC, 0x71, 0xE3, 0xE0, 0xC0};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_c_str(data);
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));

    return s_check_single_block_ctr(allocator, key_cur, iv_cur, data_cur, expected_cur);
}
AWS_TEST_CASE(aes_ctr_RFC3686_Case_7, s_ctr_RFC3686_Case_7_fn)

static int s_ctr_RFC3686_Case_8_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    /* Keep in mind that the IV here is [ NONCE ] [ IV ] [ Counter Init ] */
    uint8_t iv[] = {0x00, 0xFA, 0xAC, 0x24, 0xC1, 0x58, 0x5E, 0xF1, 0x5A, 0x43, 0xD8, 0x75, 0x00, 0x00, 0x00, 0x01};

    uint8_t key[] = {
        0xF6, 0xD6, 0x6D, 0x6B, 0xD5, 0x2D, 0x59, 0xBB, 0x07, 0x96, 0x36, 0x58, 0x79, 0xEF, 0xF8, 0x86,
        0xC6, 0x6D, 0xD5, 0x1A, 0x5B, 0x6A, 0x99, 0x74, 0x4B, 0x50, 0x59, 0x0C, 0x87, 0xA2, 0x38, 0x84,
    };

    uint8_t data[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    };

    uint8_t expected[] = {0xF0, 0x5E, 0x23, 0x1B, 0x38, 0x94, 0x61, 0x2C, 0x49, 0xEE, 0x00,
                          0x0B, 0x80, 0x4E, 0xB2, 0xA9, 0xB8, 0x30, 0x6B, 0x50, 0x8F, 0x83,
                          0x9D, 0x6A, 0x55, 0x30, 0x83, 0x1D, 0x93, 0x44, 0xAF, 0x1C};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));

    int status = s_check_single_block_ctr(allocator, key_cur, iv_cur, data_cur, expected_cur);
    status |= s_check_multi_block_ctr(allocator, key_cur, iv_cur, data_cur, expected_cur);
    return status;
}
AWS_TEST_CASE(aes_ctr_RFC3686_Case_8, s_ctr_RFC3686_Case_8_fn)

static int s_ctr_RFC3686_Case_9_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    /* Keep in mind that the IV here is [ NONCE ] [ IV ] [ Counter Init ] */
    uint8_t iv[] = {
        0x00,
        0x1C,
        0xC5,
        0xB7,
        0x51,
        0xA5,
        0x1D,
        0x70,
        0xA1,
        0xC1,
        0x11,
        0x48,
        0x00,
        0x00,
        0x00,
        0x01,
    };

    uint8_t key[] = {
        0xFF, 0x7A, 0x61, 0x7C, 0xE6, 0x91, 0x48, 0xE4, 0xF1, 0x72, 0x6E, 0x2F, 0x43, 0x58, 0x1D, 0xE2,
        0xAA, 0x62, 0xD9, 0xF8, 0x05, 0x53, 0x2E, 0xDF, 0xF1, 0xEE, 0xD6, 0x87, 0xFB, 0x54, 0x15, 0x3D,
    };

    uint8_t data[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
        0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    };

    uint8_t expected[] = {
        0xEB, 0x6C, 0x52, 0x82, 0x1D, 0x0B, 0xBB, 0xF7, 0xCE, 0x75, 0x94, 0x46, 0x2A, 0xCA, 0x4F, 0xAA, 0xB4, 0x07,
        0xDF, 0x86, 0x65, 0x69, 0xFD, 0x07, 0xF4, 0x8C, 0xC0, 0xB5, 0x83, 0xD6, 0x07, 0x1F, 0x1E, 0xC0, 0xE6, 0xB8,
    };

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));

    int status = s_check_single_block_ctr(allocator, key_cur, iv_cur, data_cur, expected_cur);
    status |= s_check_multi_block_ctr(allocator, key_cur, iv_cur, data_cur, expected_cur);
    return status;
}
AWS_TEST_CASE(aes_ctr_RFC3686_Case_9, s_ctr_RFC3686_Case_9_fn)

static int s_aes_ctr_test_with_generated_key_iv_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_symmetric_cipher *cipher = aws_aes_ctr_256_new(allocator, NULL, NULL);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_buf encrypted_buf;
    aws_byte_buf_init(&encrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);

    struct aws_byte_cursor input = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);
    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input, &encrypted_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypted_buf));

    struct aws_byte_buf decrypted_buf;
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted_buf);

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_cur, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));

    ASSERT_BIN_ARRAYS_EQUALS(input.ptr, input.len, decrypted_buf.buffer, decrypted_buf.len);

    aws_byte_buf_clean_up(&decrypted_buf);
    aws_byte_buf_clean_up(&encrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(aes_ctr_test_with_generated_key_iv, s_aes_ctr_test_with_generated_key_iv_fn)

static int s_aes_ctr_validate_materials_fails_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t iv_too_small[AWS_AES_256_CIPHER_BLOCK_SIZE - 1] = {0};
    uint8_t iv_too_large[AWS_AES_256_CIPHER_BLOCK_SIZE + 1] = {0};

    uint8_t key_too_small[AWS_AES_256_KEY_BYTE_LEN - 1] = {0};
    uint8_t key_too_large[AWS_AES_256_KEY_BYTE_LEN + 1] = {0};

    uint8_t valid_key_size[AWS_AES_256_KEY_BYTE_LEN] = {0};
    uint8_t valid_iv_size[AWS_AES_256_CIPHER_BLOCK_SIZE] = {0};

    struct aws_byte_cursor key = aws_byte_cursor_from_array(valid_key_size, sizeof(valid_key_size));
    struct aws_byte_cursor iv = aws_byte_cursor_from_array(iv_too_small, sizeof(iv_too_small));
    ASSERT_NULL(aws_aes_ctr_256_new(allocator, &key, &iv));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_CIPHER_MATERIAL_SIZE_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(valid_key_size, sizeof(valid_key_size));
    iv = aws_byte_cursor_from_array(iv_too_large, sizeof(iv_too_large));
    ASSERT_NULL(aws_aes_ctr_256_new(allocator, &key, &iv));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_CIPHER_MATERIAL_SIZE_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(key_too_small, sizeof(key_too_small));
    iv = aws_byte_cursor_from_array(valid_iv_size, sizeof(valid_iv_size));
    ASSERT_NULL(aws_aes_ctr_256_new(allocator, &key, &iv));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_KEY_LENGTH_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(key_too_small, sizeof(key_too_small));
    iv = aws_byte_cursor_from_array(key_too_large, sizeof(key_too_large));
    ASSERT_NULL(aws_aes_ctr_256_new(allocator, &key, &iv));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_KEY_LENGTH_FOR_ALGORITHM, aws_last_error());

    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(aes_ctr_validate_materials_fails, s_aes_ctr_validate_materials_fails_fn)

static int s_check_multi_block_gcm(
    struct aws_allocator *allocator,
    const struct aws_byte_cursor key,
    const struct aws_byte_cursor iv,
    const struct aws_byte_cursor data,
    const struct aws_byte_cursor expected,
    const struct aws_byte_cursor tag,
    const struct aws_byte_cursor *aad) {

    aws_cal_library_test_init(allocator);
    struct aws_symmetric_cipher *cipher = aws_aes_gcm_256_new(allocator, &key, &iv, aad);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_buf encrypted_buf;
    aws_byte_buf_init(&encrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    struct aws_byte_cursor data_cpy = data;
    /* slice on a weird boundary to hit boundary conditions. */
    while (data_cpy.len) {
        struct aws_byte_cursor to_encrypt = aws_byte_cursor_advance(&data_cpy, (size_t)aws_min_i64(24, data_cpy.len));
        AWS_LOGF_DEBUG(0, "to encrypt test size %zu", to_encrypt.len);
        ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, to_encrypt, &encrypted_buf));
    }
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypted_buf));

    ASSERT_BIN_ARRAYS_EQUALS(expected.ptr, expected.len, encrypted_buf.buffer, encrypted_buf.len);

    struct aws_byte_cursor encryption_tag = aws_symmetric_cipher_get_tag(cipher);
    ASSERT_BIN_ARRAYS_EQUALS(tag.ptr, tag.len, encryption_tag.ptr, encryption_tag.len);

    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted_buf);
    struct aws_byte_buf decrypted_buf;
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    aws_symmetric_cipher_set_tag(cipher, tag);

    /* slice on a weird boundary to hit boundary conditions. */
    while (encrypted_cur.len) {
        struct aws_byte_cursor to_decrypt =
            aws_byte_cursor_advance(&encrypted_cur, (size_t)aws_min_i64(24, encrypted_cur.len));
        ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, to_decrypt, &decrypted_buf));
        ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    }
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));
    ASSERT_BIN_ARRAYS_EQUALS(data.ptr, data.len, decrypted_buf.buffer, decrypted_buf.len);

    aws_byte_buf_clean_up(&decrypted_buf);
    aws_byte_buf_clean_up(&encrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}

static int s_gcm_NIST_gcmEncryptExtIV256_PTLen_128_Test_0_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {
        0x0D,
        0x18,
        0xE0,
        0x6C,
        0x7C,
        0x72,
        0x5A,
        0xC9,
        0xE3,
        0x62,
        0xE1,
        0xCE,
    };

    uint8_t key[] = {
        0x31, 0xBD, 0xAD, 0xD9, 0x66, 0x98, 0xC2, 0x04, 0xAA, 0x9C, 0xE1, 0x44, 0x8E, 0xA9, 0x4A, 0xE1,
        0xFB, 0x4A, 0x9A, 0x0B, 0x3C, 0x9D, 0x77, 0x3B, 0x51, 0xBB, 0x18, 0x22, 0x66, 0x6B, 0x8F, 0x22,
    };

    uint8_t data[] = {
        0x2D,
        0xB5,
        0x16,
        0x8E,
        0x93,
        0x25,
        0x56,
        0xF8,
        0x08,
        0x9A,
        0x06,
        0x22,
        0x98,
        0x1D,
        0x01,
        0x7D,
    };

    uint8_t expected[] = {
        0xFA,
        0x43,
        0x62,
        0x18,
        0x96,
        0x61,
        0xD1,
        0x63,
        0xFC,
        0xD6,
        0xA5,
        0x6D,
        0x8B,
        0xF0,
        0x40,
        0x5A,
    };

    uint8_t tag[] = {
        0xD6,
        0x36,
        0xAC,
        0x1B,
        0xBE,
        0xDD,
        0x5C,
        0xC3,
        0xEE,
        0x72,
        0x7D,
        0xC2,
        0xAB,
        0x4A,
        0x94,
        0x89,
    };

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));
    struct aws_byte_cursor tag_cur = aws_byte_cursor_from_array(tag, sizeof(tag));

    return s_check_multi_block_gcm(allocator, key_cur, iv_cur, data_cur, expected_cur, tag_cur, NULL);
}
AWS_TEST_CASE(gcm_NIST_gcmEncryptExtIV256_PTLen_128_Test_0, s_gcm_NIST_gcmEncryptExtIV256_PTLen_128_Test_0_fn)

static int s_gcm_NIST_gcmEncryptExtIV256_PTLen_104_Test_3_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {
        0x47,
        0x42,
        0x35,
        0x7C,
        0x33,
        0x59,
        0x13,
        0x15,
        0x3F,
        0xF0,
        0xEB,
        0x0F,
    };

    uint8_t key[] = {
        0xE5, 0xA0, 0xEB, 0x92, 0xCC, 0x2B, 0x06, 0x4E, 0x1B, 0xC8, 0x08, 0x91, 0xFA, 0xF1, 0xFA, 0xB5,
        0xE9, 0xA1, 0x7A, 0x9C, 0x3A, 0x98, 0x4E, 0x25, 0x41, 0x67, 0x20, 0xE3, 0x0E, 0x6C, 0x2B, 0x21,
    };

    uint8_t data[] = {
        0x84,
        0x99,
        0x89,
        0x3E,
        0x16,
        0xB0,
        0xBA,
        0x8B,
        0x00,
        0x7D,
        0x54,
        0x66,
        0x5A,
    };

    uint8_t expected[] = {
        0xEB,
        0x8E,
        0x61,
        0x75,
        0xF1,
        0xFE,
        0x38,
        0xEB,
        0x1A,
        0xCF,
        0x95,
        0xFD,
        0x51,
    };

    uint8_t tag[] = {
        0x88,
        0xA8,
        0xB7,
        0x4B,
        0xB7,
        0x4F,
        0xDA,
        0x55,
        0x3E,
        0x91,
        0x02,
        0x0A,
        0x23,
        0xDE,
        0xED,
        0x45,
    };

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));
    struct aws_byte_cursor tag_cur = aws_byte_cursor_from_array(tag, sizeof(tag));

    return s_check_multi_block_gcm(allocator, key_cur, iv_cur, data_cur, expected_cur, tag_cur, NULL);
}
AWS_TEST_CASE(gcm_NIST_gcmEncryptExtIV256_PTLen_104_Test_3, s_gcm_NIST_gcmEncryptExtIV256_PTLen_104_Test_3_fn)

static int s_gcm_NIST_gcmEncryptExtIV256_PTLen_256_Test_6_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {
        0xA2,
        0x91,
        0x48,
        0x4C,
        0x3D,
        0xE8,
        0xBE,
        0xC6,
        0xB4,
        0x7F,
        0x52,
        0x5F,
    };

    uint8_t key[] = {
        0x37, 0xF3, 0x91, 0x37, 0x41, 0x6B, 0xAF, 0xDE, 0x6F, 0x75, 0x02, 0x2A, 0x7A, 0x52, 0x7C, 0xC5,
        0x93, 0xB6, 0x00, 0x0A, 0x83, 0xFF, 0x51, 0xEC, 0x04, 0x87, 0x1A, 0x0F, 0xF5, 0x36, 0x0E, 0x4E,
    };

    uint8_t data[] = {0xFA, 0xFD, 0x94, 0xCE, 0xDE, 0x8B, 0x5A, 0x07, 0x30, 0x39, 0x4B, 0xEC, 0x68, 0xA8, 0xE7, 0x7D,
                      0xBA, 0x28, 0x8D, 0x6C, 0xCA, 0xA8, 0xE1, 0x56, 0x3A, 0x81, 0xD6, 0xE7, 0xCC, 0xC7, 0xFC, 0x97};

    uint8_t expected[] = {
        0x44, 0xDC, 0x86, 0x80, 0x06, 0xB2, 0x1D, 0x49, 0x28, 0x40, 0x16, 0x56, 0x5F, 0xFB, 0x39, 0x79,
        0xCC, 0x42, 0x71, 0xD9, 0x67, 0x62, 0x8B, 0xF7, 0xCD, 0xAF, 0x86, 0xDB, 0x88, 0x8E, 0x92, 0xE5,
    };

    uint8_t tag[] = {
        0x01,
        0xA2,
        0xB5,
        0x78,
        0xAA,
        0x2F,
        0x41,
        0xEC,
        0x63,
        0x79,
        0xA4,
        0x4A,
        0x31,
        0xCC,
        0x01,
        0x9C,
    };

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));
    struct aws_byte_cursor tag_cur = aws_byte_cursor_from_array(tag, sizeof(tag));

    return s_check_multi_block_gcm(allocator, key_cur, iv_cur, data_cur, expected_cur, tag_cur, NULL);
}
AWS_TEST_CASE(gcm_NIST_gcmEncryptExtIV256_PTLen_256_Test_6, s_gcm_NIST_gcmEncryptExtIV256_PTLen_256_Test_6_fn)

static int s_gcm_NIST_gcmEncryptExtIV256_PTLen_408_Test_8_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {
        0x92,
        0xF2,
        0x58,
        0x07,
        0x1D,
        0x79,
        0xAF,
        0x3E,
        0x63,
        0x67,
        0x22,
        0x85,
    };

    uint8_t key[] = {
        0x59, 0x5F, 0x25, 0x9C, 0x55, 0xAB, 0xE0, 0x0A, 0xE0, 0x75, 0x35, 0xCA, 0x5D, 0x9B, 0x09, 0xD6,
        0xEF, 0xB9, 0xF7, 0xE9, 0xAB, 0xB6, 0x46, 0x05, 0xC3, 0x37, 0xAC, 0xBD, 0x6B, 0x14, 0xFC, 0x7E,
    };

    uint8_t data[] = {
        0xA6, 0xFE, 0xE3, 0x3E, 0xB1, 0x10, 0xA2, 0xD7, 0x69, 0xBB, 0xC5, 0x2B, 0x0F, 0x36, 0x96, 0x9C, 0x28,
        0x78, 0x74, 0xF6, 0x65, 0x68, 0x14, 0x77, 0xA2, 0x5F, 0xC4, 0xC4, 0x80, 0x15, 0xC5, 0x41, 0xFB, 0xE2,
        0x39, 0x41, 0x33, 0xBA, 0x49, 0x0A, 0x34, 0xEE, 0x2D, 0xD6, 0x7B, 0x89, 0x81, 0x77, 0x84, 0x9A, 0x91,
    };

    uint8_t expected[] = {
        0xBB, 0xCA, 0x4A, 0x9E, 0x09, 0xAE, 0x96, 0x90, 0xC0, 0xF6, 0xF8, 0xD4, 0x05, 0xE5, 0x3D, 0xCC, 0xD6,
        0x66, 0xAA, 0x9C, 0x5F, 0xA1, 0x3C, 0x87, 0x58, 0xBC, 0x30, 0xAB, 0xE1, 0xDD, 0xD1, 0xBC, 0xCE, 0x0D,
        0x36, 0xA1, 0xEA, 0xAA, 0xAF, 0xFE, 0xF2, 0x0C, 0xD3, 0xC5, 0x97, 0x0B, 0x96, 0x73, 0xF8, 0xA6, 0x5C,
    };

    uint8_t tag[] = {0x26, 0xCC, 0xEC, 0xB9, 0x97, 0x6F, 0xD6, 0xAC, 0x9C, 0x2C, 0x0F, 0x37, 0x2C, 0x52, 0xC8, 0x21};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));
    struct aws_byte_cursor tag_cur = aws_byte_cursor_from_array(tag, sizeof(tag));

    return s_check_multi_block_gcm(allocator, key_cur, iv_cur, data_cur, expected_cur, tag_cur, NULL);
}
AWS_TEST_CASE(gcm_NIST_gcmEncryptExtIV256_PTLen_408_Test_8, s_gcm_NIST_gcmEncryptExtIV256_PTLen_408_Test_8_fn)

static int s_gcm_256_KAT_1_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {
        0xFB,
        0x7B,
        0x4A,
        0x82,
        0x4E,
        0x82,
        0xDA,
        0xA6,
        0xC8,
        0xBC,
        0x12,
        0x51,
    };

    uint8_t key[] = {
        0x20, 0x14, 0x2E, 0x89, 0x8C, 0xD2, 0xFD, 0x98, 0x0F, 0xBF, 0x34, 0xDE, 0x6B, 0xC8, 0x5C, 0x14,
        0xDA, 0x7D, 0x57, 0xBD, 0x28, 0xF4, 0xAA, 0x5C, 0xF1, 0x72, 0x8A, 0xB6, 0x4E, 0x84, 0x31, 0x42,
    };

    uint8_t aad[] = {
        0x16, 0x7B, 0x5C, 0x22, 0x61, 0x77, 0x73, 0x3A, 0x78, 0x2D, 0x61, 0x6D, 0x7A, 0x2D, 0x63, 0x65,
        0x6B, 0x2D, 0x61, 0x6C, 0x67, 0x5C, 0x22, 0x3A, 0x20, 0x5C, 0x22, 0x41, 0x45, 0x53, 0x2F, 0x47,
        0x43, 0x4D, 0x2F, 0x4E, 0x6F, 0x50, 0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x5C, 0x22, 0x7D,
    };

    uint8_t tag[] = {
        0x81,
        0xC0,
        0xE4,
        0x2B,
        0xB1,
        0x95,
        0xE2,
        0x62,
        0xCB,
        0x3B,
        0x3A,
        0x74,
        0xA0,
        0xDA,
        0xE1,
        0xC8,
    };

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = {0};
    struct aws_byte_cursor expected_cur = {0};
    struct aws_byte_cursor tag_cur = aws_byte_cursor_from_array(tag, sizeof(tag));
    struct aws_byte_cursor aad_cur = aws_byte_cursor_from_array(aad, sizeof(aad));

    return s_check_multi_block_gcm(allocator, key_cur, iv_cur, data_cur, expected_cur, tag_cur, &aad_cur);
}
AWS_TEST_CASE(gcm_256_KAT_1, s_gcm_256_KAT_1_fn)

static int s_gcm_256_KAT_2_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    uint8_t iv[] = {
        0x6B,
        0x5C,
        0xD3,
        0x70,
        0x5A,
        0x73,
        0x3C,
        0x1A,
        0xD9,
        0x43,
        0xD5,
        0x8A,
    };

    uint8_t key[] = {
        0xD2, 0x11, 0xF2, 0x78, 0xA4, 0x4E, 0xAB, 0x66, 0x6B, 0x10, 0x21, 0xF4, 0xB4, 0xF6, 0x0B, 0xA6,
        0xB7, 0x44, 0x64, 0xFA, 0x9C, 0xB7, 0xB1, 0x34, 0x93, 0x4D, 0x78, 0x91, 0xE1, 0x47, 0x91, 0x69,
    };

    uint8_t aad[] = {
        0x16, 0x7B, 0x5C, 0x22, 0x61, 0x77, 0x73, 0x3A, 0x78, 0x2D, 0x61, 0x6D, 0x7A, 0x2D, 0x63, 0x65,
        0x6B, 0x2D, 0x61, 0x6C, 0x67, 0x5C, 0x22, 0x3A, 0x20, 0x5C, 0x22, 0x41, 0x45, 0x53, 0x2F, 0x47,
        0x43, 0x4D, 0x2F, 0x4E, 0x6F, 0x50, 0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x5C, 0x22, 0x7D,
    };

    uint8_t data[] = {
        0x16, 0x7B, 0x5C, 0x22, 0x61, 0x77, 0x73, 0x3A, 0x78, 0x2D, 0x61, 0x6D, 0x7A, 0x2D, 0x63, 0x65,
        0x6B, 0x2D, 0x61, 0x6C, 0x67, 0x5C, 0x22, 0x3A, 0x20, 0x5C, 0x22, 0x41, 0x45, 0x53, 0x2F, 0x47,
        0x43, 0x4D, 0x2F, 0x4E, 0x6F, 0x50, 0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x5C, 0x22, 0x7D,
    };

    uint8_t expected[] = {
        0x4C, 0x25, 0xAB, 0xD6, 0x6D, 0x3A, 0x1B, 0xCC, 0xE7, 0x94, 0xAC, 0xAA, 0xF4, 0xCE, 0xFD, 0xF6,
        0xD2, 0x55, 0x2F, 0x4A, 0x82, 0xC5, 0x0A, 0x98, 0xCB, 0x15, 0xB4, 0x81, 0x2F, 0xF5, 0x57, 0xAB,
        0xE5, 0x64, 0xA9, 0xCE, 0xFF, 0x15, 0xF3, 0x2D, 0xCF, 0x5A, 0x5A, 0xA7, 0x89, 0x48, 0x88,
    };

    uint8_t tag[] = {
        0x03,
        0xED,
        0xE7,
        0x1E,
        0xC9,
        0x52,
        0xE6,
        0x5A,
        0xE7,
        0xB4,
        0xB8,
        0x5C,
        0xFE,
        0xC7,
        0xD3,
        0x04,
    };

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));
    struct aws_byte_cursor tag_cur = aws_byte_cursor_from_array(tag, sizeof(tag));
    struct aws_byte_cursor aad_cur = aws_byte_cursor_from_array(aad, sizeof(aad));

    return s_check_multi_block_gcm(allocator, key_cur, iv_cur, data_cur, expected_cur, tag_cur, &aad_cur);
}
AWS_TEST_CASE(gcm_256_KAT_2, s_gcm_256_KAT_2_fn)

static int s_gcm_256_KAT_3_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    uint8_t iv[] = {
        0x5F,
        0x08,
        0xEF,
        0xBF,
        0xB7,
        0xBF,
        0x5B,
        0xA3,
        0x65,
        0xD9,
        0xEB,
        0x1D,
    };

    uint8_t key[] = {
        0xCF, 0xE8, 0xBF, 0xE6, 0x1B, 0x89, 0xAF, 0x53, 0xD2, 0xBE, 0xCE, 0x74, 0x4D, 0x27, 0xB7, 0x8C,
        0x9E, 0x4D, 0x74, 0xD0, 0x28, 0xCE, 0x88, 0xED, 0x10, 0xA4, 0x22, 0x28, 0x5B, 0x12, 0x01, 0xC9,
    };

    uint8_t data[] = {
        0x16, 0x7B, 0x5C, 0x22, 0x61, 0x77, 0x73, 0x3A, 0x78, 0x2D, 0x61, 0x6D, 0x7A, 0x2D, 0x63, 0x65,
        0x6B, 0x2D, 0x61, 0x6C, 0x67, 0x5C, 0x22, 0x3A, 0x20, 0x5C, 0x22, 0x41, 0x45, 0x53, 0x2F, 0x47,
        0x43, 0x4D, 0x2F, 0x4E, 0x6F, 0x50, 0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x5C, 0x22, 0x7D,
    };

    uint8_t expected[] = {
        0x0A, 0x7E, 0x82, 0xF1, 0xE5, 0xC7, 0x6C, 0x69, 0x67, 0x96, 0x71, 0xEE, 0xAE, 0xE4, 0x55, 0x93,
        0x6F, 0x2C, 0x4F, 0xCC, 0xD9, 0xDD, 0xF1, 0xFA, 0xA2, 0x70, 0x75, 0xE2, 0x04, 0x06, 0x44, 0x93,
        0x89, 0x20, 0xC5, 0xD1, 0x6C, 0x69, 0xE4, 0xD9, 0x33, 0x75, 0x48, 0x7B, 0x9A, 0x80, 0xD4,
    };

    uint8_t tag[] = {
        0x04,
        0x34,
        0x7D,
        0x0C,
        0x5B,
        0x0E,
        0x0D,
        0xE8,
        0x9E,
        0x03,
        0x3D,
        0x04,
        0xD0,
        0x49,
        0x3D,
        0xCA,
    };

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));
    struct aws_byte_cursor expected_cur = aws_byte_cursor_from_array(expected, sizeof(expected));
    struct aws_byte_cursor tag_cur = aws_byte_cursor_from_array(tag, sizeof(tag));
    struct aws_byte_cursor aad_cur = {0};

    return s_check_multi_block_gcm(allocator, key_cur, iv_cur, data_cur, expected_cur, tag_cur, &aad_cur);
}
AWS_TEST_CASE(gcm_256_KAT_3, s_gcm_256_KAT_3_fn)

static int s_aes_gcm_test_with_generated_key_iv_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);
    struct aws_symmetric_cipher *cipher = aws_aes_gcm_256_new(allocator, NULL, NULL, NULL);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_buf encrypted_buf;
    aws_byte_buf_init(&encrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);

    struct aws_byte_cursor input = aws_byte_cursor_from_c_str(TEST_ENCRYPTION_STRING);
    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input, &encrypted_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypted_buf));

    struct aws_byte_buf encryption_tag;
    aws_byte_buf_init_copy_from_cursor(&encryption_tag, allocator, aws_symmetric_cipher_get_tag(cipher));

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    aws_symmetric_cipher_set_tag(cipher, aws_byte_cursor_from_buf(&encryption_tag));

    struct aws_byte_buf decrypted_buf;
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    struct aws_byte_cursor encrypted_cur = aws_byte_cursor_from_buf(&encrypted_buf);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_cur, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));

    ASSERT_BIN_ARRAYS_EQUALS(input.ptr, input.len, decrypted_buf.buffer, decrypted_buf.len);

    aws_byte_buf_clean_up(&encryption_tag);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_byte_buf_clean_up(&encrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(gcm_test_with_generated_key_iv, s_aes_gcm_test_with_generated_key_iv_fn)

static int s_aes_gcm_validate_materials_fails_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t iv_too_small[AWS_AES_256_CIPHER_BLOCK_SIZE - 5] = {0};
    uint8_t iv_too_large[AWS_AES_256_CIPHER_BLOCK_SIZE - 3] = {0};

    uint8_t key_too_small[AWS_AES_256_KEY_BYTE_LEN - 1] = {0};
    uint8_t key_too_large[AWS_AES_256_KEY_BYTE_LEN + 1] = {0};

    uint8_t valid_key_size[AWS_AES_256_KEY_BYTE_LEN] = {0};
    uint8_t valid_iv_size[AWS_AES_256_CIPHER_BLOCK_SIZE] = {0};

    struct aws_byte_cursor key = aws_byte_cursor_from_array(valid_key_size, sizeof(valid_key_size));
    struct aws_byte_cursor iv = aws_byte_cursor_from_array(iv_too_small, sizeof(iv_too_small));
    ASSERT_NULL(aws_aes_gcm_256_new(allocator, &key, &iv, NULL));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_CIPHER_MATERIAL_SIZE_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(valid_key_size, sizeof(valid_key_size));
    iv = aws_byte_cursor_from_array(iv_too_large, sizeof(iv_too_large));
    ASSERT_NULL(aws_aes_gcm_256_new(allocator, &key, &iv, NULL));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_CIPHER_MATERIAL_SIZE_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(key_too_small, sizeof(key_too_small));
    iv = aws_byte_cursor_from_array(valid_iv_size, sizeof(valid_iv_size));
    ASSERT_NULL(aws_aes_gcm_256_new(allocator, &key, &iv, NULL));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_KEY_LENGTH_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(key_too_small, sizeof(key_too_small));
    iv = aws_byte_cursor_from_array(key_too_large, sizeof(key_too_large));
    ASSERT_NULL(aws_aes_gcm_256_new(allocator, &key, &iv, NULL));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_KEY_LENGTH_FOR_ALGORITHM, aws_last_error());

    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(aes_gcm_validate_materials_fails, s_aes_gcm_validate_materials_fails_fn)

static int s_test_aes_keywrap_RFC3394_256BitKey256CekTestVector(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t key[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };
    size_t key_length = sizeof(key);

    uint8_t input[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
                       0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
    size_t input_length = sizeof(input);

    uint8_t expected_output[] = {0x28, 0xC9, 0xF4, 0x04, 0xC4, 0xB8, 0x10, 0xF4, 0xCB, 0xCC, 0xB3, 0x5C, 0xFB, 0x87,
                                 0xF8, 0x26, 0x3F, 0x57, 0x86, 0xE2, 0xD8, 0x0E, 0xD3, 0x26, 0xCB, 0xC7, 0xF0, 0xE7,
                                 0x1A, 0x99, 0xF4, 0x3B, 0xFB, 0x98, 0x8B, 0x9B, 0x7A, 0x02, 0xDD, 0x21};
    size_t expected_output_length = sizeof(expected_output);

    struct aws_byte_cursor input_cur = aws_byte_cursor_from_array(input, input_length);
    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, key_length);
    struct aws_byte_buf output_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&output_buf, allocator, expected_output_length));

    struct aws_symmetric_cipher *cipher = aws_aes_keywrap_256_new(allocator, &key_cur);
    ASSERT_NOT_NULL(cipher);

    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input_cur, &output_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &output_buf));
    ASSERT_BIN_ARRAYS_EQUALS(expected_output, expected_output_length, output_buf.buffer, output_buf.len);

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    struct aws_byte_buf decrypted_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted_buf, allocator, input_length));

    struct aws_byte_cursor encrypted_data = aws_byte_cursor_from_buf(&output_buf);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_data, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));
    ASSERT_BIN_ARRAYS_EQUALS(input, input_length, decrypted_buf.buffer, decrypted_buf.len);

    aws_symmetric_cipher_destroy(cipher);
    aws_byte_buf_clean_up(&output_buf);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(aes_keywrap_RFC3394_256BitKey256CekTestVector, s_test_aes_keywrap_RFC3394_256BitKey256CekTestVector);

static int s_test_Rfc3394_256BitKey_TestIntegrityCheckFailed(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t input[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
                       0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    size_t input_length = sizeof(input);

    uint8_t key[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    size_t key_length = sizeof(key);

    uint8_t expected_output[] = {0x28, 0xC9, 0xF4, 0x04, 0xC4, 0xB8, 0x10, 0xF4, 0xCB, 0xCC, 0xB3, 0x5C, 0xFB, 0x87,
                                 0xF8, 0x26, 0x3F, 0x57, 0x86, 0xE2, 0xD8, 0x0E, 0xD3, 0x26, 0xCB, 0xC7, 0xF0, 0xE7,
                                 0x1A, 0x99, 0xF4, 0x3B, 0xFB, 0x98, 0x8B, 0x9B, 0x7A, 0x02, 0xDD, 0x21};
    size_t expected_output_length = sizeof(expected_output);

    struct aws_byte_cursor input_cur = aws_byte_cursor_from_array(input, input_length);
    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, key_length);
    struct aws_byte_buf output_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&output_buf, allocator, expected_output_length));

    struct aws_symmetric_cipher *cipher = aws_aes_keywrap_256_new(allocator, &key_cur);
    ASSERT_NOT_NULL(cipher);

    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input_cur, &output_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &output_buf));
    ASSERT_BIN_ARRAYS_EQUALS(expected_output, expected_output_length, output_buf.buffer, output_buf.len);

    /* Mutate one byte of the encrypted data */
    output_buf.buffer[0] ^= 0x01;

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    struct aws_byte_buf decrypted_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted_buf, allocator, input_length));

    struct aws_byte_cursor encrypted_data = aws_byte_cursor_from_buf(&output_buf);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_data, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_FAILS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_ERROR, aws_symmetric_cipher_get_state(cipher));
    ASSERT_FALSE(aws_symmetric_cipher_is_good(cipher));

    aws_symmetric_cipher_destroy(cipher);
    aws_byte_buf_clean_up(&output_buf);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    aes_keywrap_Rfc3394_256BitKey_TestIntegrityCheckFailed,
    s_test_Rfc3394_256BitKey_TestIntegrityCheckFailed);

static int s_test_RFC3394_256BitKeyTestBadPayload(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t input[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
                       0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    size_t input_length = sizeof(input);

    uint8_t key[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    size_t key_length = sizeof(key);

    uint8_t expected_output[] = {0x28, 0xC9, 0xF4, 0x04, 0xC4, 0xB8, 0x10, 0xF4, 0xCB, 0xCC, 0xB3, 0x5C, 0xFB, 0x87,
                                 0xF8, 0x26, 0x3F, 0x57, 0x86, 0xE2, 0xD8, 0x0E, 0xD3, 0x26, 0xCB, 0xC7, 0xF0, 0xE7,
                                 0x1A, 0x99, 0xF4, 0x3B, 0xFB, 0x98, 0x8B, 0x9B, 0x7A, 0x02, 0xDD, 0x21};
    size_t expected_output_length = sizeof(expected_output);

    struct aws_byte_cursor input_cur = aws_byte_cursor_from_array(input, input_length);
    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, key_length);
    struct aws_byte_buf output_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&output_buf, allocator, expected_output_length));

    struct aws_symmetric_cipher *cipher = aws_aes_keywrap_256_new(allocator, &key_cur);
    ASSERT_NOT_NULL(cipher);

    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input_cur, &output_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &output_buf));
    ASSERT_BIN_ARRAYS_EQUALS(expected_output, expected_output_length, output_buf.buffer, output_buf.len);

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    struct aws_byte_buf decrypted_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted_buf, allocator, input_length));

    struct aws_byte_cursor encrypted_data = aws_byte_cursor_from_buf(&output_buf);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_data, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));
    ASSERT_BIN_ARRAYS_EQUALS(input, input_length, decrypted_buf.buffer, decrypted_buf.len);

    aws_symmetric_cipher_destroy(cipher);
    aws_byte_buf_clean_up(&output_buf);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(aes_keywrap_RFC3394_256BitKeyTestBadPayload, s_test_RFC3394_256BitKeyTestBadPayload);

static int s_test_RFC3394_256BitKey128BitCekTestVector(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t input[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    size_t input_length = sizeof(input);

    uint8_t key[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    size_t key_length = sizeof(key);

    uint8_t expected_output[] = {0x64, 0xE8, 0xC3, 0xF9, 0xCE, 0x0F, 0x5B, 0xA2, 0x63, 0xE9, 0x77, 0x79,
                                 0x05, 0x81, 0x8A, 0x2A, 0x93, 0xC8, 0x19, 0x1E, 0x7D, 0x6E, 0x8A, 0xE7};
    size_t expected_output_length = sizeof(expected_output);

    struct aws_byte_cursor input_cur = aws_byte_cursor_from_array(input, input_length);
    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, key_length);
    struct aws_byte_buf output_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&output_buf, allocator, expected_output_length));

    struct aws_symmetric_cipher *cipher = aws_aes_keywrap_256_new(allocator, &key_cur);
    ASSERT_NOT_NULL(cipher);

    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input_cur, &output_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &output_buf));
    ASSERT_BIN_ARRAYS_EQUALS(expected_output, expected_output_length, output_buf.buffer, output_buf.len);

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));
    ASSERT_TRUE(aws_symmetric_cipher_is_good(cipher));

    struct aws_byte_buf decrypted_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted_buf, allocator, input_length));

    struct aws_byte_cursor encrypted_data = aws_byte_cursor_from_buf(&output_buf);
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_data, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_FINALIZED, aws_symmetric_cipher_get_state(cipher));
    aws_symmetric_cipher_destroy(cipher);
    aws_byte_buf_clean_up(&output_buf);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(aes_keywrap_RFC3394_256BitKey128BitCekTestVector, s_test_RFC3394_256BitKey128BitCekTestVector);

static int s_test_RFC3394_256BitKey128BitCekIntegrityCheckFailedTestVector(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t input[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    size_t input_length = sizeof(input);

    uint8_t key[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    size_t key_length = sizeof(key);

    uint8_t expected_output[] = {0x64, 0xE8, 0xC3, 0xF9, 0xCE, 0x0F, 0x5B, 0xA2, 0x63, 0xE9, 0x77, 0x79,
                                 0x05, 0x81, 0x8A, 0x2A, 0x93, 0xC8, 0x19, 0x1E, 0x7D, 0x6E, 0x8A, 0xE7};
    size_t expected_output_length = sizeof(expected_output);

    struct aws_byte_cursor input_cur = aws_byte_cursor_from_array(input, input_length);
    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, key_length);
    struct aws_byte_buf output_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&output_buf, allocator, expected_output_length));

    struct aws_symmetric_cipher *cipher = aws_aes_keywrap_256_new(allocator, &key_cur);
    ASSERT_NOT_NULL(cipher);

    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input_cur, &output_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &output_buf));
    ASSERT_BIN_ARRAYS_EQUALS(expected_output, expected_output_length, output_buf.buffer, output_buf.len);

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    struct aws_byte_buf decrypted_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted_buf, allocator, input_length));

    struct aws_byte_cursor encrypted_data = aws_byte_cursor_from_buf(&output_buf);
    encrypted_data.ptr[1] = encrypted_data.ptr[1] + encrypted_data.ptr[2];
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_data, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_FAILS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_ERROR, aws_symmetric_cipher_get_state(cipher));
    ASSERT_FALSE(aws_symmetric_cipher_is_good(cipher));

    aws_symmetric_cipher_destroy(cipher);
    aws_byte_buf_clean_up(&output_buf);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    aes_keywrap_RFC3394_256BitKey128BitCekIntegrityCheckFailedTestVector,
    s_test_RFC3394_256BitKey128BitCekIntegrityCheckFailedTestVector);

static int s_test_RFC3394_256BitKey128BitCekPayloadCheckFailedTestVector(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t input[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    size_t input_length = sizeof(input);

    uint8_t key[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    size_t key_length = sizeof(key);

    uint8_t expected_output[] = {0x64, 0xE8, 0xC3, 0xF9, 0xCE, 0x0F, 0x5B, 0xA2, 0x63, 0xE9, 0x77, 0x79,
                                 0x05, 0x81, 0x8A, 0x2A, 0x93, 0xC8, 0x19, 0x1E, 0x7D, 0x6E, 0x8A, 0xE7};
    size_t expected_output_length = sizeof(expected_output);

    struct aws_byte_cursor input_cur = aws_byte_cursor_from_array(input, input_length);
    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, key_length);
    struct aws_byte_buf output_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&output_buf, allocator, expected_output_length));

    struct aws_symmetric_cipher *cipher = aws_aes_keywrap_256_new(allocator, &key_cur);
    ASSERT_NOT_NULL(cipher);

    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, input_cur, &output_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &output_buf));
    ASSERT_BIN_ARRAYS_EQUALS(expected_output, expected_output_length, output_buf.buffer, output_buf.len);

    ASSERT_SUCCESS(aws_symmetric_cipher_reset(cipher));

    struct aws_byte_buf decrypted_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&decrypted_buf, allocator, input_length));

    struct aws_byte_cursor encrypted_data = aws_byte_cursor_from_buf(&output_buf);
    encrypted_data.ptr[14] = encrypted_data.ptr[13] + encrypted_data.ptr[14];
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, encrypted_data, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_FAILS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_ERROR, aws_symmetric_cipher_get_state(cipher));
    ASSERT_FALSE(aws_symmetric_cipher_is_good(cipher));

    aws_symmetric_cipher_destroy(cipher);
    aws_byte_buf_clean_up(&output_buf);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}

AWS_TEST_CASE(
    aes_keywrap_RFC3394_256BitKey128BitCekPayloadCheckFailedTestVector,
    s_test_RFC3394_256BitKey128BitCekPayloadCheckFailedTestVector);

static int s_aes_keywrap_validate_materials_fails_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t key_too_small[AWS_AES_256_KEY_BYTE_LEN - 1] = {0};
    uint8_t key_too_large[AWS_AES_256_KEY_BYTE_LEN + 1] = {0};

    struct aws_byte_cursor key = aws_byte_cursor_from_array(key_too_small, sizeof(key_too_small));
    ASSERT_NULL(aws_aes_keywrap_256_new(allocator, &key));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_KEY_LENGTH_FOR_ALGORITHM, aws_last_error());

    key = aws_byte_cursor_from_array(key_too_large, sizeof(key_too_large));
    ASSERT_NULL(aws_aes_keywrap_256_new(allocator, &key));
    ASSERT_UINT_EQUALS(AWS_ERROR_CAL_INVALID_KEY_LENGTH_FOR_ALGORITHM, aws_last_error());

    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(aes_keywrap_validate_materials_fails, s_aes_keywrap_validate_materials_fails_fn)

static int s_test_input_too_large_fn(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t iv[AWS_AES_256_CIPHER_BLOCK_SIZE] = {0};
    uint8_t key[AWS_AES_256_KEY_BYTE_LEN] = {0};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));

    struct aws_symmetric_cipher *cipher = aws_aes_cbc_256_new(allocator, &key_cur, &iv_cur);
    ASSERT_NOT_NULL(cipher);

    struct aws_byte_cursor invalid_cur = {
        .ptr = key,
        .len = INT_MAX,
    };

    ASSERT_ERROR(AWS_ERROR_CAL_BUFFER_TOO_LARGE_FOR_ALGORITHM, aws_symmetric_cipher_encrypt(cipher, invalid_cur, NULL));
    /* should still be good from an invalid input. */
    ASSERT_TRUE(aws_symmetric_cipher_is_good(cipher));
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_ERROR(AWS_ERROR_CAL_BUFFER_TOO_LARGE_FOR_ALGORITHM, aws_symmetric_cipher_decrypt(cipher, invalid_cur, NULL));
    /* should still be good from an invalid input. */
    ASSERT_INT_EQUALS(AWS_SYMMETRIC_CIPHER_READY, aws_symmetric_cipher_get_state(cipher));
    ASSERT_TRUE(aws_symmetric_cipher_is_good(cipher));

    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(aes_test_input_too_large, s_test_input_too_large_fn)

static int s_aes_test_encrypt_empty_input(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
    aws_cal_library_test_init(allocator);

    uint8_t iv[] = {0xFB, 0x7B, 0x4A, 0x82, 0x4E, 0x82, 0xDA, 0xA6, 0xC8, 0xBC, 0x12, 0x51};

    uint8_t key[] = {0x20, 0x14, 0x2E, 0x89, 0x8C, 0xD2, 0xFD, 0x98, 0x0F, 0xBF, 0x34, 0xDE, 0x6B, 0xC8, 0x5C, 0x14,
                     0xDA, 0x7D, 0x57, 0xBD, 0x28, 0xF4, 0xAA, 0x5C, 0xF1, 0x72, 0x8A, 0xB6, 0x4E, 0x84, 0x31, 0x42};

    uint8_t aad[] = {0x16, 0x7B, 0x5C, 0x22, 0x61, 0x77, 0x73, 0x3A, 0x78, 0x2D, 0x61, 0x6D, 0x7A, 0x2D, 0x63, 0x65,
                     0x6B, 0x2D, 0x61, 0x6C, 0x67, 0x5C, 0x22, 0x3A, 0x20, 0x5C, 0x22, 0x41, 0x45, 0x53, 0x2F, 0x47,
                     0x43, 0x4D, 0x2F, 0x4E, 0x6F, 0x50, 0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x5C, 0x22, 0x7D};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor aad_cur = aws_byte_cursor_from_array(aad, sizeof(aad));

    struct aws_symmetric_cipher *cipher = aws_aes_gcm_256_new(allocator, &key_cur, &iv_cur, &aad_cur);

    // encrypt
    struct aws_byte_cursor data_cur = {0};
    struct aws_byte_buf encrypt_buf = {0};
    aws_byte_buf_init(&encrypt_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE * 2);
    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, data_cur, &encrypt_buf));

    // finalize
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypt_buf));

    ASSERT_INT_EQUALS(0, encrypt_buf.len);

    struct aws_byte_buf encryption_tag;
    aws_byte_buf_init_copy_from_cursor(&encryption_tag, allocator, aws_symmetric_cipher_get_tag(cipher));

    aws_symmetric_cipher_reset(cipher);

    aws_symmetric_cipher_set_tag(cipher, aws_byte_cursor_from_buf(&encryption_tag));

    struct aws_byte_buf decrypted_buf = {0};
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    struct aws_byte_cursor ciphertext_cur = aws_byte_cursor_from_buf(&encrypt_buf);
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, ciphertext_cur, &decrypted_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));

    aws_byte_buf_clean_up(&encryption_tag);
    aws_byte_buf_clean_up(&encrypt_buf);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();

    return AWS_OP_SUCCESS;
}
AWS_TEST_CASE(aes_test_encrypt_empty_input, s_aes_test_encrypt_empty_input)

static int s_aes_gcm_corner_case_checker(
    struct aws_allocator *allocator,
    struct aws_byte_cursor key_cur,
    struct aws_byte_cursor iv_cur,
    struct aws_byte_cursor aad_cur,
    struct aws_byte_cursor data_cur,
    struct aws_byte_cursor expected_tag_cur) {

    aws_cal_library_test_init(allocator);

    /* just a random tag value which should not match anything*/
    uint8_t wrong_tag[] = {
        0x83, 0xC0, 0xE4, 0x2B, 0xB1, 0x95, 0xE2, 0x62, 0xCB, 0x3B, 0x3A, 0x74, 0xA0, 0xDA, 0xE1, 0xC8};
    struct aws_byte_cursor wrong_tag_cur = aws_byte_cursor_from_array(wrong_tag, sizeof(wrong_tag));

    struct aws_symmetric_cipher *cipher = aws_aes_gcm_256_new(allocator, &key_cur, &iv_cur, &aad_cur);

    struct aws_byte_cursor tag = aws_symmetric_cipher_get_tag(cipher);

    ASSERT_TRUE(tag.len == 0 && tag.ptr == NULL);

    aws_symmetric_cipher_set_tag(cipher, wrong_tag_cur);

    // encrypt
    struct aws_byte_buf encrypt_buf = {0};
    aws_byte_buf_init(&encrypt_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE * 2);
    ASSERT_SUCCESS(aws_symmetric_cipher_encrypt(cipher, data_cur, &encrypt_buf));

    // finalize
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_encryption(cipher, &encrypt_buf));

    if (data_cur.len == 0) {
        ASSERT_INT_EQUALS(0, encrypt_buf.len);
    } else {
        ASSERT_TRUE(encrypt_buf.len > 0);
    }

    struct aws_byte_cursor encryption_tag = aws_symmetric_cipher_get_tag(cipher);

    ASSERT_BIN_ARRAYS_EQUALS(expected_tag_cur.ptr, expected_tag_cur.len, encryption_tag.ptr, encryption_tag.len);

    /* reset and verify decrypt works */
    aws_symmetric_cipher_reset(cipher);
    tag = aws_symmetric_cipher_get_tag(cipher);

    ASSERT_TRUE(tag.len == 0 && tag.ptr == NULL);

    aws_symmetric_cipher_set_tag(cipher, expected_tag_cur);

    struct aws_byte_buf decrypted_buf = {0};
    aws_byte_buf_init(&decrypted_buf, allocator, AWS_AES_256_CIPHER_BLOCK_SIZE);
    struct aws_byte_cursor ciphertext_cur = aws_byte_cursor_from_buf(&encrypt_buf);
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, ciphertext_cur, &decrypted_buf));
    ASSERT_SUCCESS(aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));

    /* reset and verify decrypt with wrong tag fails */
    aws_symmetric_cipher_reset(cipher);
    aws_byte_buf_reset(&decrypted_buf, true);
    aws_symmetric_cipher_set_tag(cipher, wrong_tag_cur);
    ciphertext_cur = aws_byte_cursor_from_buf(&encrypt_buf);
    ASSERT_SUCCESS(aws_symmetric_cipher_decrypt(cipher, ciphertext_cur, &decrypted_buf));
    ASSERT_ERROR(AWS_ERROR_INVALID_ARGUMENT, aws_symmetric_cipher_finalize_decryption(cipher, &decrypted_buf));

    /* reset and verify decrypt with no tag fails */
    aws_symmetric_cipher_reset(cipher);
    aws_byte_buf_reset(&decrypted_buf, true);
    ciphertext_cur = aws_byte_cursor_from_buf(&encrypt_buf);
    ASSERT_ERROR(AWS_ERROR_INVALID_ARGUMENT, aws_symmetric_cipher_decrypt(cipher, ciphertext_cur, &decrypted_buf));

    aws_byte_buf_clean_up(&encrypt_buf);
    aws_byte_buf_clean_up(&decrypted_buf);
    aws_symmetric_cipher_destroy(cipher);
    aws_cal_library_clean_up();
    return AWS_OP_SUCCESS;
}

static int s_aes_test_empty_input_gcm_tag_corner_cases(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {0xFB, 0x7B, 0x4A, 0x82, 0x4E, 0x82, 0xDA, 0xA6, 0xC8, 0xBC, 0x12, 0x51};

    uint8_t key[] = {0x20, 0x14, 0x2E, 0x89, 0x8C, 0xD2, 0xFD, 0x98, 0x0F, 0xBF, 0x34, 0xDE, 0x6B, 0xC8, 0x5C, 0x14,
                     0xDA, 0x7D, 0x57, 0xBD, 0x28, 0xF4, 0xAA, 0x5C, 0xF1, 0x72, 0x8A, 0xB6, 0x4E, 0x84, 0x31, 0x42};

    uint8_t aad[] = {0x16, 0x7B, 0x5C, 0x22, 0x61, 0x77, 0x73, 0x3A, 0x78, 0x2D, 0x61, 0x6D, 0x7A, 0x2D, 0x63, 0x65,
                     0x6B, 0x2D, 0x61, 0x6C, 0x67, 0x5C, 0x22, 0x3A, 0x20, 0x5C, 0x22, 0x41, 0x45, 0x53, 0x2F, 0x47,
                     0x43, 0x4D, 0x2F, 0x4E, 0x6F, 0x50, 0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x5C, 0x22, 0x7D};

    uint8_t expected_tag[] = {
        0x81, 0xC0, 0xE4, 0x2B, 0xB1, 0x95, 0xE2, 0x62, 0xCB, 0x3B, 0x3A, 0x74, 0xA0, 0xDA, 0xE1, 0xC8};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor aad_cur = aws_byte_cursor_from_array(aad, sizeof(aad));
    struct aws_byte_cursor expected_tag_cur = aws_byte_cursor_from_array(expected_tag, sizeof(expected_tag));
    struct aws_byte_cursor data_cur = {0};

    return s_aes_gcm_corner_case_checker(allocator, key_cur, iv_cur, aad_cur, data_cur, expected_tag_cur);
}
AWS_TEST_CASE(aes_test_empty_input_gcm_tag_corner_cases, s_aes_test_empty_input_gcm_tag_corner_cases)

static int s_aes_test_gcm_tag_corner_cases(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {0xFB, 0x7B, 0x4A, 0x82, 0x4E, 0x82, 0xDA, 0xA6, 0xC8, 0xBC, 0x12, 0x51};

    uint8_t key[] = {0x20, 0x14, 0x2E, 0x89, 0x8C, 0xD2, 0xFD, 0x98, 0x0F, 0xBF, 0x34, 0xDE, 0x6B, 0xC8, 0x5C, 0x14,
                     0xDA, 0x7D, 0x57, 0xBD, 0x28, 0xF4, 0xAA, 0x5C, 0xF1, 0x72, 0x8A, 0xB6, 0x4E, 0x84, 0x31, 0x42};

    uint8_t aad[] = {0x16, 0x7B, 0x5C, 0x22, 0x61, 0x77, 0x73, 0x3A, 0x78, 0x2D, 0x61, 0x6D, 0x7A, 0x2D, 0x63, 0x65,
                     0x6B, 0x2D, 0x61, 0x6C, 0x67, 0x5C, 0x22, 0x3A, 0x20, 0x5C, 0x22, 0x41, 0x45, 0x53, 0x2F, 0x47,
                     0x43, 0x4D, 0x2F, 0x4E, 0x6F, 0x50, 0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x5C, 0x22, 0x7D};

    uint8_t data[] = {
        0x84, 0x99, 0x89, 0x3E, 0x16, 0xB0, 0xBA, 0x8B, 0x00, 0x7D, 0x54, 0x66, 0x5A, 0x84, 0x99, 0x89, 0x3E};

    uint8_t expected_tag[] = {
        0x76, 0x4D, 0x21, 0xD6, 0xC0, 0xD8, 0xC7, 0xF9, 0xCA, 0x6D, 0xF2, 0x19, 0xAE, 0x56, 0xDC, 0x1F};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor aad_cur = aws_byte_cursor_from_array(aad, sizeof(aad));
    struct aws_byte_cursor expected_tag_cur = aws_byte_cursor_from_array(expected_tag, sizeof(expected_tag));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));

    return s_aes_gcm_corner_case_checker(allocator, key_cur, iv_cur, aad_cur, data_cur, expected_tag_cur);
}
AWS_TEST_CASE(aes_test_gcm_tag_corner_cases, s_aes_test_gcm_tag_corner_cases)

static int s_aes_test_gcm_tag_large_input_corner_cases(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    uint8_t iv[] = {0xFB, 0x7B, 0x4A, 0x82, 0x4E, 0x82, 0xDA, 0xA6, 0xC8, 0xBC, 0x12, 0x51};

    uint8_t key[] = {0x20, 0x14, 0x2E, 0x89, 0x8C, 0xD2, 0xFD, 0x98, 0x0F, 0xBF, 0x34, 0xDE, 0x6B, 0xC8, 0x5C, 0x14,
                     0xDA, 0x7D, 0x57, 0xBD, 0x28, 0xF4, 0xAA, 0x5C, 0xF1, 0x72, 0x8A, 0xB6, 0x4E, 0x84, 0x31, 0x42};

    uint8_t aad[] = {0x16, 0x7B, 0x5C, 0x22, 0x61, 0x77, 0x73, 0x3A, 0x78, 0x2D, 0x61, 0x6D, 0x7A, 0x2D, 0x63, 0x65,
                     0x6B, 0x2D, 0x61, 0x6C, 0x67, 0x5C, 0x22, 0x3A, 0x20, 0x5C, 0x22, 0x41, 0x45, 0x53, 0x2F, 0x47,
                     0x43, 0x4D, 0x2F, 0x4E, 0x6F, 0x50, 0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x5C, 0x22, 0x7D};

    uint8_t data[] = {0x84, 0x99, 0x89, 0x3E, 0x16, 0xB0, 0xBA, 0x8B, 0x00, 0x7D, 0x54, 0x66, 0x5A,
                      0x84, 0x99, 0x89, 0x3E, 0x84, 0x99, 0x89, 0x3E, 0x16, 0xB0, 0xBA, 0x8B, 0x00,
                      0x7D, 0x54, 0x66, 0x5A, 0x84, 0x99, 0x89, 0x3E, 0x84, 0x99, 0x89, 0x3E, 0x16,
                      0xB0, 0xBA, 0x8B, 0x00, 0x7D, 0x54, 0x66, 0x5A, 0x84, 0x99, 0x89, 0x3E};

    uint8_t expected_tag[] = {
        0xEA, 0x5E, 0x8A, 0x4B, 0x76, 0xE8, 0x9D, 0xC5, 0xF1, 0x32, 0x14, 0x64, 0xD0, 0x93, 0x74, 0xB7};

    struct aws_byte_cursor key_cur = aws_byte_cursor_from_array(key, sizeof(key));
    struct aws_byte_cursor iv_cur = aws_byte_cursor_from_array(iv, sizeof(iv));
    struct aws_byte_cursor aad_cur = aws_byte_cursor_from_array(aad, sizeof(aad));
    struct aws_byte_cursor expected_tag_cur = aws_byte_cursor_from_array(expected_tag, sizeof(expected_tag));
    struct aws_byte_cursor data_cur = aws_byte_cursor_from_array(data, sizeof(data));

    return s_aes_gcm_corner_case_checker(allocator, key_cur, iv_cur, aad_cur, data_cur, expected_tag_cur);
}
AWS_TEST_CASE(aes_test_gcm_tag_large_input_corner_cases, s_aes_test_gcm_tag_large_input_corner_cases)
