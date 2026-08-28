/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/checksums/checksums.h>
#include <aws/checksums/crc.h>
#include <aws/checksums/private/crc64_priv.h>
#include <aws/checksums/private/crc_util.h>
#include <aws/common/encoding.h>
#include <aws/testing/aws_test_harness.h>

// The polynomial used for CRC64NVME (in bit-reflected form)
static const uint64_t POLY_CRC64NVME = 0x9a6c9329ac4bc9b5;

// Any input with the CRC of that input appended should produce this CRC value. (Note: inverting the bits)
static const uint64_t RESIDUE_CRC64NVME = (uint64_t)~0xf310303b2b6f6e42;

static const uint8_t DATA_32_ZEROS[32] = {0};
static const uint64_t KNOWN_CRC64NVME_32_ZEROES = 0xCF3473434D4ECF3B;

static const uint8_t DATA_32_VALUES[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
                                           16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
static const uint64_t KNOWN_CRC64NVME_32_VALUES = 0xB9D9D4A8492CBD7F;

static const uint8_t TEST_VECTOR[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
static const uint64_t KNOWN_CRC64NVME_TEST_VECTOR = 0xAE8B14860A799888;

typedef uint64_t(crc_fn)(const uint8_t *input, int length, uint64_t previousCrc64);
#define CRC_FUNC_NAME(crc_func) #crc_func, crc_func
#define DATA_NAME(dataset) #dataset, dataset, sizeof(dataset)
#define TEST_BUFFER_SIZE 2048 + 64

// Very, very slow reference implementation that computes CRC64NVME.
static uint64_t crc64nvme_reference(const uint8_t *input, int length, const uint64_t previousCrc64) {
    uint64_t crc = ~previousCrc64;
    while (length-- > 0) {
        crc ^= *input++;
        for (int j = 8; j > 0; --j) {
            crc = (crc >> 1) ^ ((((crc & 1) ^ 1) - 1) & POLY_CRC64NVME);
        }
    }
    return ~crc;
}

/* Makes sure that the specified crc function produces the expected results for known input and output */
static int s_test_known_crc(
    const char *func_name,
    crc_fn *func,
    const char *data_name,
    const uint8_t *input,
    const size_t length,
    const uint64_t expected_crc,
    const uint64_t expected_residue) {

    uint64_t result = func(input, (int)length, 0);
    ASSERT_HEX_EQUALS(expected_crc, result, "%s(%s)", func_name, data_name);

    // Compute the residue of the buffer (the CRC of the buffer plus its CRC) - will always be a constant value
    uint64_t result_le = aws_bswap64_if_be(result);
    uint64_t residue = func((const uint8_t *)&result_le, 8, result); // assuming little endian
    ASSERT_HEX_EQUALS(expected_residue, residue, "len %d residue %s(%s)", length, func_name, data_name);

    // chain the crc computation so 2 calls each operate on about 1/2 of the buffer
    uint64_t crc1 = func(input, (int)(length / 2), 0);
    result = func(input + (length / 2), (int)(length - length / 2), crc1);
    ASSERT_HEX_EQUALS(expected_crc, result, "chaining %s(%s)", func_name, data_name);

    crc1 = 0;
    for (size_t i = 0; i < length; ++i) {
        crc1 = func(input + i, 1, crc1);
    }
    ASSERT_HEX_EQUALS(expected_crc, crc1, "one byte at a time %s(%s)", func_name, data_name);

    return AWS_OP_SUCCESS;
}

/* helper function that groups crc64nvme tests */
static int s_test_known_crc64nvme(struct aws_allocator *allocator, const char *func_name, crc_fn *func) {
    int res = 0;

    // Quick sanity check of some known CRC values for known input.
    res |= s_test_known_crc(func_name, func, DATA_NAME(DATA_32_ZEROS), KNOWN_CRC64NVME_32_ZEROES, RESIDUE_CRC64NVME);
    res |= s_test_known_crc(func_name, func, DATA_NAME(DATA_32_VALUES), KNOWN_CRC64NVME_32_VALUES, RESIDUE_CRC64NVME);
    res |= s_test_known_crc(func_name, func, DATA_NAME(TEST_VECTOR), KNOWN_CRC64NVME_TEST_VECTOR, RESIDUE_CRC64NVME);

    if (func == crc64nvme_reference) {
        // Don't proceed further since we'd just be testing the reference function against itself
        return res;
    }

    struct aws_byte_buf test_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&test_buf, allocator, TEST_BUFFER_SIZE));

    // Spin through buffer offsets
    for (int off = 0; off < 16; off++) {
        // Fill the test buffer with different values for each iteration
        aws_byte_buf_write_u8_n(&test_buf, (uint8_t)off + 129, test_buf.capacity - test_buf.len);
        uint64_t expected = 0;
        int len = 1;
        // Spin through input data lengths
        for (int i = 0; i < (TEST_BUFFER_SIZE - off) && !res; i++, len++) {
            test_buf.buffer[off + i] = (uint8_t)((i + 1) * 131);
            // Compute the expected CRC one byte at a time using the reference function
            expected = crc64nvme_reference(&test_buf.buffer[off + i], 1, expected);
            // Recompute the full CRC of the buffer at each offset and length and compare against expected value
            res |= s_test_known_crc(
                func_name, func, "test_buffer", &test_buf.buffer[off], len, expected, RESIDUE_CRC64NVME);
        }
        aws_byte_buf_reset(&test_buf, false);
    }
    aws_byte_buf_clean_up(&test_buf);

    return res;
}

/**
 * The reference functions are included in these tests to verify that they aren't obviously broken.
 */
static int s_test_crc64nvme(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    int res = 0;

    res |= s_test_known_crc64nvme(allocator, CRC_FUNC_NAME(crc64nvme_reference));
    res |= s_test_known_crc64nvme(allocator, CRC_FUNC_NAME(aws_checksums_crc64nvme_sw));
    res |= s_test_known_crc64nvme(allocator, CRC_FUNC_NAME(aws_checksums_crc64nvme));

    return res;
}

AWS_TEST_CASE(test_crc64nvme, s_test_crc64nvme)

static int s_test_crc64nvme_init(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_checksums_library_init(allocator);

    int res = 0;

    res |= s_test_known_crc64nvme(allocator, CRC_FUNC_NAME(crc64nvme_reference));
    res |= s_test_known_crc64nvme(allocator, CRC_FUNC_NAME(aws_checksums_crc64nvme_sw));
    res |= s_test_known_crc64nvme(allocator, CRC_FUNC_NAME(aws_checksums_crc64nvme));

    aws_checksums_library_clean_up();

    return res;
}
AWS_TEST_CASE(test_crc64nvme_init, s_test_crc64nvme_init)

static int s_test_large_buffer_crc64(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
#if SIZE_BITS == 32 || defined(__OpenBSD__) /* openbsd fails to allocate big buffer */
    (void)allocator;
    return AWS_OP_SKIP;
#else
    const size_t len = 3 * 1024 * 1024 * 1024ULL;
    const uint8_t *many_zeroes = aws_mem_calloc(allocator, len, sizeof(uint8_t));
    uint64_t result = aws_checksums_crc64nvme_ex(many_zeroes, len, 0);
    aws_mem_release(allocator, (void *)many_zeroes);
    ASSERT_HEX_EQUALS(0xa1dddd7c6fd17075, result);
    return AWS_OP_SUCCESS;
#endif
}
AWS_TEST_CASE(test_large_buffer_crc64, s_test_large_buffer_crc64)
