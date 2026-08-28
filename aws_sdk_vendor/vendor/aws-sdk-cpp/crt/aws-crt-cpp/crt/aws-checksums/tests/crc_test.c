/**
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aws/checksums/checksums.h>
#include <aws/checksums/crc.h>
#include <aws/checksums/private/crc32_priv.h>
#include <aws/checksums/private/crc_util.h>

#include <aws/common/device_random.h>

#include <aws/testing/aws_test_harness.h>

static const uint8_t DATA_32_ZEROS[32] = {0};
static const uint8_t DATA_32_VALUES[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
                                           16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};

static const uint8_t TEST_VECTOR[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

// The polynomial used for CRC32 (in bit-reflected form)
static const uint32_t POLY_CRC32 = 0xedb88320;
// Any input with the CRC32 of that input appended should produce this CRC32 value. (Note: inverting the bits)
static const uint32_t RESIDUE_CRC32 = ~0xdebb20e3;
static const uint32_t KNOWN_CRC32_32_ZEROES = 0x190A55AD;
static const uint32_t KNOWN_CRC32_32_VALUES = 0x91267E8A;
static const uint32_t KNOWN_CRC32_TEST_VECTOR = 0xCBF43926;

// The polynomial used for CRC32C (in bit-reflected form)
static const uint32_t POLY_CRC32C = 0x82f63b78;
// Any input with the CRC32c of that input appended should produce this CRC32c value. (Note: inverting the bits)
static const uint32_t RESIDUE_CRC32C = ~0xb798b438;
static const uint32_t KNOWN_CRC32C_32_ZEROES = 0x8A9136AA;
static const uint32_t KNOWN_CRC32C_32_VALUES = 0x46DD794E;
static const uint32_t KNOWN_CRC32C_TEST_VECTOR = 0xE3069283;

typedef uint32_t(crc_fn)(const uint8_t *input, int length, uint32_t previousCrc32);
#define CRC_FUNC_NAME(crc_func) #crc_func, crc_func
#define DATA_NAME(dataset) #dataset, dataset, sizeof(dataset)
#define TEST_BUFFER_SIZE 2048 + 64

// Slow reference implementation that computes a 32-bit bit-reflected/bit-inverted CRC using the provided polynomial.
static uint32_t s_crc_32_reference(const uint8_t *input, int length, const uint32_t previousCrc, uint32_t polynomial) {
    uint32_t crc = ~previousCrc;
    while (length-- > 0) {
        crc ^= *input++;
        for (int j = 8; j > 0; --j) {
            crc = (crc >> 1) ^ ((((crc & 1) ^ 1) - 1) & polynomial);
        }
    }
    return ~crc;
}

// Very, very slow reference implementation that computes a CRC32.
static uint32_t s_crc32_reference(const uint8_t *input, int length, const uint32_t previousCrc) {
    return s_crc_32_reference(input, length, previousCrc, POLY_CRC32);
}

// Very, very slow reference implementation that computes a CRC32c.
static uint32_t s_crc32c_reference(const uint8_t *input, int length, const uint32_t previousCrc) {
    return s_crc_32_reference(input, length, previousCrc, POLY_CRC32C);
}

/* Makes sure that the specified crc function produces the expected results for known input and output */
static int s_test_known_crc_32(
    const char *func_name,
    crc_fn *func,
    const char *data_name,
    const uint8_t *input,
    const size_t length,
    const uint32_t expected_crc,
    const uint32_t expected_residue) {

    uint32_t result = func(input, (int)length, 0);
    ASSERT_HEX_EQUALS(expected_crc, result, "%s(%s)", func_name, data_name);

    uint32_t result_le = aws_bswap32_if_be(result);
    // Compute the residue of the buffer (the CRC of the buffer plus its CRC) - will always be a constant value
    uint32_t residue = (uint32_t)func((const uint8_t *)&result_le, 4, result); // assuming little endian
    ASSERT_HEX_EQUALS(expected_residue, residue, "len %d residue %s(%s)", length, func_name, data_name);

    // chain the crc computation so 2 calls each operate on about 1/2 of the buffer
    uint32_t crc1 = func(input, (int)(length / 2), 0);
    result = func(input + (length / 2), (int)(length - length / 2), crc1);
    ASSERT_HEX_EQUALS(expected_crc, result, "chaining %s(%s)", func_name, data_name);

    crc1 = 0;
    for (size_t i = 0; i < length; ++i) {
        crc1 = func(input + i, 1, crc1);
    }
    ASSERT_HEX_EQUALS(expected_crc, crc1, "one byte at a time %s(%s)", func_name, data_name);

    return AWS_OP_SUCCESS;
}

/* helper function that tests increasing input data lengths vs the reference crc function */
static int s_test_vs_reference_crc_32(
    struct aws_allocator *allocator,
    uint32_t polynomial,
    uint32_t residue,
    const char *func_name,
    crc_fn *func) {

    int res = 0;

    struct aws_byte_buf test_buf;
    ASSERT_SUCCESS(aws_byte_buf_init(&test_buf, allocator, TEST_BUFFER_SIZE));

    // Spin through buffer offsets
    for (int off = 0; off < 16; off++) {
        // Fill the test buffer with different values for each iteration
        aws_byte_buf_write_u8_n(&test_buf, (uint8_t)off + 129, test_buf.capacity - test_buf.len);
        uint32_t expected = 0;
        int len = 1;
        // Spin through input data lengths
        for (int i = 0; i < (TEST_BUFFER_SIZE - off) && !res; i++, len++) {
            test_buf.buffer[off + i] = (uint8_t)((i + 1) * 131);
            // Compute the expected CRC one byte at a time using the reference function
            expected = s_crc_32_reference(&test_buf.buffer[off + i], 1, expected, polynomial);
            // Recompute the full CRC of the buffer at each offset and length and compare against expected value
            res |= s_test_known_crc_32(func_name, func, "test_buffer", &test_buf.buffer[off], len, expected, residue);
            if (res != 0) {
                continue;
            }
        }
        aws_byte_buf_reset(&test_buf, false);
    }
    aws_byte_buf_clean_up(&test_buf);

    return res;
}

/* helper function that groups crc32 tests*/
static int s_test_known_crc32(struct aws_allocator *allocator, const char *func_name, crc_fn *func) {
    int res = 0;
    res |= s_test_known_crc_32(func_name, func, DATA_NAME(DATA_32_ZEROS), KNOWN_CRC32_32_ZEROES, RESIDUE_CRC32);
    res |= s_test_known_crc_32(func_name, func, DATA_NAME(DATA_32_VALUES), KNOWN_CRC32_32_VALUES, RESIDUE_CRC32);
    res |= s_test_known_crc_32(func_name, func, DATA_NAME(TEST_VECTOR), KNOWN_CRC32_TEST_VECTOR, RESIDUE_CRC32);
    if (func != s_crc32_reference) {
        res |= s_test_vs_reference_crc_32(allocator, POLY_CRC32, RESIDUE_CRC32, func_name, func);
    }
    return res;
}

/* helper function that groups crc32c tests*/
static int s_test_known_crc32c(struct aws_allocator *allocator, const char *func_name, crc_fn *func) {
    int res = 0;

    res |= s_test_known_crc_32(func_name, func, DATA_NAME(DATA_32_ZEROS), KNOWN_CRC32C_32_ZEROES, RESIDUE_CRC32C);
    res |= s_test_known_crc_32(func_name, func, DATA_NAME(DATA_32_VALUES), KNOWN_CRC32C_32_VALUES, RESIDUE_CRC32C);
    res |= s_test_known_crc_32(func_name, func, DATA_NAME(TEST_VECTOR), KNOWN_CRC32C_TEST_VECTOR, RESIDUE_CRC32C);
    if (func != s_crc32c_reference) {
        res |= s_test_vs_reference_crc_32(allocator, POLY_CRC32C, RESIDUE_CRC32C, func_name, func);
    }

    return res;
}

/**
 * Quick sanity check of some known CRC values for known input.
 * The reference functions are included in these tests to verify that they aren't obviously broken.
 */
static int s_test_crc32c(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    int res = 0;

    res |= s_test_known_crc32c(allocator, CRC_FUNC_NAME(s_crc32c_reference));
    res |= s_test_known_crc32c(allocator, CRC_FUNC_NAME(aws_checksums_crc32c_sw));
    res |= s_test_known_crc32c(allocator, CRC_FUNC_NAME(aws_checksums_crc32c));

    return res;
}
AWS_TEST_CASE(test_crc32c, s_test_crc32c)

static int s_test_crc32c_init(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_checksums_library_init(allocator);

    int res = 0;

    res |= s_test_known_crc32c(allocator, CRC_FUNC_NAME(s_crc32c_reference));
    res |= s_test_known_crc32c(allocator, CRC_FUNC_NAME(aws_checksums_crc32c_sw));
    res |= s_test_known_crc32c(allocator, CRC_FUNC_NAME(aws_checksums_crc32c));

    aws_checksums_library_clean_up();

    return res;
}
AWS_TEST_CASE(test_crc32c_init, s_test_crc32c_init)

static int s_test_crc32(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    int res = 0;

    res |= s_test_known_crc32(allocator, CRC_FUNC_NAME(s_crc32_reference));
    res |= s_test_known_crc32(allocator, CRC_FUNC_NAME(aws_checksums_crc32_sw));
    res |= s_test_known_crc32(allocator, CRC_FUNC_NAME(aws_checksums_crc32));

    return res;
}
AWS_TEST_CASE(test_crc32, s_test_crc32)

static int s_test_crc32_init(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;

    aws_checksums_library_init(allocator);

    int res = 0;

    res |= s_test_known_crc32(allocator, CRC_FUNC_NAME(s_crc32_reference));
    res |= s_test_known_crc32(allocator, CRC_FUNC_NAME(aws_checksums_crc32_sw));
    res |= s_test_known_crc32(allocator, CRC_FUNC_NAME(aws_checksums_crc32));

    aws_checksums_library_clean_up();

    return res;
}
AWS_TEST_CASE(test_crc32_init, s_test_crc32_init)

static int s_test_large_buffer_crc32(struct aws_allocator *allocator, void *ctx) {
    (void)ctx;
#if SIZE_BITS == 32 || defined(__OpenBSD__) /* openbsd fails to allocate big buffer */
    (void)allocator;
    return AWS_OP_SKIP;
#else
    const size_t len = 3 * 1024 * 1024 * 1024ULL;
    const uint8_t *many_zeroes = aws_mem_calloc(allocator, len, sizeof(uint8_t));
    uint32_t result = aws_checksums_crc32_ex(many_zeroes, len, 0);
    aws_mem_release(allocator, (void *)many_zeroes);
    ASSERT_HEX_EQUALS(0x480BBE37, result);
    return AWS_OP_SUCCESS;
#endif
}
AWS_TEST_CASE(test_large_buffer_crc32, s_test_large_buffer_crc32)
